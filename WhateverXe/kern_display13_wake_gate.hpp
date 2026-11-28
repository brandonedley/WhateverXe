// Deadlock-free bounded workLoop gate acquisition and wake settling helpers
// for Display13 sleep/wake transitions (prepareDisplay13Wake /
// wrapAlderLakeNPowerRestore / stopDisplay13Engine /
// wrapAlderLakeNPowerOverride).
//
// Why this exists:
//   IOFramebuffer::checkPowerWork discards the return code of
//   setAttribute(\x27pwrs\x27, 2) and never retries. Returning kIOReturnNotReady
//   from wrapDisplay13SetPowerState(fb, 2) leaves fb+0x199 bit 0x20
//   (fPagingState) permanently clear, causing WindowServer to block
//   unkillably in IOFramebuffer::_extEntry -> sleepGate(&fb->0x188) until
//   the 120 s userspace watchdog kernel panic.
//
//   Because prepareDisplay13Wake runs on IOGraphicsSystem while concurrent
//   controller wake work (hwRestoreState, hwEnableInterrupts, bklt, HPD)
//   may hold controller->workLoop (c.workLoop) for 1-50 ms, a single-shot
//   tryCloseGate() fails deterministically on any overlap. Meanwhile
//   unbounded blocking closeGate() while holding c.lock or IOGraphicsSystem
//   gate risks ABBA deadlock.
//
//   Dropping c.lock and acquiring c.workLoop with a bounded non-blocking
//   retry loop preserves canonical lock order (c.workLoop -> c.lock),
//   survives multi-ms contention, and bounds worst-case wait.
#ifndef kern_display13_wake_gate_hpp
#define kern_display13_wake_gate_hpp

#include <stdint.h>

namespace Display13Wake {

constexpr unsigned DefaultFastSpins = 5;
constexpr unsigned DefaultFastDelayUs = 100;
constexpr unsigned DefaultMaxGateWaitMs = 500;
constexpr unsigned DefaultMaxBarWaitMs = 50;
constexpr unsigned DefaultPG0FastSpins = 5;
constexpr unsigned DefaultPG0FastDelayUs = 100;
constexpr unsigned DefaultPG0MaxWaitMs = 5;

struct GateResult {
    bool ok {};
    bool acquired {};
    bool inGate {};
    uint32_t retries {};
};

struct WaitResult {
    bool ok {};
    uint32_t retries {};
};

struct Stats {
    uint32_t version {1};
    uint32_t wakeCalls {};
    uint32_t gateRetriesTotal {};
    uint32_t gateMaxRetries {};
    uint32_t gateTimeouts {};
    uint32_t barRetriesTotal {};
    uint32_t barTimeouts {};
    uint32_t pg0RetriesTotal {};
    uint32_t pg0Timeouts {};
    uint32_t restoreFailures {};
    uint32_t lastFailureStage {};
    uint32_t lastFailureStatus {};

    void recordGate(const GateResult &r) {
        ++wakeCalls;
        gateRetriesTotal += r.retries;
        if (r.retries > gateMaxRetries) gateMaxRetries = r.retries;
        if (!r.ok) ++gateTimeouts;
    }

    void recordBar(const WaitResult &r) {
        barRetriesTotal += r.retries;
        if (!r.ok) ++barTimeouts;
    }

    void recordPG0(const WaitResult &r) {
        pg0RetriesTotal += r.retries;
        if (!r.ok) ++pg0Timeouts;
    }

    void recordRestoreFailure(uint32_t stage, uint32_t status) {
        ++restoreFailures;
        lastFailureStage = stage;
        lastFailureStatus = status;
    }
};

// Bounded non-blocking retry loop for IOWorkLoop gate acquisition.
// Caller MUST NOT hold c.lock when invoking this function.
template<class GateOps>
inline GateResult acquireGate(GateOps &ops,
                              unsigned fastSpins = DefaultFastSpins,
                              unsigned fastDelayUs = DefaultFastDelayUs,
                              unsigned maxSleepMs = DefaultMaxGateWaitMs) {
    GateResult r {};
    if (ops.inGate()) {
        r.ok = true;
        r.inGate = true;
        return r;
    }
    const unsigned totalAttempts = fastSpins + maxSleepMs + 1;
    for (unsigned attempt = 0; attempt < totalAttempts; ++attempt) {
        if (ops.tryCloseGate()) {
            r.ok = true;
            r.acquired = true;
            r.retries = attempt;
            return r;
        }
        if (attempt < fastSpins) {
            ops.delayUs(fastDelayUs);
        } else if (attempt + 1 < totalAttempts) {
            ops.sleepMs(1);
        }
    }
    r.retries = totalAttempts;
    return r;
}

// Bounded wait for BAR0 MMIO readback != 0xFFFFFFFF after PCI D3 -> D0 wake.
// Caller MUST NOT hold c.lock when invoking this function.
template<class IO>
inline WaitResult waitBarReady(IO &io,
                               uint32_t reg,
                               uint32_t &outValue,
                               unsigned maxSleepMs = DefaultMaxBarWaitMs) {
    WaitResult r {};
    outValue = io.read(reg);
    if (outValue != UINT32_MAX) {
        r.ok = true;
        return r;
    }
    for (unsigned ms = 1; ms <= maxSleepMs; ++ms) {
        io.sleepMs(1);
        outValue = io.read(reg);
        r.retries = ms;
        if (outValue != UINT32_MAX) {
            r.ok = true;
            return r;
        }
    }
    return r;
}

// Bounded post-DC9-exit wait for PG0 fuse distribution (SKL_FUSE_STATUS bit 27).
template<class IO>
inline WaitResult waitPG0Fuse(IO &io,
                              uint32_t fuseReg,
                              uint32_t pg0Mask,
                              uint32_t &outFuses,
                              unsigned fastSpins = DefaultPG0FastSpins,
                              unsigned fastDelayUs = DefaultPG0FastDelayUs,
                              unsigned maxSleepMs = DefaultPG0MaxWaitMs) {
    WaitResult r {};
    outFuses = io.read(fuseReg);
    if (outFuses != UINT32_MAX && (outFuses & pg0Mask) == pg0Mask) {
        r.ok = true;
        return r;
    }
    const unsigned totalWaits = fastSpins + maxSleepMs;
    for (unsigned step = 1; step <= totalWaits; ++step) {
        if (step <= fastSpins) {
            io.delayUs(fastDelayUs);
        } else {
            io.sleepMs(1);
        }
        outFuses = io.read(fuseReg);
        r.retries = step;
        if (outFuses != UINT32_MAX && (outFuses & pg0Mask) == pg0Mask) {
            r.ok = true;
            return r;
        }
    }
    return r;
}

} // namespace Display13Wake

#endif
