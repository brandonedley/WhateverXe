#include "kern_display13_wake_gate.hpp"

#include <cassert>
#include <cstdio>

namespace {

struct MockGateOps {
    bool alreadyInGate {};
    unsigned failAttempts {};
    unsigned tryCalls {};
    unsigned delayCalls {};
    unsigned sleepCalls {};

    bool inGate() const { return alreadyInGate; }
    bool tryCloseGate() {
        ++tryCalls;
        return tryCalls > failAttempts;
    }
    void delayUs(unsigned) { ++delayCalls; }
    void sleepMs(unsigned) { ++sleepCalls; }
};

struct MockBarIO {
    unsigned invalidReads {};
    uint32_t settledValue {0x3};
    unsigned reads {};
    unsigned sleepCalls {};

    uint32_t read(uint32_t) {
        ++reads;
        return reads <= invalidReads ? UINT32_MAX : settledValue;
    }
    void sleepMs(unsigned) { ++sleepCalls; }
};

struct MockFuseIO {
    unsigned unreadyReads {};
    uint32_t readyValue {1U << 27};
    unsigned reads {};
    unsigned delayCalls {};
    unsigned sleepCalls {};

    uint32_t read(uint32_t) {
        ++reads;
        return reads <= unreadyReads ? 0U : readyValue;
    }
    void delayUs(unsigned) { ++delayCalls; }
    void sleepMs(unsigned) { ++sleepCalls; }
};

} // namespace

int main() {
    using namespace Display13Wake;

    // 1. Caller already in gate: zero tries, not acquired.
    {
        MockGateOps ops {true, 100};
        const auto r = acquireGate(ops);
        assert(r.ok && r.inGate && !r.acquired && r.retries == 0);
        assert(ops.tryCalls == 0 && ops.delayCalls == 0 && ops.sleepCalls == 0);
    }

    // 2. Immediate acquisition on first try.
    {
        MockGateOps ops {false, 0};
        const auto r = acquireGate(ops);
        assert(r.ok && !r.inGate && r.acquired && r.retries == 0);
        assert(ops.tryCalls == 1 && ops.delayCalls == 0 && ops.sleepCalls == 0);
    }

    // 3. Contention cleared during fast spin phase.
    {
        MockGateOps ops {false, 3};
        const auto r = acquireGate(ops);
        assert(r.ok && r.acquired && r.retries == 3);
        assert(ops.tryCalls == 4 && ops.delayCalls == 3 && ops.sleepCalls == 0);
    }

    // 4. Contention cleared during ms sleep phase (multi-ms wake overlap).
    {
        MockGateOps ops {false, 25};
        const auto r = acquireGate(ops);
        assert(r.ok && r.acquired && r.retries == 25);
        assert(ops.tryCalls == 26 && ops.delayCalls == DefaultFastSpins && ops.sleepCalls == 20);
    }

    // 5. Acquisition on the very last attempt succeeds.
    {
        const unsigned lastAttempt = DefaultFastSpins + DefaultMaxGateWaitMs;
        MockGateOps ops {false, lastAttempt};
        const auto r = acquireGate(ops);
        assert(r.ok && r.acquired && r.retries == lastAttempt);
        assert(ops.tryCalls == lastAttempt + 1 && ops.sleepCalls == DefaultMaxGateWaitMs);
    }

    // 6. Permanent contention bounds wait (deadlock-free timeout).
    {
        MockGateOps ops {false, 999999};
        const auto r = acquireGate(ops);
        assert(!r.ok && !r.acquired && !r.inGate);
        assert(r.retries == DefaultFastSpins + DefaultMaxGateWaitMs + 1);
        assert(ops.delayCalls == DefaultFastSpins && ops.sleepCalls == DefaultMaxGateWaitMs);
    }

    // 7. BAR0 immediately valid: zero sleeps.
    {
        MockBarIO io {0, 0x403};
        uint32_t v = 0;
        const auto r = waitBarReady(io, 0x45404, v);
        assert(r.ok && r.retries == 0 && v == 0x403 && io.sleepCalls == 0);
    }

    // 8. BAR0 settles after 12 ms D3 -> D0 wake delay.
    {
        MockBarIO io {12, 0x3};
        uint32_t v = 0;
        const auto r = waitBarReady(io, 0x45404, v);
        assert(r.ok && r.retries == 12 && v == 0x3 && io.sleepCalls == 12);
    }

    // 9. BAR0 remains 0xFFFFFFFF: bounded timeout.
    {
        MockBarIO io {999, 0x3};
        uint32_t v = 0;
        const auto r = waitBarReady(io, 0x45404, v);
        assert(!r.ok && r.retries == DefaultMaxBarWaitMs && v == UINT32_MAX);
    }

    // 10. PG0 fuse immediately ready.
    {
        MockFuseIO io {0, 1U << 27};
        uint32_t f = 0;
        const auto r = waitPG0Fuse(io, 0x42000, 1U << 27, f);
        assert(r.ok && r.retries == 0 && f == (1U << 27) && io.delayCalls == 0 && io.sleepCalls == 0);
    }

    // 11. PG0 fuse settles after fast spins + 2 ms sleep.
    {
        MockFuseIO io {7, 0xF8000000U};
        uint32_t f = 0;
        const auto r = waitPG0Fuse(io, 0x42000, 1U << 27, f);
        assert(r.ok && r.retries == 7 && io.delayCalls == DefaultPG0FastSpins && io.sleepCalls == 2);
    }

    // 12. PG0 fuse timeout bounds wait.
    {
        MockFuseIO io {999, 1U << 27};
        uint32_t f = 0;
        const auto r = waitPG0Fuse(io, 0x42000, 1U << 27, f);
        assert(!r.ok && r.retries == DefaultPG0FastSpins + DefaultPG0MaxWaitMs);
    }

    // 13. Stats aggregation.
    {
        Stats s {};
        s.recordGate({true, true, false, 4});
        s.recordGate({true, true, false, 19});
        s.recordGate({false, false, false, 506});
        s.recordBar({true, 3});
        s.recordPG0({true, 2});
        s.recordRestoreFailure(4, 6);
        assert(s.wakeCalls == 3 && s.gateRetriesTotal == 529 && s.gateMaxRetries == 506 && s.gateTimeouts == 1);
        assert(s.barRetriesTotal == 3 && s.barTimeouts == 0);
        assert(s.pg0RetriesTotal == 2 && s.pg0Timeouts == 0);
        assert(s.restoreFailures == 1 && s.lastFailureStage == 4 && s.lastFailureStatus == 6);
    }

    std::printf("display13_wake_gate: ok (13 test groups passed)\n");
    return 0;
}
