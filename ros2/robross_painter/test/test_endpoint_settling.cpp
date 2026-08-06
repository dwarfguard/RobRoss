#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "endpoint_settling.hpp"

using robross_painter::SettleConfig;
using robross_painter::SettleGate;
using robross_painter::SettleOutcome;
using robross_painter::SettleSample;

namespace
{

SettleConfig makeConfig(double dwell, double timeout, double max_age,
                        int required_samples)
{
    SettleConfig cfg;
    cfg.dwell = dwell;
    cfg.timeout = timeout;
    cfg.sample_max_age = max_age;
    cfg.required_samples = required_samples;
    return cfg;
}

// Endpoint reached and held at rest.
SettleSample qualifying(std::uint64_t seq, double t)
{
    SettleSample s;
    s.sequence = seq;
    s.receipt_time = t;
    s.valid = true;
    s.endpoint_ok = true;
    s.stationary = true;
    s.error_ratio = 0.1;
    return s;
}

// At/near the target geometrically but still moving (feedback lag / fly-through).
SettleSample moving(std::uint64_t seq, double t)
{
    SettleSample s = qualifying(seq, t);
    s.stationary = false;
    return s;
}

// Stopped, but at the wrong location.
SettleSample offTarget(std::uint64_t seq, double t)
{
    SettleSample s = qualifying(seq, t);
    s.endpoint_ok = false;
    s.error_ratio = 3.0;
    return s;
}

// Missing / non-finite position or velocity: fails closed.
SettleSample invalid(std::uint64_t seq, double t)
{
    SettleSample s;
    s.sequence = seq;
    s.receipt_time = t;
    s.valid = false;
    return s;
}

}  // namespace

// 1. Initial lag then convergence: several moving samples, then a steady hold
//    that satisfies both the dwell and the sample count.
TEST(SettleGate, LagThenConvergenceSettles)
{
    SettleGate gate(makeConfig(0.05, 1.0, 0.02, 6), 0.0, 0);
    std::uint64_t seq = 1;
    // 5 samples in tolerance but still moving (0.00..0.04).
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(gate.offer(moving(seq, 0.01 * seq)), SettleOutcome::Continue);
        ++seq;
    }
    // Now stationary and in tolerance: settles once span >= dwell AND count >= 6.
    bool settled = false;
    for (int i = 0; i < 12; ++i) {
        if (gate.offer(qualifying(seq, 0.01 * seq)) == SettleOutcome::Settled) {
            settled = true;
            break;
        }
        ++seq;
    }
    EXPECT_TRUE(settled);
}

// 2. Endpoint fly-through: in tolerance but never stationary -> times out.
TEST(SettleGate, FlyThroughNeverSettles)
{
    SettleGate gate(makeConfig(0.05, 0.2, 0.02, 6), 0.0, 0);
    bool timed_out = false;
    for (std::uint64_t seq = 1; seq <= 40; ++seq) {
        const auto outcome = gate.offer(moving(seq, 0.01 * seq));
        EXPECT_NE(outcome, SettleOutcome::Settled);
        if (outcome == SettleOutcome::TimedOut) {
            timed_out = true;
            break;
        }
    }
    EXPECT_TRUE(timed_out);
}

// 3. Stationary at the wrong endpoint: stops but out of tolerance -> times out.
TEST(SettleGate, StationaryWrongEndpointNeverSettles)
{
    SettleGate gate(makeConfig(0.05, 0.2, 0.02, 6), 0.0, 0);
    bool timed_out = false;
    for (std::uint64_t seq = 1; seq <= 40; ++seq) {
        const auto outcome = gate.offer(offTarget(seq, 0.01 * seq));
        EXPECT_NE(outcome, SettleOutcome::Settled);
        if (outcome == SettleOutcome::TimedOut) {
            timed_out = true;
            break;
        }
    }
    EXPECT_TRUE(timed_out);
}

// 4. A single invalid sample mid-hold resets the dwell and delays settling.
TEST(SettleGate, InvalidSampleResetsDwell)
{
    SettleGate gate(makeConfig(0.05, 1.0, 0.02, 6), 0.0, 0);
    std::uint64_t seq = 1;
    // 5 consecutive qualifying samples (0.00..0.04): count 5, span 0.04 < dwell.
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(gate.offer(qualifying(seq, 0.01 * seq)),
                  SettleOutcome::Continue);
        ++seq;
    }
    EXPECT_EQ(gate.qualifyingSamples(), 5);
    // One invalid sample resets the qualifying run.
    EXPECT_EQ(gate.offer(invalid(seq, 0.01 * seq)), SettleOutcome::Continue);
    ++seq;
    EXPECT_EQ(gate.qualifyingSamples(), 0);
    // Must re-accumulate a full dwell before settling. The reset means it cannot
    // settle before at least required_samples qualifying samples arrive again.
    bool settled = false;
    for (int i = 0; i < 12; ++i) {
        if (gate.offer(qualifying(seq, 0.01 * seq)) == SettleOutcome::Settled) {
            settled = true;
            EXPECT_GE(i + 1, 6);
            break;
        }
        ++seq;
    }
    EXPECT_TRUE(settled);
}

// 5. An inter-sample gap wider than sample_max_age resets the dwell.
TEST(SettleGate, ExcessiveGapResetsDwell)
{
    SettleGate gate(makeConfig(0.05, 2.0, 0.02, 6), 0.0, 0);
    std::uint64_t seq = 1;
    for (int i = 0; i < 5; ++i) {  // 0.00..0.04
        gate.offer(qualifying(seq, 0.01 * seq));
        ++seq;
    }
    EXPECT_EQ(gate.qualifyingSamples(), 5);
    // Gap of 0.06 s (> 0.02 s) before the next qualifying sample.
    const auto outcome = gate.offer(qualifying(seq, 0.10));
    EXPECT_EQ(outcome, SettleOutcome::Continue);
    // Reset happened, then this sample restarts the run at 1.
    EXPECT_EQ(gate.qualifyingSamples(), 1);
}

// 6. Enough elapsed time but too few samples: dwell span met, count not met.
TEST(SettleGate, EnoughTimeTooFewSamples)
{
    SettleGate gate(makeConfig(0.05, 5.0, 0.02, 20), 0.0, 0);
    // Samples spaced exactly at sample_max_age (0.02, not a gap): span reaches
    // 0.10 s (>= dwell) but only 6 samples arrive (< required 20).
    SettleOutcome outcome = SettleOutcome::Continue;
    std::uint64_t seq = 1;
    for (double t = 0.0; t <= 0.10 + 1e-9; t += 0.02) {
        outcome = gate.offer(qualifying(seq, t));
        ++seq;
    }
    EXPECT_EQ(outcome, SettleOutcome::Continue);
    EXPECT_LT(gate.qualifyingSamples(), 20);
}

// 7. Missing velocity (invalid samples) fails closed rather than settling.
TEST(SettleGate, MissingVelocityFailsClosed)
{
    SettleGate gate(makeConfig(0.05, 0.2, 0.02, 6), 0.0, 0);
    bool timed_out = false;
    for (std::uint64_t seq = 1; seq <= 40; ++seq) {
        const auto outcome = gate.offer(invalid(seq, 0.01 * seq));
        EXPECT_NE(outcome, SettleOutcome::Settled);
        if (outcome == SettleOutcome::TimedOut) {
            timed_out = true;
            break;
        }
    }
    EXPECT_TRUE(timed_out);
}

// 8. Oscillation around the goal never holds long enough: times out.
TEST(SettleGate, OscillationTimesOut)
{
    SettleGate gate(makeConfig(0.05, 0.3, 0.02, 6), 0.0, 0);
    bool timed_out = false;
    for (std::uint64_t seq = 1; seq <= 60; ++seq) {
        const SettleSample s = (seq % 2 == 0) ? qualifying(seq, 0.01 * seq)
                                              : moving(seq, 0.01 * seq);
        const auto outcome = gate.offer(s);
        EXPECT_NE(outcome, SettleOutcome::Settled);
        if (outcome == SettleOutcome::TimedOut) {
            timed_out = true;
            break;
        }
    }
    EXPECT_TRUE(timed_out);
}

// 9. A clean hold settles exactly when both dwell span and sample count are met.
TEST(SettleGate, CleanHoldSettlesAtRequiredCount)
{
    SettleGate gate(makeConfig(0.05, 1.0, 0.02, 6), 0.0, 0);
    // dt = 0.01 s: the 6th sample is at t = 0.05 (span 0.05 == dwell, count 6).
    for (std::uint64_t seq = 1; seq <= 5; ++seq) {
        EXPECT_EQ(gate.offer(qualifying(seq, 0.01 * (seq - 1))),
                  SettleOutcome::Continue);
    }
    EXPECT_EQ(gate.offer(qualifying(6, 0.05)), SettleOutcome::Settled);
    EXPECT_EQ(gate.qualifyingSamples(), 6);
}

// The no-sample path: budget expiry is reported without any offered sample.
TEST(SettleGate, HasTimedOutWhenNoSampleArrives)
{
    SettleGate gate(makeConfig(0.05, 0.5, 0.02, 6), 0.0, 0);
    EXPECT_FALSE(gate.hasTimedOut(0.4));
    EXPECT_TRUE(gate.hasTimedOut(0.5));
    EXPECT_TRUE(gate.hasTimedOut(0.7));
}

TEST(SettleGate, QualifyingSampleAtDeadlineTimesOut)
{
    SettleGate gate(makeConfig(0.049, 0.10, 0.02, 6), 0.0, 0);
    for (std::uint64_t seq = 1; seq <= 5; ++seq) {
        EXPECT_EQ(gate.offer(qualifying(seq, 0.04 + 0.01 * seq)),
                  SettleOutcome::Continue);
    }

    EXPECT_EQ(gate.offer(qualifying(6, 0.10)), SettleOutcome::TimedOut);
}

TEST(SettleGate, QualifyingSampleAfterDeadlineTimesOut)
{
    SettleGate gate(makeConfig(0.049, 0.10, 0.02, 6), 0.0, 0);
    for (std::uint64_t seq = 1; seq <= 5; ++seq) {
        EXPECT_EQ(gate.offer(qualifying(seq, 0.049 + 0.01 * seq)),
                  SettleOutcome::Continue);
    }

    EXPECT_EQ(gate.offer(qualifying(6, 0.109)), SettleOutcome::TimedOut);
}
