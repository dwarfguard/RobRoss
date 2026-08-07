#pragma once

#include <cstdint>
#include <limits>

namespace robross_painter
{

// Configuration for one endpoint-settling attempt. All times are seconds on a
// monotonic clock (the caller supplies the clock readings, so this stays pure).
struct SettleConfig
{
    double dwell = 0.15;           // continuous in-tolerance hold required (s)
    double timeout = 1.5;          // total budget from trajectory completion (s)
    double sample_max_age = 0.05;  // max gap between consecutive samples (s)
    int required_samples = 20;     // min qualifying samples spanning the dwell
};

// One already-evaluated feedback sample handed to the gate. The caller does all
// ROS/MoveIt work (freshness, finite check, endpoint geometry, joint velocity)
// and passes only the resulting facts plus timing, so the gate can be tested
// without hardware, ROS executors, or wall-clock sleeps.
struct SettleSample
{
    std::uint64_t sequence = 0;  // monotonic joint-state counter
    double receipt_time = 0.0;   // when this sample was received (monotonic s)
    bool valid = false;          // finite position AND velocity for every joint
    bool endpoint_ok = false;    // measured endpoint within tolerance
    bool stationary = false;     // max joint speed <= velocity tolerance
    // Max normalized endpoint error (error / limit), for diagnostics / best-sample
    // tracking. Infinity when the sample is invalid or out of tolerance.
    double error_ratio = std::numeric_limits<double>::infinity();
};

enum class SettleOutcome
{
    Continue,  // keep waiting for more samples
    Settled,   // endpoint reached and held; accept this sample
    TimedOut   // budget exhausted before settling; fail closed
};

// Time/sample bookkeeping for endpoint settling. Holds no ROS state; every
// decision is a pure function of the samples offered plus the times they carry.
// A sample "qualifies" only when endpoint tolerance, low velocity, and validity
// hold *together*; the gate reports Settled only once qualifying samples hold
// continuously for `dwell` AND number at least `required_samples`. Any
// non-qualifying sample, or an inter-sample gap wider than `sample_max_age`,
// resets the dwell — so a stop at the wrong location (stationary but out of
// tolerance) and a right-location fly-through (in tolerance but moving) both
// fail to settle.
class SettleGate
{
public:
    SettleGate(const SettleConfig &config, double completion_time,
               std::uint64_t completion_sequence)
        : config_(config),
          completion_time_(completion_time),
          last_sequence_(completion_sequence)
    {
    }

    // Feed one sample that arrived (its sequence must exceed lastSequence()).
    SettleOutcome offer(const SettleSample &sample)
    {
        ++samples_seen_;
        if (sample.error_ratio < best_error_ratio_) {
            best_error_ratio_ = sample.error_ratio;
        }

        // The timeout is a hard budget. A sample received at or after the
        // deadline cannot complete the gate, even if it would otherwise finish
        // the dwell and sample-count requirements.
        if (sample.receipt_time - completion_time_ >= config_.timeout) {
            return SettleOutcome::TimedOut;
        }

        if (have_last_receipt_ &&
            sample.receipt_time - last_receipt_ > config_.sample_max_age) {
            resetQualifying();
        }
        last_receipt_ = sample.receipt_time;
        have_last_receipt_ = true;
        last_sequence_ = sample.sequence;

        const bool qualifies =
            sample.valid && sample.endpoint_ok && sample.stationary;
        if (qualifies) {
            if (!qualifying_) {
                qualifying_since_ = sample.receipt_time;
                qualifying_ = true;
            }
            ++qualifying_samples_;
            if (sample.receipt_time - qualifying_since_ >= config_.dwell &&
                qualifying_samples_ >= config_.required_samples) {
                return SettleOutcome::Settled;
            }
        } else {
            resetQualifying();
        }

        return SettleOutcome::Continue;
    }

    // No new sample arrived; report whether the overall budget is spent.
    bool hasTimedOut(double now) const
    {
        return now - completion_time_ >= config_.timeout;
    }

    std::uint64_t lastSequence() const { return last_sequence_; }
    int qualifyingSamples() const { return qualifying_samples_; }
    int samplesSeen() const { return samples_seen_; }
    double bestErrorRatio() const { return best_error_ratio_; }

private:
    void resetQualifying()
    {
        qualifying_ = false;
        qualifying_since_ = 0.0;
        qualifying_samples_ = 0;
    }

    SettleConfig config_;
    double completion_time_;
    std::uint64_t last_sequence_;
    bool have_last_receipt_ = false;
    double last_receipt_ = 0.0;
    bool qualifying_ = false;
    double qualifying_since_ = 0.0;
    int qualifying_samples_ = 0;
    int samples_seen_ = 0;
    double best_error_ratio_ = std::numeric_limits<double>::infinity();
};

}  // namespace robross_painter
