# Code-Review Findings vs. Remediation-Plan Phases

**Status:** Analysis / roadmap (no code changes made)
**Created:** 2026-07-23
**Related:** `docs/aubo-painting-tracking-code-review.md`,
`docs/aubo-painting-tracking-remediation-plan.md`

## 1. Purpose

The code review raised 11 findings against the unpushed Phase 2 commits. Those
findings were first grouped into batches A/B/C/D by **risk and effort**, so the
batches deliberately cut *across* the remediation plan's phases. This document
re-projects every finding onto the plan's phase structure and records the
plan-imposed sequence for the remaining work, so the review can be worked off
phase-by-phase (one reversible slice per commit, per plan §14).

Findings already closed:

- **2.5** — `servoj_time` parse hardening (driver commit `9bba97a`, pushed).
- **2.10** — description submodule pushed before the parent driver commit.
- **Slice 1 (2.2, 2.4, 2.7, 2.11)** — Phase 0 analyzer honesty + geometry, in
  `analyze_tracking_bag.py` (+ tests, README). Tracking and timing gates now
  report PASS / FAIL / **INCOMPLETE** instead of silently passing without delay
  evidence or without a known configured rate; `normal_pp_per_cycle` selects the
  oscillating tangential axis (real sine fixture); diagonal/`mixed` samples are
  retained. Verified: `robross_painter` colcon test 82 passed. Not committed.

The remaining open findings are mapped below.

## 2. Finding → phase map

| Finding | Remediation phase | Plan clause it satisfies | Status |
| --- | --- | --- | --- |
| 2.2 tracking gate passes without a delay estimate | **Phase 0** (§5 analyzer) | "report … command-to-feedback phase delay"; honest Phase 2B gate | **done** Slice 1 |
| 2.4 timing gate passes with missing `servoj_config` | **Phase 0** (§5) | rate/jitter reporting feeding the Phase 2B gate | **done** Slice 1 |
| 2.7 sine geometry reports zero cycles | **Phase 0** (§5) | "per-cycle canvas-normal peak-to-peak … such as the sine path" | **done** Slice 1 |
| 2.11 diagonal/curve samples discarded | **Phase 0** (§5) | "split reversal and curved commands by instantaneous canvas direction" | **done** Slice 1 |
| 2.3 delay median/p95 semantics not per-cycle | **Phase 0** (§5) → Phase 2B gate (§7, <30/50 ms) | open |
| 2.5 `servoj_time` accepts invalid input | **Phase 2A/2B** (§7) | "reject an obvious configuration mismatch" | **done** `9bba97a` |
| 2.6 failed ServoJ writes reported OK | **Phase 2A** (§7) | "unexpected return codes must no longer be silently discarded" + Queue-full Policy | open |
| 2.9 diagnostics run on the RT control thread | **Phase 2A** (§7) | "logging … throttled so diagnostics do not create additional timing load" | open |
| 2.1 no full-rate per-call command telemetry | **Phase 2A** (§7) | "timestamped commanded joint positions for every ServoJ call … phase delay through each sine cycle" | open |
| 2.8 launch does not pair rate with `servoj_time` | **Phase 2B** (§7) | "derive it from one authoritative controller period" | open |
| 2.10 submodule commit not on remote | **§13/§14** cross-repo versioning | "changes spanning the two repositories … versioned together" | **done** |

### How the risk/effort batches split across phases

- **Batch A** (analyzer + parse hardening) splits: 2.5 was Phase 2A (done); the
  rest — 2.2, 2.4, 2.7, 2.11 — are **Phase 0**.
- **Batch B** (driver + launch) splits: 2.6 is **Phase 2A**, 2.8 is **Phase 2B**.
- **Batch C** (2.3) is **Phase 0**.
- **Batch D** (2.1, 2.9) is **Phase 2A**.
- Nothing in the review touches Phase 1, 3, 4, or 5.

## 3. Plan-imposed sequencing

Per §7, before the Phase 2B hardware trials can yield a trustworthy `PASS`:
(a) Phase 2A instrumentation must be complete and real-time-safe, and (b) the
Phase 0 analyzer must honestly enforce the gate. The dependencies are:

- **2.1 + 2.9 are coupled.** The new full-rate telemetry must itself be
  real-time-safe, so 2.9's off-RT-thread worker is the vehicle for 2.1. Do them
  together.
- **2.3 follows 2.1.** Per-cycle/temporal delay is best computed from the
  full-rate command stream; controller_state at ~56–60 Hz is too coarse (the
  review's own 2.1 rationale).
- **2.2, 2.4, 2.7, 2.11 are independent, offline, low-risk.** They immediately
  stop the analyzer from printing a false `PASS` and run on any existing bag.
- **2.8 must land before the trials** so a run cannot be mis-configured.

## 4. Recommended work order

### Slice 1 — Phase 0 analyzer honesty + geometry (2.2, 2.4, 2.7, 2.11)

Low-risk, offline, no hardware, no motion-path change; makes the very next
dry-run bag analyzable without lying. Files:
`ros2/robross_painter/scripts/analyze_tracking_bag.py`,
`ros2/robross_painter/test/test_analyze_tracking_bag.py`, short README note.

- **2.2** — when there is no delay estimate at all (all references monotonic →
  `joint_delay_ms` empty), the tracking gate must report **INCOMPLETE**, not
  `PASS`. Delay is a mandatory Phase 2B criterion; absence of evidence ≠ pass.
- **2.4** — when `servoj_config` is absent (configured rate unknown), the timing
  gate must be **INCOMPLETE** rather than silently dropping the rate check; also
  fold any return-code/queue-full events from the trailing unreported window
  into the gate.
- **2.7** — in `normal_pp_per_cycle`, select the **oscillating** tangential axis
  (most reversals via `_reversal_indices`) instead of `argmax(range)`, so the
  real sine fixture (~90 mm monotonic X, ~48 mm oscillating Y) reports cycles
  instead of zero. Add a real-geometry test.
- **2.11** — in `direction_resolved_normal_err` / `_instantaneous_direction`,
  retain diagonal/`mixed` samples as their own reported bucket instead of
  discarding, so the curved portion carrying the largest canvas-normal error is
  not dropped.

Reuse existing helpers: `_tracking_gate`, `_render_tracking`, `_render_servoj`,
`normal_pp_per_cycle`, `_reversal_indices`, `direction_resolved_normal_err`.

### Slice 2 — Phase 2A driver telemetry (2.1 + 2.9 + 2.6)

RT-safe per-ServoJ-call command/timestamp capture handed to a non-RT diagnostics
worker; escalate persistent non-OK return codes / exceptions to a control error
with a transient-tolerant streak policy (mirroring the existing timing latch);
bounded queue-full handling that preserves the newest command. **Behavior-
changing and touches §4 safety constraints — confirm the escalation policy
before editing.** Files: `aubo_hardware_interface.{h,cpp}`, `servo_timing_stats.h`,
driver `test/`. Description + driver changes versioned and pushed together
(submodule first — the 2.10 lesson).

### Slice 3 — Phase 0 per-cycle delay (2.3)

Replace whole-segment joint-lag percentiles with per-cycle/temporal delay
distributions consuming Slice 2's full-rate command stream. Analyzer + tests.

### Slice 4 — Phase 2B config authority (2.8)

Pair update-rate with `servoj_time` from one authoritative selection (or reject
the pair before activating hardware). Launch file + description xacro.

### Then — Phase 2B trials

Run the matched-timing trials above paper (A: 125 Hz / t=0.008, B: 200 Hz /
t=0.005) with the now-trustworthy analyzer and gate.

## 5. Verification (when slices are executed)

- Analyzer slices:
  `python3 -m pytest ros2/robross_painter/test/test_analyze_tracking_bag.py -q`
  — extend with real-sine-geometry, missing-config-INCOMPLETE, and
  delay-absent-INCOMPLETE cases; confirm the synthetic-ramp bag reports the
  tracking gate as INCOMPLETE (delay n/a) rather than PASS.
- Package gate (plan §12):
  `colcon build --packages-select robross_painter && colcon test --packages-select robross_painter && colcon test-result --verbose`
  (env: `export PATH="/usr/bin:$PATH"; source /opt/ros/humble/setup.bash; source install/setup.bash`).
- Driver slices: `colcon test --packages-select aubo_ros2_driver`.

## 6. Discipline

One phase-slice per commit (§14 rollback rule). Cross-repo (driver + description)
changes versioned and pushed together, submodule first. Commit/push only when
asked.
