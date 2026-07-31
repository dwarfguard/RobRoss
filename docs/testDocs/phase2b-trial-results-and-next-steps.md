# Phase 2B Trial Results (2026-07-24) and Next Steps

**Status:** Analysis of recorded evidence + work order
**Created:** 2026-07-24
**Evidence:** `robross_phase2_20260724_144321/` (committed on `sai`)
**Related:** `docs/aubo-painting-tracking-remediation-plan.md` (§7),
`docs/testDocs/code-review-phase-alignment.md`,
`robross_phase2_20260724_144321/real_hardware_servoj_warning_analysis.md`

## 1. What was run

Two matched update-rate / ServoJ-period trials, above the paper (20 mm-above
canvas pose), per the Phase 2B trial table (§7):

| Trial | Controller rate | ServoJ `t` | Bag |
| --- | --- | --- | --- |
| A | 125 Hz | 0.008 s | `trial_a_125hz_008s/` |
| B | 200 Hz | 0.005 s | `trial_b_200hz_005s/` |

The analyzer (`analyze_tracking_bag.py`, Phase 0 / Slice 1) was re-run
independently against both raw `.db3` bags with the trial's own canvas pose
(`canvas_calibration_20mm_above.yaml`) and hardware calibration
(`contact_hardware_source.yaml`). The re-run reproduced the committed
`trial_a_summary.md` / `trial_b_summary.md` byte-for-byte — a clean
reproducibility check on the analyzer.

## 2. Gate results — both trials FAIL

Phase 2B acceptance gate (remediation plan §7):

| §7 gate criterion | Trial A (125/0.008) | Trial B (200/0.005) |
| --- | --- | --- |
| Update rate ≥ 95% of configured | 100% ✓ | **79%** ✗ |
| No queue-full in fixtures | **91 events** ✗ | **7457 events** ✗ |
| No unexplained ServoJ error code | ok ✓ | ok ✓ |
| Joint delay median < 30 / p95 < 50 ms | **96 / 128 ms** ✗ | n/a (INCOMPLETE axis) |
| Canvas-normal ≤ ±0.25 mm | **3.65 mm** ✗ (travel-dominated) | **2.11 mm** ✗ |

Honest-gate verdicts (Slice 1 behavior confirmed on live data):

- **Trial A — tracking gate FAIL, timing gate FAIL.** A held real-time rate
  (100% of 125 Hz), latched no timing fault, and returned all-OK ServoJ codes.
  It fails the timing gate *only* on queue-full (91 events / 1041 ms blocked) and
  the tracking gate on the ~96 ms command-to-feedback delay (3× over the 30 ms
  budget). The worst |normal| / per-cycle-pp figures are dominated by the
  20 mm-off-plane travel (`move_to`) segments; the actual `paint_path` strokes
  track tight (~±0.2–0.5 mm normal).
- **Trial B — not a valid tracking trial; driver-saturation failure.** The
  200 Hz / 5 ms config overwhelmed the ServoJ queue: rate collapsed to 79%
  (~158 Hz), queue-full fired 7457 times (85.9 s cumulative blocked), and the
  timing fault **latched**. The run degraded before drawing anything — the
  tracking table contains only `move_to` segments (no `paint_path`), with
  tangential errors of 60–151 mm and forearm joint error up to 27°. The tracking
  gate reports delay **MISSING** (INCOMPLETE on that axis) and FAILs on normal
  error.

## 3. The evidence confirms the open code-review findings

`real_hardware_servoj_warning_analysis.md` pins the dominant disturbance, and it
matches the still-open Phase 2A findings:

- **Finding 2.6 (confirmed on hardware).** The driver handles SDK queue-full
  (rc=2) with an **unbounded 5 ms blocking retry on the ros2_control write
  thread**, which manufactures the late cycles (~12 ms) and, via absolute-deadline
  catch-up, feeds back into more saturation. The `rc=ok` field also masks
  transient queue-full responses (the `qf_*` fields preserve them) — exactly the
  "failed/queue-full writes reported OK" finding.
- **Finding 2.1.** The `aubo_servoj_diag` output is windowed aggregates, not
  full-rate per-ServoJ-call command/timestamp telemetry; per-cycle phase delay
  cannot be computed from it.
- **Finding 2.9.** Diagnostics formatting runs on the RT control thread.

## 4. New blocker not yet on the roadmap

Warning-doc §1: the **installed** description Xacro dropped `servoj_time` and
silently fell back to 0.005 s (`aubo_description/CMakeLists.txt` runs `xacro.sh`
only at configure time, so pulls / incremental builds leave the installed copy
stale). This is a live recurrence of the known "colcon install dir not
refreshed" gotcha, and it is the exact class of silent config mismatch that
**finding 2.8** (config authority) exists to reject.

Trial A appears to have **survived** it — it reports `t=0.008` with a matched
8 ms period and essentially zero mismatch warnings (1 across 297 windows), which
would be impossible under the 0.005 fallback — so Trial A is read as a valid
matched trial. But until this is fixed, any *re-run* 125 Hz trial risks silently
reverting to 0.005, and a mismatched pair must be rejected loudly rather than
run.

## 5. Decision per the plan

Plan §7 (line ~416) is operative: *"If neither timing pair meets the gate, stop
contact testing and investigate the RPC streaming architecture before tuning
lookahead or gain."* Neither pair passed, so contact testing (Stage 3) stays
halted and the next move is the streaming architecture — specifically the
queue-full blocking retry (2.6), not gain/lookahead tuning (§4 also forbids the
latter before `t` is matched and measured; it now is).

## 6. Next steps (in order)

1. **Slice 2a — bounded queue-full policy (2.6) + off-RT diagnostics (2.9)
   [DONE, uncommitted].** The unbounded blocking queue-full retry is replaced by
   a bounded, newest-command-preserving policy (default 0 in-cycle retries: on
   `AUBO_QUEUE_FULL` the cycle is dropped and the next cycle resends a fresher
   command, so the RT write thread never sleeps on back-pressure) that latches a
   control fault when drops persist for a full report window — the same
   transient-tolerant streak pattern as the timing-mismatch latch. The periodic
   timing report's string formatting + `/rosout` publish move to an off-RT worker
   thread. Driver-only (`aubo_ros2_driver`); `aubo_ros2_driver` colcon test
   15 passed. **Slice 2b — full-rate per-call telemetry (2.1) [DONE,
   uncommitted]:** opt-in CSV (`AUBO_SERVOJ_TELEMETRY_CSV` env var, off by
   default) written by the Slice 2a off-RT worker — ROS/wall time, 6 commanded
   joint positions, `t`, RPC duration, return code, retries, dropped flag per
   cycle. This is the stream Slice 3 (2.3) consumes for per-cycle delay.
   `aubo_ros2_driver` colcon test 16 passed.
2. **Config authority + install staleness (2.8).** Reject a mismatched
   rate/`servoj_time` pair; ensure the installed Xacro forwards `servoj_time`.
3. **Slice 3 — per-cycle delay (2.3),** consuming Slice 2's full-rate stream, so
   the delay gate is trustworthy at full rate rather than at the coarse ~62 Hz
   `controller_state` sampling.
4. **Re-run Phase 2B** above paper (A definitely; B only if the architecture fix
   makes 200 Hz viable) and re-judge the gate. Stage 3 paper contact only on a
   real PASS.
5. **Environment fixes** from the warning doc that help the rate/jitter gate:
   enable FIFO RT scheduling (§6); drop the RTDE state request from 500 Hz to the
   ~200 Hz actually delivered (§5).

## 7. Escalation policy for Slice 2 (2.6)

Per §7 "Queue-full Policy" and §4, the replacement retry policy must: avoid
blocking the loop for an unbounded duration; preserve the newest safe position
command; surface repeated queue saturation as a hardware/control error; and not
silently continue on stale setpoints. The escalation latch mirrors the existing
ServoJ timing-fault latch (same streak-tolerant pattern), so a transient
queue-full burst does not trip a fault but sustained saturation does.
