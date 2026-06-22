# EID Tracking Diagnostics Data Manifest

This directory contains the experiment artifacts referenced by
`docs/EID跟踪问题诊断报告.md`.

## Included datasets

| Path | Purpose |
|---|---|
| `analysis_artifacts/eid_tracking_diagnostics/` | Original EID diagnostic sweep, including low/high alpha comparisons. |
| `analysis_artifacts/eid_tracking_diagnostics_alpha090/` | Full diagnostic sweep with enabled hip/knee `filter_alpha` forced to `0.90`. |
| `analysis_artifacts/eid_tracking_counterfactual/` | Counterfactual runs with inverse feedforward effectively disabled (`u_star≈0`). |

## Key files

| File | Description |
|---|---|
| `*/eid_tracking_diagnostics_metrics.csv` | Aggregated per-scenario metrics used by the report tables. |
| `*/figures/*.png` | Diagnostic figures for no-disturbance, disturbance-window, heatmap, and spectrum analysis. |
| `*/runs/*/mujoco_closed_loop_log.csv` | Per-run time-series logs with references, measured states, torque commands, EID states, and disturbance telemetry. |
| `*/runs/*/summary.csv` | Per-run summary metrics exported by `scripts/run_mujoco.py`. |
| `*/configs/*.yaml` | Generated input configs for each scenario/method pair. |

## Notes

- The data is intentionally scoped to the EID tracking diagnosis and does not include unrelated MuJoCo smoke-test artifacts.
- No individual file in these datasets exceeds GitHub's 100 MB file limit.
