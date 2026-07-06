# MPC velocity-target variant comparison

Lower values are better for the metrics below. The comparison discards the startup warmup window.

## Aggregate change of 4-ref velocity variant vs 3-ref baseline

- q tracking RMSE: -0.00%
- reference acceleration RMS: -0.28%
- reference jerk RMS: -1.39%
- policy-node velocity error RMS: -6.06%

## CSV artifacts

- `analysis_artifacts/mpc_velocity_compare/metrics_by_joint.csv`
- `analysis_artifacts/mpc_velocity_compare/aggregate_metrics.csv`
- `analysis_artifacts/mpc_velocity_compare/comparison_vs_preview_mpc_3ref.csv`

## Figures

- `analysis_artifacts/mpc_velocity_compare/figures/mpc_velocity_compare_metric_ratios.svg`
- `analysis_artifacts/mpc_velocity_compare/figures/mpc_velocity_compare_smoothness.svg`
- `analysis_artifacts/mpc_velocity_compare/figures/mpc_velocity_compare_tracking_timeseries.svg`

## Aggregate table

| method | joint_id | samples | q_rmse | q_max_abs_error | dq_rmse | dq_max_abs_error | ref_dq_rms | ref_ddq_rms | ref_jerk_rms | policy_node_q_error_rms | policy_node_dq_error_rms | tau_applied_rms | tau_applied_abs_max | tau_rate_rms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| preview_mpc_3ref | 1.5 | 1400 | 0.0188231 | 0.0275765 | 0.0910163 | 0.14319 | 0.748015 | 3.86194 | 885.172 | 2.85606e-13 | 0.0948123 | 2.30881 | 3.42663 | 28.9622 |
| preview_mpc_velocity_4ref | 1.5 | 1400 | 0.0188227 | 0.0276113 | 0.091028 | 0.145211 | 0.747985 | 3.85124 | 872.861 | 2.85606e-13 | 0.089067 | 2.3088 | 3.42872 | 29.0224 |
