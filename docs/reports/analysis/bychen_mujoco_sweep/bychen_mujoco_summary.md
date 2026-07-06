# Bychen MuJoCo 2x2 sweep

Scope: Unitree H1 MuJoCo simulation with the existing C++ EID stepper. These numbers are simulation evidence for mechanism screening, not hardware validation.

Disturbance: joints `1,2`, torques `6,-4` N m, half-cosine window 4/4.2/5.2/5.4 s.

| case | hip_so | knee_so | hip_rmse | knee_rmse | coord_rmse | hip_delta_tau_total_rms | knee_delta_tau_total_rms | combined_delta_tau_total_rms | hip_eta_u_rms | knee_eta_u_rms | combined_flags |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | 1.000000 | 1.000000 | 0.033399 | 0.034868 | 0.067562 | 0.178727 | 0.137295 | 0.225374 | 0.004732 | 0.015854 | 4 |
| B | 1.250000 | 1.000000 | 0.033040 | 0.034874 | 0.067200 | 0.178686 | 0.137283 | 0.225334 | 0.004697 | 0.015856 | 4 |
| C | 1.000000 | 0.750000 | 0.033345 | 0.036955 | 0.069595 | 0.178817 | 0.135396 | 0.224294 | 0.004727 | 0.016193 | 4 |
| D | 1.250000 | 0.750000 | 0.032986 | 0.036962 | 0.069233 | 0.178770 | 0.135377 | 0.224244 | 0.004691 | 0.016195 | 4 |

Relative changes are computed against case A. Negative values are lower than A.

| case | hip_rmse_vs_A_pct | knee_rmse_vs_A_pct | coord_rmse_vs_A_pct | hip_delta_tau_total_rms_vs_A_pct | knee_delta_tau_total_rms_vs_A_pct | combined_delta_tau_total_rms_vs_A_pct | hip_tau_total_rms_vs_A_pct | knee_tau_total_rms_vs_A_pct |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| B | -1.07 | 0.02 | -0.54 | -0.02 | -0.01 | -0.02 | -0.02 | -0.03 |
| C | -0.16 | 5.99 | 3.01 | 0.05 | -1.38 | -0.48 | -0.40 | -0.35 |
| D | -1.24 | 6.00 | 2.47 | 0.02 | -1.40 | -0.50 | -0.41 | -0.38 |

Generated artifacts:

- `analysis_artifacts/bychen_mujoco_sweep/bychen_mujoco_metrics.csv`
- `analysis_artifacts/bychen_mujoco_sweep/bychen_mujoco_metrics_with_relative.csv`
- `analysis_artifacts/bychen_mujoco_sweep/figures/bychen_mujoco_pareto.png` and `analysis_artifacts/bychen_mujoco_sweep/figures/bychen_mujoco_pareto.pdf`
- `analysis_artifacts/bychen_mujoco_sweep/figures/bychen_mujoco_2x2_heatmap.png` and `analysis_artifacts/bychen_mujoco_sweep/figures/bychen_mujoco_2x2_heatmap.pdf`
