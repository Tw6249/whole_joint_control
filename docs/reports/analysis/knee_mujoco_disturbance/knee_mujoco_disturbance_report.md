# MuJoCo Knee Disturbance Decomposition

This report decomposes the right-knee input residual against the controller's local single-joint plant.

## Disturbance Window RMS

| method | n_runs | tau_sw_k_rms | d_g_rms | d_hacc_rms | d_kacc_rms | d_vel_rms | d_model_rms | d_in_rms | recon_err_over_model_rms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| eid | 5 | 3.626 | 0.9519 | 2.155 | 3.378e-09 | 0.0634 | 1.929 | 3.57 | 1.874e-16 |
| pd | 5 | 3.625 | 1.006 | 1.417 | 1.796e-09 | 0.06825 | 1.053 | 3.117 | 2.028e-16 |

## EID Compensation Alignment

| target | corr_zero_lag | best_corr_abs_lag | best_lag_ms |
| --- | --- | --- | --- |
| d_g | -0.1501 | -0.3449 | 79.28 |
| d_hacc | 0.2336 | 0.5011 | -33.74 |
| d_in | -0.1491 | -0.511 | 48.38 |
| d_kacc | 0.3343 | 0.6639 | -34.56 |
| d_model | 0.2204 | 0.5532 | -34.56 |
| d_vel | 0.1009 | -0.2334 | 79.28 |
| tau_sw_k | -0.6101 | -0.6232 | -14.23 |

## Figures

- `analysis_artifacts/knee_mujoco_disturbance/figures/knee_mujoco_decomposition_timeseries_eid_r01.png`
- `analysis_artifacts/knee_mujoco_disturbance/figures/knee_mujoco_component_rms.png`
- `analysis_artifacts/knee_mujoco_disturbance/figures/knee_mujoco_eid_alignment.png`
