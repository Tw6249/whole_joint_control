# Hip-Knee Mutual Coupling Analysis

This report excludes externally injected torques.
Coupling terms are computed from MuJoCo inverse dynamics along measured right hip pitch and right knee trajectories.

## Steady Window RMS

| method | n_runs | d_k_from_hip_acc_rms | d_k_from_hip_vel_rms | d_k_from_hip_dynamic_rms | d_k_total_rms | d_h_from_knee_acc_rms | d_h_from_knee_vel_rms | d_h_from_knee_dynamic_rms | d_h_total_rms | d_k_recon_err_rms | d_h_recon_err_rms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| eid | 5 | 2.06 | 0.07001 | 2.066 | 1.752 | 3.72 | 0.06384 | 3.716 | 3.33 | 3.933e-16 | 8.095e-16 |
| pd | 5 | 1.39 | 0.06419 | 1.4 | 0.9158 | 1.791 | 0.06119 | 1.786 | 1.616 | 2.131e-16 | 5.287e-16 |

## Figures

- `analysis_artifacts/hip_knee_mutual_coupling/figures/hip_knee_non_inertial_mechanism.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/hip_knee_non_inertial_mechanism.pdf`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/mutual_dynamic_coupling_timeseries.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/mutual_dynamic_coupling_timeseries.pdf`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/local_model_residual_timeseries.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/local_model_residual_timeseries.pdf`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/mutual_coupling_rms.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/mutual_coupling_rms.pdf`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/coupling_component_share.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/coupling_component_share.pdf`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/hip_acc_to_knee_torque_link.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/hip_acc_to_knee_torque_link.pdf`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/knee_acc_to_hip_torque_link.png`
- `analysis_artifacts/hip_knee_mutual_coupling/figures/knee_acc_to_hip_torque_link.pdf`
