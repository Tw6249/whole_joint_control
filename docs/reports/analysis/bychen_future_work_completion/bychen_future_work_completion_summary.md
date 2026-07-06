# Future-work completion analysis

Scope: MuJoCo candidate simulation plus local identified-model numerical studies. These results are mechanism-screening evidence and do not replace hardware validation.

## MuJoCo O4+U1 versus O5+U1

| label | hip_rmse | knee_rmse | coord_rmse | hip_delta_tau_total_rms | knee_delta_tau_total_rms | hip_eta_u_rms | knee_eta_u_rms | combined_flags |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| O4+U1 | 0.0333992 | 0.0348685 | 0.0675625 | 0.178727 | 0.137295 | 0.00473243 | 0.0158537 | 4 |
| O5+U1 | 0.0330672 | 0.0336399 | 0.0659954 | 0.178641 | 0.13842 | 0.00469971 | 0.0156551 | 4 |

## Numerical ablation

| case | ablation_label | coord_rmse | knee_delta_u_rms | knee_eta_u_rms | coord_recovery_rmse |
| --- | --- | --- | --- | --- | --- |
| O4+U1 | PD / inverse only | 0.0755619 | 0.0250322 | 0 | 0.00201443 |
| O4+U1 | center feedback only | 0.0938873 | 0.0256082 | 0 | 0.00251959 |
| O4+U1 | input compensation only | 0.0754977 | 0.0250299 | 0.00482002 | 0.00201289 |
| O4+U1 | full EID | 0.0934962 | 0.0255955 | 0.0289689 | 0.00250814 |
| O5+U1 | PD / inverse only | 0.0755619 | 0.0250322 | 0 | 0.00201443 |
| O5+U1 | center feedback only | 0.0902214 | 0.0255307 | 0 | 0.00240073 |
| O5+U1 | input compensation only | 0.0754852 | 0.0250294 | 0.00576229 | 0.00201259 |
| O5+U1 | full EID | 0.0898342 | 0.0255176 | 0.0287576 | 0.00238992 |

## Residual lambda numerical study

| case | lambda | coord_rmse | knee_delta_u_rms | knee_eta_u_rms |
| --- | --- | --- | --- | --- |
| O4+U1 | 0 | 0.0892145 | 0.0255144 | 0.0287638 |
| O4+U1 | 0.5 | 0.0913265 | 0.0255573 | 0.0288675 |
| O4+U1 | 1 | 0.0934962 | 0.0255955 | 0.0289689 |
| O5+U1 | 0 | 0.0856704 | 0.0254209 | 0.0285439 |
| O5+U1 | 0.5 | 0.0877226 | 0.0254715 | 0.028652 |
| O5+U1 | 1 | 0.0898342 | 0.0255176 | 0.0287576 |

## Residual lambda pole/noise summary

| case | lambda | joint | max_pole_abs | noise_to_control_peak_mag |
| --- | --- | --- | --- | --- |
| O4+U1 | 0 | Knee | 0.954197 | 655.14 |
| O4+U1 | 0.5 | Knee | 0.954197 | 208.088 |
| O4+U1 | 1 | Knee | 0.954197 | 126.188 |
| O5+U1 | 0 | Knee | 0.999999 | 11782.1 |
| O5+U1 | 0.5 | Knee | 0.954197 | 287.081 |
| O5+U1 | 1 | Knee | 0.954197 | 138.543 |

Generated files:

- `analysis_artifacts/bychen_future_work_completion/mujoco_o4u1_o5u1_metrics.csv`
- `analysis_artifacts/bychen_future_work_completion/mujoco_o4u1_o5u1_frequency_summary.csv`
- `analysis_artifacts/bychen_future_work_completion/numerical_ablation_metrics.csv`
- `analysis_artifacts/bychen_future_work_completion/numerical_residual_lambda_metrics.csv`
- `analysis_artifacts/bychen_future_work_completion/numerical_residual_lambda_poles_frequency.csv`
- `analysis_artifacts/bychen_future_work_completion/figures/mujoco_o4u1_o5u1_pareto.png`
- `analysis_artifacts/bychen_future_work_completion/figures/mujoco_o4u1_o5u1_spectrum.png`
- `analysis_artifacts/bychen_future_work_completion/figures/numerical_ablation_summary.png`
- `analysis_artifacts/bychen_future_work_completion/figures/numerical_residual_lambda_tradeoff.png`
