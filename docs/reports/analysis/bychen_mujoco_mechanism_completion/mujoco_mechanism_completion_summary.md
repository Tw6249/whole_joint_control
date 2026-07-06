# MuJoCo mechanism completion

Scope: all rows in this folder were generated with `scripts/run_mujoco.py` and the C++ `h1_controller_stepper` executable.  The results are simulation-based mechanism screening evidence, not hardware validation.

Protocol: duration `8` s, dt `0.002` s, disturbance joints `1,2`, disturbance torques `6,-4` N m, half-cosine timing `4/4.2/5.2/5.4` s.

## Candidate layer

| case | hip_rmse | knee_rmse | coord_rmse | knee_delta_tau_total_rms | knee_eta_u_rms | combined_flags | saturation_joint_rows |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A_O4U1 | 0.033399 | 0.034868 | 0.067562 | 0.137295 | 0.015854 | 4 | 8000 |
| B_HipUp | 0.033040 | 0.034874 | 0.067200 | 0.137283 | 0.015856 | 4 | 8000 |
| D_HipUpKneeDown | 0.032986 | 0.036962 | 0.069233 | 0.135377 | 0.016195 | 4 | 8000 |
| O5U1 | 0.033067 | 0.033640 | 0.065995 | 0.138420 | 0.015655 | 4 | 8000 |
| KneeVelDown | 0.033350 | 0.036784 | 0.069429 | 0.135514 | 0.016194 | 4 | 8000 |

Relative changes against `A_O4U1` are available in `analysis_artifacts/bychen_mujoco_mechanism_completion/mujoco_mechanism_metrics_with_relative.csv`.

## Structural ablation layer

| case | eid_mode | coord_rmse | knee_delta_tau_total_rms | knee_eta_u_rms | coord_recovery_rmse | combined_flags |
| --- | --- | --- | --- | --- | --- | --- |
| O4U1_pd_inverse_only | pd_inverse_only | 0.059987 | 0.132226 | 0.000000 | 0.017111 | 4 |
| O4U1_center_feedback_only | center_feedback_only | 0.067726 | 0.137277 | 0.000000 | 0.018157 | 4 |
| O4U1_input_compensation_only | input_compensation_only | 0.059821 | 0.132180 | 0.015753 | 0.017092 | 4 |
| O4U1_full_eid | full_eid | 0.067562 | 0.137295 | 0.015854 | 0.018133 | 4 |
| O5U1_pd_inverse_only | pd_inverse_only | 0.059987 | 0.132226 | 0.000000 | 0.017111 | 4 |
| O5U1_center_feedback_only | center_feedback_only | 0.066155 | 0.138418 | 0.000000 | 0.017941 | 4 |
| O5U1_input_compensation_only | input_compensation_only | 0.059822 | 0.132128 | 0.015572 | 0.017092 | 4 |
| O5U1_full_eid | full_eid | 0.065995 | 0.138420 | 0.015655 | 0.017918 | 4 |

## Residual lambda layer

| case | residual_eta_lambda | coord_rmse | knee_delta_tau_total_rms | knee_eta_u_rms | combined_flags |
| --- | --- | --- | --- | --- | --- |
| O4U1_lambda_0 | 0.000000 | 0.065733 | 0.138952 | 0.015668 | 4 |
| O4U1_lambda_0p5 | 0.500000 | 0.066631 | 0.138136 | 0.015760 | 4 |
| O4U1_lambda_1 | 1.000000 | 0.067562 | 0.137295 | 0.015854 | 4 |
| O5U1_lambda_0 | 0.000000 | 0.064238 | 0.139901 | 0.015483 | 4 |
| O5U1_lambda_0p5 | 0.500000 | 0.065104 | 0.139192 | 0.015565 | 4 |
| O5U1_lambda_1 | 1.000000 | 0.065995 | 0.138420 | 0.015655 | 4 |

## Frequency summary

| case | signal | peak_freq_hz_0p5_100 | bandpower_0p5_3hz | bandpower_3_15hz | bandpower_15_80hz |
| --- | --- | --- | --- | --- | --- |
| A_O4U1 | coord_error | 0.976563 | 0.000111334 | 8.42937e-07 | 6.76769e-08 |
| A_O4U1 | knee_delta_tau_total | 20.5078 | 0.000159025 | 2.32298e-05 | 0.0229257 |
| A_O4U1 | knee_eta_u | 0.976563 | 1.41757e-06 | 5.99228e-08 | 4.60755e-06 |
| B_HipUp | coord_error | 0.976563 | 0.000111593 | 8.11369e-07 | 6.75087e-08 |
| B_HipUp | knee_delta_tau_total | 20.5078 | 0.000159836 | 2.18171e-05 | 0.0229225 |
| B_HipUp | knee_eta_u | 0.976563 | 1.42287e-06 | 5.87901e-08 | 4.60793e-06 |
| D_HipUpKneeDown | coord_error | 0.976563 | 0.000112732 | 8.97998e-07 | 6.56875e-08 |
| D_HipUpKneeDown | knee_delta_tau_total | 20.5078 | 0.000155285 | 2.3648e-05 | 0.0222454 |
| D_HipUpKneeDown | knee_eta_u | 0.976563 | 1.49097e-06 | 6.18593e-08 | 3.21556e-06 |
| O5U1 | coord_error | 0.976563 | 0.000111217 | 7.73551e-07 | 6.82377e-08 |
| O5U1 | knee_delta_tau_total | 20.5078 | 0.000162532 | 2.46052e-05 | 0.023359 |
| O5U1 | knee_eta_u | 20.5078 | 1.3843e-06 | 6.06421e-08 | 5.74543e-06 |
| KneeVelDown | coord_error | 0.976563 | 0.000112503 | 9.27813e-07 | 6.59147e-08 |
| KneeVelDown | knee_delta_tau_total | 20.5078 | 0.000154717 | 2.58146e-05 | 0.0222964 |
| KneeVelDown | knee_eta_u | 0.976563 | 1.48635e-06 | 6.424e-08 | 3.24717e-06 |

Generated artifacts:

- `analysis_artifacts/bychen_mujoco_mechanism_completion/mujoco_mechanism_metrics.csv`
- `analysis_artifacts/bychen_mujoco_mechanism_completion/mujoco_mechanism_metrics_with_relative.csv`
- `analysis_artifacts/bychen_mujoco_mechanism_completion/mujoco_mechanism_frequency_summary.csv`
- `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_candidate_pareto.png` and `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_candidate_pareto.pdf`
- `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_structure_ablation.png` and `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_structure_ablation.pdf`
- `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_residual_lambda.png` and `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_residual_lambda.pdf`
- `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_spectrum_summary.png` and `analysis_artifacts/bychen_mujoco_mechanism_completion/figures/mujoco_mechanism_spectrum_summary.pdf`
