# Bychen MuJoCo deep analysis

Scope: post-processing of the already generated A-D MuJoCo logs. The frequency layer reports Welch PSD/CSD/coherence and a closed-loop empirical H1 FRF estimate. The augmented layer identifies a local discrete plant from MuJoCo logs and then constructs the EID augmented matrix with the controller's prior/update timing. The eigenvalue table contains 320 poles across cases, joints, and delay settings.

## Frequency layer: selected disturbance-window PSD peaks

| case | signal | peak_freq_hz_0p5_100 | peak_psd | bandpower_0p5_3hz | bandpower_15_80hz |
| --- | --- | --- | --- | --- | --- |
| A | e_coord | 0.976563 | 0.000167919 | 0.000111335 | 6.76766e-08 |
| A | delta_tau_total_knee | 20.5078 | 0.00784346 | 0.000159313 | 0.0229257 |
| A | eta_u_knee | 0.976563 | 2.02461e-06 | 1.41758e-06 | 4.60755e-06 |
| B | e_coord | 0.976563 | 0.000166941 | 0.000111594 | 6.75084e-08 |
| B | delta_tau_total_knee | 20.5078 | 0.0078382 | 0.000160123 | 0.0229225 |
| B | eta_u_knee | 0.976563 | 2.02484e-06 | 1.42288e-06 | 4.60793e-06 |
| C | e_coord | 0.976563 | 0.000172743 | 0.000112521 | 6.58406e-08 |
| C | delta_tau_total_knee | 20.5078 | 0.00760335 | 0.000154519 | 0.0222467 |
| C | eta_u_knee | 0.976563 | 2.21017e-06 | 1.48477e-06 | 3.2146e-06 |
| D | e_coord | 0.976563 | 0.000171665 | 0.000112733 | 6.56873e-08 |
| D | delta_tau_total_knee | 20.5078 | 0.00759887 | 0.000155523 | 0.0222454 |
| D | eta_u_knee | 0.976563 | 2.2108e-06 | 1.49098e-06 | 3.21556e-06 |

## Frequency layer: disturbance-to-coordination coherence

| case | peak_coherence_freq_hz_0p5_80 | peak_coherence | cross_spectrum_phase_rad_at_peak |
| --- | --- | --- | --- |
| A | 5.85938 | 0.977684 | 1.92779 |
| B | 5.85938 | 0.977792 | 1.93323 |
| C | 5.85938 | 0.978267 | 1.92853 |
| D | 5.85938 | 0.978768 | 1.93497 |

## Frequency layer: closed-loop empirical FRF H1 estimate

| case | peak_mag_freq_hz_0p5_80 | peak_mag_db | phase_deg_at_peak | coherence_at_peak_mag | coherence_95_threshold |
| --- | --- | --- | --- | --- | --- |
| A | 18.5547 | -33.6173 | 148.388 | 0.0393461 | 0.348164 |
| B | 18.5547 | -33.5873 | 148.595 | 0.0396066 | 0.348164 |
| C | 18.5547 | -33.8465 | 147.023 | 0.037923 | 0.348164 |
| D | 18.5547 | -33.8201 | 147.134 | 0.0381442 | 0.348164 |

## Identified MuJoCo local plants

| joint | a11 | a12 | a21 | a22 | b1 | b2 | r2_q | r2_dq | rmse_q | rmse_dq | samples |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Hip | 1 | 0.00198637 | 0.00225772 | 0.993186 | 1.21261e-05 | 0.00606305 | 1 | 0.999993 | 8.98474e-05 | 0.00206708 | 9596 |
| Knee | 0.999953 | 0.00197093 | -0.0233259 | 0.985465 | 2.99898e-05 | 0.0149949 | 1 | 0.99999 | 0.000112771 | 0.00316078 | 9596 |

## Identified augmented closed-loop poles

| case | joint | delay_steps | max_pole_magnitude | num_poles_outside_unit_circle |
| --- | --- | --- | --- | --- |
| A | Hip | 0 | 0.98126 | 0 |
| A | Hip | 2 | 0.981807 | 0 |
| A | Knee | 0 | 0.954197 | 0 |
| A | Knee | 2 | 0.948035 | 0 |
| B | Hip | 0 | 0.98126 | 0 |
| B | Hip | 2 | 0.981766 | 0 |
| B | Knee | 0 | 0.954197 | 0 |
| B | Knee | 2 | 0.948035 | 0 |
| C | Hip | 0 | 0.98126 | 0 |
| C | Hip | 2 | 0.981807 | 0 |
| C | Knee | 0 | 0.954197 | 0 |
| C | Knee | 2 | 0.948417 | 0 |
| D | Hip | 0 | 0.98126 | 0 |
| D | Hip | 2 | 0.981766 | 0 |
| D | Knee | 0 | 0.954197 | 0 |
| D | Knee | 2 | 0.948417 | 0 |

## Identified augmented Bode response: input disturbance to q error

| case | joint | peak_freq_hz_0p5_80 | peak_mag_db | phase_deg_at_peak | mag_db_at_20_hz |
| --- | --- | --- | --- | --- | --- |
| A | Hip | 2.17944 | -38.9856 | 124.133 | -73.4482 |
| A | Knee | 0.50032 | -38.9985 | 170.701 | -64.6856 |
| B | Hip | 2.17944 | -39.2221 | 124.471 | -73.4338 |
| B | Knee | 0.50032 | -38.9985 | 170.701 | -64.6856 |
| C | Hip | 2.17944 | -38.9856 | 124.133 | -73.4482 |
| C | Knee | 0.50032 | -38.2531 | 170.431 | -64.7647 |
| D | Hip | 2.17944 | -39.2221 | 124.471 | -73.4338 |
| D | Knee | 0.50032 | -38.2531 | 170.431 | -64.7647 |

## Identified augmented singular-value response: measurement noise to control

| case | joint | peak_freq_hz_0p5_80 | peak_mag_db | mag_db_at_20_hz |
| --- | --- | --- | --- | --- |
| A | Hip | 4.20785 | 43.1609 | 41.8759 |
| A | Knee | 44.3217 | 42.0203 | 41.871 |
| B | Hip | 4.20785 | 43.163 | 41.9409 |
| B | Knee | 44.3217 | 42.0203 | 41.871 |
| C | Hip | 4.20785 | 43.1609 | 41.8759 |
| C | Knee | 18.9756 | 41.6945 | 41.693 |
| D | Hip | 4.20785 | 43.163 | 41.9409 |
| D | Knee | 18.9756 | 41.6945 | 41.693 |

Generated files:

- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/frequency/bychen_mujoco_psd_peaks.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/frequency/bychen_mujoco_coherence_summary.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/frequency/bychen_mujoco_empirical_frf_summary.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/linearized/bychen_mujoco_identified_plants.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/linearized/bychen_mujoco_augmented_poles.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/linearized/bychen_mujoco_augmented_eigenvalues.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/linearized/bychen_mujoco_augmented_frequency_response.csv`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_log_psd.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_disturbance_coherence.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_empirical_frf_bode.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_augmented_delay_poles.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_augmented_zplane_poles.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_identified_bode_disturbance.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_identified_noise_singular_value.png`
- `analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis/figures/bychen_mujoco_identified_augmented_frequency_response.png`
