# EID 输入逆分析

本报告按当前单关节半隐式欧拉模型计算输入矩阵和伪逆：

$$
g=\begin{bmatrix}T_s^2/J_\mathrm{eff}\\T_s/J_\mathrm{eff}\end{bmatrix},\qquad
g^+=(g^Tg)^{-1}g^T
$$

离线重放项使用

$$
\eta_u^{\mathrm{pinv}}=g^+K_o(x-\hat{x})
$$

该结果只用于离线量级检查，不直接替换实时控制器中的 `ku_q/ku_dq`。

## Config Input Inverse

| joint_id | joint_name | Jeff | g_q | g_dq | pinv_q | pinv_dq | weighted_pinv_q | weighted_pinv_dq | ku_q | ku_dq |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | RightHipPitch | 1.00509 | 3.97976e-06 | 0.00198988 | 1.00508 | 502.541 | 83757.1 | 335.028 | 12 | 1 |
| 2 | RightKnee | 0.250148 | 1.59905e-05 | 0.00799525 | 0.250147 | 125.074 | 20845.7 | 83.3828 | 12 | 1 |

## Log Replay

| joint_id | joint_name | old_eta_u_rms | pinv_eta_u_rms | weighted_eta_u_rms | pinv_over_old_rms | weighted_over_old_rms | corr_old_pinv | old_high_freq_power_ratio | pinv_high_freq_power_ratio |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | RightHipPitch | 0.013209 | 5.91012 | 61.6786 | 447.433 | 4669.45 | 0.916673 | 0.851236 | 0.857627 |
| 2 | RightKnee | 0.0184199 | 2.2837 | 16.6514 | 123.98 | 903.99 | 0.917828 | 0.236671 | 0.128541 |

## Artifacts

- `analysis_artifacts/eid_input_inverse/eid_input_inverse_config_summary.csv`
- `analysis_artifacts/eid_input_inverse/eid_input_inverse_log_summary.csv`
- `analysis_artifacts/eid_input_inverse/eid_input_inverse_detail.csv`
