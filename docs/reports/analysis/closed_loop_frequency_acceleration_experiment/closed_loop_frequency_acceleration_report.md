# 闭环 MuJoCo 实验：通过提高参考频率加大关节加速度

本报告是真正重新运行闭环 MuJoCo 实验得到的结果。实验保持髋/膝角度幅值不变，通过提高正弦参考频率来提高关节加速度；正弦加速度峰值随频率平方增长。

## 结论

提高频率后，第 2 行关节间加速度传递力矩确实增大；但第 3 行总残差没有变小。在本次闭环实验中，第 2 行占比随加速度提高而上升，说明耦合传递力更加主导；总残差 RMS 则同步增大，说明这不是抵消残差的机制。

## 原始频率与最高频率对比

| 方法 | 方向 | 频率 Hz | 参考加速度倍率 | 实测 qdd RMS | 第 2 行 RMS | 第 3 行 RMS | 扣除第 2 行后剩余 RMS | 第 2 行占比 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| PD | hip_to_knee | 0.8 | 1.00 | 4.237 | 1.355 | 0.714 | 0.959 | 58.6% |
| PD | knee_to_hip | 0.8 | 1.00 | 5.003 | 1.600 | 1.355 | 1.841 | 46.5% |
| PD | hip_to_knee | 1.4 | 3.06 | 14.339 | 4.546 | 3.787 | 1.051 | 81.2% |
| PD | knee_to_hip | 1.4 | 3.06 | 15.343 | 4.866 | 3.525 | 2.257 | 68.3% |
| EID | hip_to_knee | 0.8 | 1.00 | 4.175 | 1.335 | 0.709 | 0.885 | 60.1% |
| EID | knee_to_hip | 0.8 | 1.00 | 5.229 | 1.672 | 1.420 | 1.846 | 47.5% |
| EID | hip_to_knee | 1.4 | 3.06 | 10.296 | 3.265 | 2.726 | 0.758 | 81.2% |
| EID | knee_to_hip | 1.4 | 3.06 | 15.409 | 4.876 | 3.842 | 2.063 | 70.3% |

## 图

![闭环实验 RMS 变化](../../../../analysis_artifacts/closed_loop_frequency_acceleration_experiment/figures/closed_loop_frequency_acceleration_rms.png)

![闭环实验第 2 行占比变化](../../../../analysis_artifacts/closed_loop_frequency_acceleration_experiment/figures/closed_loop_frequency_acceleration_share.png)

## 输出文件

```text
analysis_artifacts/closed_loop_frequency_acceleration_experiment/closed_loop_frequency_acceleration_summary.csv
analysis_artifacts/closed_loop_frequency_acceleration_experiment/closed_loop_frequency_acceleration_detail.csv
analysis_artifacts/closed_loop_frequency_acceleration_experiment/figures/closed_loop_frequency_acceleration_rms.png
analysis_artifacts/closed_loop_frequency_acceleration_experiment/figures/closed_loop_frequency_acceleration_share.png
```
