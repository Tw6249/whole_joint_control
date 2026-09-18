# 髋膝实验

## 实机批处理

| 编号 | 内容 | 运行脚本 | 配置文件前缀 |
| --- | --- | --- | --- |
| P1 | PD/EID 无扰动基线 | `run_real_p1_batch.py` | `configs/h1_real_p1_*` |
| P2 | PD/EID 扰动对比 | `run_real_p2_batch.py` | `configs/h1_real_p2_*` |
| P2D | 软件力矩脉冲扰动 | `run_real_p2d_batch.py` | 复用 P2 配置 |
| P3 | PD 三点 MPC 与四点 velocity-MPC | `run_real_p3_mpc_pd_batch.py` | `configs/h1_real_p3_*` |
| P4 | EID Ko/Ku 参数扫描 | `run_real_p4_batch.py` | `configs/h1_real_p4_*` |
| P4D | Ko/Ku 扫描加软件扰动 | `run_real_p4d_batch.py` | 复用 P4 配置 |
| P6.1 | 六候选同批复测矩阵 | `run_real_p61_batch.py` | `configs/h1_real_p61_*` |

`run_real_p1p2_tuning.py` 用于 P1/P2 调参；`analyze_real_p61.py` 与 `plot_real_p1_timeseries.py` 提供专用分析。

P1/P2 的校验、运行排序、日志匹配和执行循环共用 `batch_common.py`；各自入口保留命令行默认值、确认口令、实验元数据和 manifest 格式。

```bash
python experiments/hip_knee/run_real_p1_batch.py --help
python experiments/hip_knee/run_real_all_batches.py
```

总批处理原先引用不存在的 `run_real_p3_batch.py`，其描述为 quintic/Preview-MPC 对比。整理后 P3 明确指向现有 `run_real_p3_mpc_pd_batch.py`，执行三点 MPC/四点 velocity-MPC 对比；这不是缺失历史实验的复原。使用总批处理前以 dry-run 打印的计划为准，也可用 `--only p1,p2,p4` 排除 P3。

## 仿真实验

- `run_hip_knee_domain_experiment.py`：髋膝控制域实验。
- `run_knee_reference_points_experiment.py`：膝关节参考点实验。
- `run_frequency_acceleration_experiment.py`：频率与加速度实验。
- `sweep_hip_knee_source_amplitude.py`：参考源幅值扫描。
- `compare_mpc_velocity_variant.py`：MPC 速度变体对比。

这些入口通过 `scripts/simulation/run_mujoco.py` 使用 C++ stepper。共用配置位于 `config/simulation/`。
