# 实验索引

这里保存具体实验的运行入口、独立配置及专用分析代码。历史编号与参数变体保留，未将任一版本指定为已经验证的最新实机方案。命令从项目根目录运行。

| 实验系列 | 内容 | 入口 |
| --- | --- | --- |
| 髋膝控制 | P1/P2/P3/P4/P6.1、扰动、扫参及仿真参考轨迹实验 | [hip_knee](hip_knee/README.md) |
| 落地站立 | Preview-MPC 与在线 LSTM + EID，各力矩/保护阈值变体 | [landing_stand](landing_stand/README.md) |
| 悬吊站立 | 10 关节 EID 悬吊站立 | [suspended_stand](suspended_stand/README.md) |
| bychen 专题 | 原有专题扫参、机理分析与图表生成 | [bychen](bychen/README.md) |

通用仿真配置在 `config/simulation/`，实机单关节 bring-up 配置在 `config/hardware/`，在线策略部署参数在 `config/policy/`；每个实机实验的专用配置在对应目录的 `configs/`。

实机 `run_real_*` 入口默认 dry-run；`--help` 查看参数。整理检查只运行帮助与 dry-run，没有连接机器人。实验日志仍写入 `data/`，分析产物写入 `analysis_artifacts/`，报告写入 `docs/reports/analysis/`，均由 Git 忽略。
