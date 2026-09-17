# 通用工具

从项目根目录运行；具体实验入口见 [实验索引](../experiments/README.md)。

| 目录 | 用途 | 主要入口 |
| --- | --- | --- |
| `simulation/` | MuJoCo 闭环、参数拟合、视频渲染 | `run_mujoco.py`、`fit_mujoco_eid_params.py`、`render_mujoco_joint_video.py` |
| `hardware/` | 实机力矩测试工具 | `run_h1_torque_test.py` |
| `analysis/` | 日志索引、误差、扰动、耦合和求解时间分析 | `index_runs.py`、`analyze_*.py` |
| `reporting/` | 绘图、报告生成与 Markdown 图片检查 | `plot_*.py`、`make_*.py`、`build_*.py`、`check_markdown_assets.py` |

示例：

```bash
python scripts/simulation/run_mujoco.py --duration 1 --export-summary
python -m scripts.analysis.index_runs --help
python scripts/reporting/check_markdown_assets.py docs
```

部分分析和报告脚本默认读取历史日期目录；整理保留这些复现约定。没有相应数据时，应先运行对应实验或按脚本参数指定数据位置。
