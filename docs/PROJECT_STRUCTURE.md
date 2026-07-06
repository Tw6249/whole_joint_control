# 项目结构约定

本项目按“源码、配置、实验数据、分析结果、文档、报告构建产物”分区管理。新增文件时优先使用下列位置，避免把代码、算法说明、图表和结果散落到根目录。

## 顶层目录

| 路径 | 内容 | 放置规则 |
| --- | --- | --- |
| `include/`, `src/` | C++ 控制器、stepper、实机入口 | 只放可编译源码和头文件。算法实现随控制器模块放置。 |
| `scripts/` | Python 实验、分析、绘图和批处理脚本 | 脚本默认输出到 `data/` 或 `analysis_artifacts/`，不要直接写到根目录。 |
| `config/` | MuJoCo、mock、实机实验 YAML 配置 | 新实验配置按实验名或阶段命名。 |
| `tests/` | C++ 回归测试 | 与控制器安全、配置解析、轨迹生成相关的测试放这里。 |
| `data/` | 原始日志、直接仿真输出、实机采集数据 | 保留可复算的原始输入和逐次运行日志。 |
| `analysis_artifacts/` | 派生指标、CSV 汇总表、论文/报告图、配置和运行记录 | 每个实验或问题一个子目录；不要在这里新增 Markdown 阅读报告。 |
| `docs/` | 面向阅读的 Markdown 文档、汇报素材、交付图 | 正文配图放 `docs/figures/`，交付 PDF 放 `docs/reports/`。 |
| `docs/reports/analysis/` | 从分析产物中整理出的 Markdown 报告和表格 | 报告正文引用 `analysis_artifacts/` 中的原始图表和 CSV。 |
| `docs/reports/latex/` | LaTeX 报告源码和对应报告图 | 保留可重新编译的报告工程；临时编译文件由 `.gitignore` 忽略。 |
| `h1_official_mujoco/` | H1 MuJoCo 模型和官方资源 | 按上游结构保留。 |
| `unitree_rl_gym/` | Unitree RL Gym 上游工程 | 按上游结构保留，不混入本项目分析产物。 |
| `build/`, `tmp/`, `output/` | 构建、临时渲染、一次性输出 | 这些目录是临时产物，默认不作为正式资料入口。 |

## 文档与图片

- Markdown 正文和它的稳定配图应放在同一个归档域内。例如 `docs/实物控制实验结果总结.md` 引用 `docs/figures/...`。
- LaTeX 报告若需要独立编译，优先把报告专用图片放在对应 `docs/reports/latex/<report>/figures/`；若引用全项目分析图，路径应显式指向 `analysis_artifacts/<experiment>/figures/`。
- 移动文档时必须同步更新图片相对路径，并运行 `python scripts/check_markdown_assets.py` 检查本地图片引用。

## 实验结果

- 原始数据和逐次日志放 `data/<experiment>/`。
- 清洗后的指标 CSV、对比图和统计产物放 `analysis_artifacts/<experiment>/`。
- 面向阅读的分析报告和 Markdown 表格放 `docs/reports/analysis/<experiment>/`。
- 面向汇报或论文正文的精选图，可以复制到 `docs/figures/` 或报告工程 `figures/`，但应在正文中说明原始汇总表所在目录。

## 命名建议

- 新实验目录使用小写英文和下划线，例如 `analysis_artifacts/closed_loop_frequency_acceleration_experiment/`。
- 可交付中文文档可以保留中文文件名；代码、脚本和机器读取的配置建议使用英文文件名。
- 不要在根目录新增零散 `.md`、`.png`、`.pdf`、`.csv`。根目录只保留工程入口、构建配置和一级目录。
- 不要在 `analysis_artifacts/` 新增 Markdown 报告；脚本生成报告时写入 `docs/reports/analysis/`。
