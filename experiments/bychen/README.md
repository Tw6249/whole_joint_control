# bychen 专题实验

保留原专题名称和文件名，用于追溯历史实验：

- `run_bychen_mujoco_sweep.py`：MuJoCo 扫参。
- `run_bychen_mujoco_deep_analysis.py`：深入分析与模型辨识相关工具。
- `run_bychen_mujoco_mechanism_completion.py`：机理补充实验。
- `run_bychen_future_work_completion.py`：后续补充实验。
- `run_bychen_full_analysis.py`：已有数据的综合分析。
- `make_bychen_answer_figures.py`：专题图表生成。

从项目根目录运行。部分入口依赖历史 `data/` 或 `analysis_artifacts/` 内容；当前本地工作区保留这些历史结果，但它们已归档并退出 Git 跟踪，新克隆需恢复归档或重跑实验。通用算法分析工具位于 `scripts/analysis/`，报告路径工具位于 `scripts/reporting/report_paths.py`。
