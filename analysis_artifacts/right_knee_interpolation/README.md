# Right Knee Interpolation Analysis

本专题整理右膝单关节 position-only policy 输出下的插值模式对比实验。

## 入口

- 报告：`docs/right_knee_interpolation_report.md`
- 绘图脚本：`scripts/plot_right_knee_interpolation.js`
- 图表：`figures/*.svg`

## 数据来源

主要读取 `data/mujoco_fit/` 中的已有实验输出：

- `track_position_only_open_loop/`
- `track_position_only_ruckig/`
- `track_position_only_rl_smoothed/`
- `right_knee_position_only_interpolation_comparison_metrics.csv`
- `right_knee_closed_loop_tracking_position_only_metrics.csv`

## 复现图表

```powershell
node analysis_artifacts\right_knee_interpolation\scripts\plot_right_knee_interpolation.js
```
