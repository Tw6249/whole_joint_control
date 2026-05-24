# Analysis Artifacts

本目录保存专题分析产物。每个子目录对应一个分析主题，优先包含：

- `docs/`：分析报告或实验说明。
- `scripts/`：生成数据、统计指标或绘图的脚本。
- `figures/`：报告引用的小型图表。
- `configs/`：该专题使用的派生配置。
- `data_index.md`：如数据分散在 `data/` 中，可用它记录来源。

原始大日志、SQLite 数据库、Parquet 等运行数据默认放在 `data/`，不直接复制到这里。

## 当前专题

| 专题 | 入口报告 | 说明 |
|---|---|---|
| `eid_joint_coupling/` | `eid_joint_coupling/docs/eid_joint_coupling_analysis.md` | EID 单关节、髋膝耦合、控制周期 sweep 分析。 |
| `right_knee_interpolation/` | `right_knee_interpolation/docs/right_knee_interpolation_report.md` | 右膝单关节插值模式对比实验。 |
