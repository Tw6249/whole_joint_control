# 分析产物目录

`analysis_artifacts/` 只保存脚本生成的可复算产物，包括 CSV 指标、PNG/PDF 图、配置快照、逐次运行日志和中间结果。

面向阅读的 Markdown 报告和表格已统一归档到 `docs/reports/analysis/`。新增分析脚本应继续把数据和图写到本目录，但不要把 Markdown 报告写到本目录。

## 目录约定

- `figures/`：由脚本生成的图，报告可跨目录引用。
- `configs/`：实验脚本生成或冻结的配置。
- `runs/`：逐次仿真或实机运行记录。
- `*.csv`：可复核的指标、明细和汇总表。

若需要阅读结论，请从 `docs/reports/analysis/README.md` 进入。
