# 文档入口

本目录收纳算法说明、模块说明、仿真使用说明及保留的历史报告。

## 常用文档

- [代码结构说明](CODE_STRUCTURE.md)：核心目录、模块职责、构建入口和测试边界。
- [控制算法说明](控制算法.md)：当前控制器核心递推关系。
- [MPC 方法梳理](MPC方法梳理.md)：预览 MPC 的设计和实现说明。
- [Preview-MPC 算法说明](preview_mpc_algorithm_explainer.md)：`policy_interpolation: preview_mpc` 的简短入口说明。
- [LaTeX 报告模板](reports/latex/template.tex)：用于生成新的实验报告，不包含具体实验内容。

通用工具位于 `scripts/`，具体实验脚本和专用配置位于 `experiments/`。历史 `data/` 和 `analysis_artifacts/` 已退出 Git 跟踪，本地文件继续保留；新产物遵循忽略规则。原有已跟踪报告继续保留，不能仅凭 `.gitignore` 判断是否受版本控制。归档和恢复方式见 [清理记录](CLEANUP.md)。

## 工程入口

- [实验索引](../experiments/README.md)
- [通用工具](../scripts/README.md)
- [环境与依赖](DEPENDENCIES.md)
- [目录迁移说明](REORGANIZATION.md)

> 2026-09-17 远端同步：已补回原仓库跟踪的历史产物；当前 Git 状态、历史路径和子模块引用说明见 [远端同步记录](REMOTE_SYNC.md)。
