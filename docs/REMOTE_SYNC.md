# 远端同步记录

同步日期：2026-09-17。

本文记录同步完成时的状态。后续历史数据退出跟踪、旧构建与 gitlink 清理见 [CLEANUP.md](CLEANUP.md)。

- 远端：`https://github.com/Tw6249/whole_joint_control.git`。
- 基准：`origin/main`，提交 `7d5b821`（2026-07-06，Commit full workspace update）。
- 本地分支：`main`，跟踪 `origin/main`。

## 合并方式

原本地目录是没有 Git 历史的代码副本。此次获取远端完整历史，以远端 main 为本地提交父节点，将整理后的本地代码作为其上的增量集成；没有制造无共同祖先的合并，也没有覆盖本地控制器实现。

对照整理前备份，共有 14 个同路径文件内容与远端不同，涉及控制器、参考生成、配置、安全层、日志、测试、构建和文档。本地包含远端没有的在线策略、MPC 预计算与计时、输入补偿增益计算及保护阈值扩展，保留这些实现。另有 32 个本地独有文件，保留在整理后的路径。

远端独有的 1271 个普通文件已原样补回，包括历史数据、分析结果、报告及 build-codex 构建产物，避免把代码包没有携带的内容误提交为删除。这些文件原本已经被 Git 跟踪；`.gitignore` 不会使已跟踪文件自动退出版本控制。当前构建继续使用本机 `build/`，不使用历史 build-codex。

远端 `unitree_rl_gym` 是 gitlink，提交为 `276801e46c5d433564f24658bac64f254b7d2d4b`；保留这一历史引用，但未拉取其内容。远端 `.gitmodules` 没有相应 URL 映射，不能据目录名称推断来源。当前控制工程不依赖它。原失效 Ruckig 声明仍按整理结果移除。

另一个远端分支 `codex/eid-tracking-diagnostics-report-data` 已获取但未合入。它与 main 分叉，包含不同版本的实验实现；本次以默认分支 main 为同步对象。

## 目录与历史产物

当前代码入口见项目 README 和 experiments/README.md。恢复的历史报告、数据 manifest 和 docs/PROJECT_STRUCTURE.md 保留原内容，其中旧路径按 docs/path_migration.json 对照；它们不是现行运行指南。

整理前的文档描述“代码包不含历史产物”适用于下载副本；本次同步后，原仓库已跟踪的历史产物重新出现在本地。新增产物继续遵循忽略规则。

## 备份

合并前的整理版备份位于项目父目录：`whole_joint_control-before-remote-sync-20260917-225542.zip`。远端原版仍可通过 Git 提交访问。

## 验证

C++ Debug 构建、CTest 2/2、0.2 秒 MuJoCo 闭环及 Git 空白检查通过。逐文件核对远端普通文件均有原路径或迁移目标；1271 个补回文件与远端索引一致。未执行实机控制，未推送远端。
