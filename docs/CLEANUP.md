# 遗留内容清理与恢复

本次清理以 Git 提交 `52001155a100be47bf9f772878727e0e57aa7788` 为起点。实验参数、控制算法、策略模型和原始数据内容保持不变。

## 已处理内容

| 内容 | 处理 |
| --- | --- |
| `build-codex/` | 80 个历史文件退出 Git 跟踪并移入项目外备份的 `retired/`，包括其中日志；当前使用 `build/` |
| `include/demo_pd_controller.hpp` | 未被构建入口、源码或测试引用，退出跟踪并移入 `retired/` |
| `unitree_rl_gym` | 移除无 URL 映射且未初始化的 gitlink；原提交号保存在备份 manifest 和 Git 历史中 |
| `docs/PROJECT_STRUCTURE.md` | 旧正文已备份，原路径保留为指向现行结构说明的短入口 |
| `data/`、`analysis_artifacts/` | 1146 个历史文件退出 Git 跟踪，本地原路径和内容保留；后续由现有忽略规则管理 |
| P1/P2 批处理 | 校验、运行排序、日志匹配和执行循环抽入 `experiments/hip_knee/batch_common.py` |
| MPC 对比脚本 | 默认可执行文件路径从 `build-codex/Debug/` 改为 `build/Debug/`；仍可用参数指定其他构建 |
| 入口文档 | 补充在线策略与 MPC 插值选项，纠正历史产物分发和 Git 跟踪状态的描述 |

P1/P2 原命令入口、参数默认值、确认口令、实验元数据和 manifest 格式保留。现有“返回 -6 且日志完整时允许继续”的行为也保留，本次没有改变实机执行规则。

落地站立变体已复用基础脚本，继续保留其复现实验入口。配置快照、报告图片、左右侧模型网格即使字节相同，也继续保留各自路径。

## 去重归档

项目父目录下的本地备份：

`whole_joint_control-cleanup-backup-20260918-112443/`

- `objects.zip`：按 SHA-256 存储唯一文件内容，相同内容只保存一次。
- `manifest.json`：原路径、大小、SHA-256、起始 Git HEAD 和 gitlink 信息。
- `restore.py`：验证哈希并恢复到一个新目录，拒绝覆盖已有目录。
- `retired/`：移出的旧构建和示例文件，便于直接查看。
- `restored-copy/`：完整恢复验证后保留的未压缩副本，可直接核对清理前文件。

备份覆盖清理前 1467 个已跟踪普通文件，合并为 1200 个唯一内容对象；ZIP 约 379.7 MiB。历史 `data/` 与 `analysis_artifacts/` 中有 171 组重复内容，归档内减少约 29.7 MiB 未压缩重复内容。

去重发生在归档中，本地历史目录继续保留原文件，没有使用会导致副本联动修改的硬链接。备份不包含 `.git/`、原本被忽略的文件或未初始化子模块的内容。

恢复示例，从项目根目录执行：

```powershell
python ../whole_joint_control-cleanup-backup-20260918-112443/restore.py --output ../whole_joint_control-restored
```

该归档仅在本机，不随代码提交。其他克隆可从保留的 Git 历史取回历史数据：

```powershell
git archive --format=zip --output=history.zip 52001155a100be47bf9f772878727e0e57aa7788 data analysis_artifacts
```

先解压到独立目录再按需恢复。退出跟踪不会删除本地数据，也不会缩小已有 `.git` 历史；本次未重写历史。

## 验证

- 所有归档对象 SHA-256 校验通过，并实际恢复了全部 1467 个文件。
- 本地 1146 个历史数据与分析文件逐一对照备份，字节未变。
- P1/P2 重构前后 54 组参数组合的计划与 dry-run 输出完全一致。
- 6 项 Python 回归测试通过，覆盖不执行硬件的 dry-run、元数据和时长匹配、失败中止、完整日志后的既有 -6 处理规则、manifest 和确认口令。
- C++ Debug 构建与 CTest 2/2 通过。
- 文档图片检查发现历史输入逆报告引用的两张 `data_direct_*` 图片在清理前已缺失，已改为缺图与原输出路径说明；未改写实验结论或补造图片。
- 31 份 Markdown 的图片引用检查、修改文件的 Python 3.10 语法检查和 Git 空白检查通过。
- MPC 对比入口使用当前 `build/` 完成两种方法各 0.5 秒 mock 运行及报告生成。日志出现 `0x4`（命令限幅）；此项仅验证路径和流程，不作为控制性能或实机安全结论。

Python 回归命令：

```powershell
python -m unittest discover -s tests -p test_batch_common.py -v
```

本轮没有连接机器人，没有提交或推送 Git 变更。
