# 目录整理记录

日期：2026-09-17。

## 迁移范围

- 将原扁平 `scripts/` 拆成通用工具与具体实验；实验编号和原文件名保留。
- 通用 YAML 按用途归入 `config/` 子目录，具体实验 YAML 归入对应 `experiments/*/configs/`。
- MuJoCo 模型连同网格和许可证迁入 `models/mujoco/h1/`。
- 策略权重迁入 `models/policies/`，部署 YAML 迁入 `config/policy/`，MATLAB 参考实现迁入 `reference/matlab/`。
- 修正 Python 包导入、项目根目录、子进程路径、模型引用、C++ 默认配置与 CMake 测试路径。
- 清理未被当前代码引用的空 RL Gym 目录与失效 Ruckig 子模块声明，补充依赖和实验索引。

完整逐文件对应表见 [path_migration.json](path_migration.json)。旧脚本路径不再保留重复副本；请按对照表更新个人命令或外部自动化。现有历史 CSV/manifest 若记录旧配置路径，应保留原件，并按对照表在分析时定位配置。

## 特别说明

原总批处理 P3 指向缺失的 `run_real_p3_batch.py`；当前改为本项目实际包含的三点 MPC/四点 velocity-MPC 对比入口，并更新计划描述。该变化不代表已恢复原 quintic/Preview-MPC 实验；详情见 [髋膝实验](../experiments/hip_knee/README.md)。

控制增益、关节映射、限幅、确认流程及历史实验编号均保留。参数变体脚本合并、控制器大文件拆分和测试按模块拆分留作后续独立重构。

## 备份

整理前没有 Git 元数据。完整原文件备份位于项目父目录的 `whole_joint_control-before-reorganization-20260917-182047.zip`；恢复时建议先解压到独立目录再比较，不要直接覆盖当前工作区。

仿真检查发现 Python 端仍按 33 个调试槽读取、C++ 已输出 40 个；现已将 Python 协议长度对齐到 `kJointDebugSize=40`，保留原信号命名并将新增槽输出为 `debug_33` 到 `debug_39`。

## 本次验证结果

- CMake Debug 构建通过；CTest 2/2 通过。
- 38 个 argparse/包装脚本的 `--help` 入口通过；Python 文件兼容 Python 3.10 语法。
- 16 个 `run_real_*` 默认 dry-run：8 个通过，另 8 个在缺少 `build-h1/h1_direct` 的环境检查处停止。未运行 `--execute`，没有实机验证。
- 三个 MuJoCo XML 均可加载；文件入口和 `python -m` 入口均完成 0.2 秒闭环及汇总导出。这仅验证接口连通，不证明长期稳定性。
- 迁移表中的 144 个目标文件均存在；模型、网格、图像与权重的 SHA-256 与备份一致。
- 全部迁移 YAML 经解析比对，仅资源路径发生预期变化，控制参数不变。
- 文档图片引用检查通过。三张未包含的历史实验图改为输出路径说明，未生成或补造实验结果。

验证产物位于忽略目录 `tmp/`；本次 Python 验证使用项目 `.venv/`，它继承本机已有科学计算库并安装缺少的依赖，不是可复现的历史环境锁。

> 2026-09-17 远端同步：已补回原仓库跟踪的历史产物；当前 Git 状态、历史路径和子模块引用说明见 [远端同步记录](REMOTE_SYNC.md)。

> 后续清理：已归档历史数据、移除旧构建和失效引用；当前状态见 [清理记录](CLEANUP.md)。
