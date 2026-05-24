# 项目结构索引

本文档说明当前仓库中各目录的职责、主要入口和整理规则。目标是让代码、实验数据、分析报告和原始材料各有位置，避免后续实验继续堆在根目录。

## 主要入口

| 路径 | 角色 | 说明 |
|---|---|---|
| `README.md` | 项目总入口 | 构建、运行、配置和实验数据库的简要说明。 |
| `include/` | C++ 头文件 | 控制器、参考轨迹、配置解析、安全逻辑等核心实现。 |
| `src/` | C++ 源文件 | 控制器运行入口、mock 闭环、MuJoCo stepper 等。 |
| `scripts/` | Python 工具脚本 | MuJoCo 运行、参数拟合、数据库管理、可视化 dashboard。 |
| `config/` | 运行配置 | 全身、右膝、Ruckig、RL-smoothed 等 YAML 配置。 |
| `tests/` | 单元/安全测试 | 当前主要为 C++ safety/config/interpolation 测试。 |
| `docs/` | 工程文档 | 数据库设计、使用说明、项目结构索引。 |

## 实验与报告

| 路径 | 角色 | 说明 |
|---|---|---|
| `analysis_artifacts/` | 专题分析产物 | 每个子目录是一组可复现分析，包含 `docs/`、`scripts/`、`figures/`、必要配置。 |
| `analysis_artifacts/eid_joint_coupling/` | EID 关节耦合分析 | 单关节/双关节/控制周期 sweep 的分析脚本和报告。 |
| `analysis_artifacts/right_knee_interpolation/` | 右膝插值对比分析 | 右膝单关节插值模式对比报告、绘图脚本和 SVG 图。 |
| `eid_rl_report_package/` | EID-RL 报告包 | 面向阅读/汇报的整理版报告及图表。 |
| `eid-rl/` | EID-RL 原始材料 | 早期记录、原始图片和未标准化材料。保留作溯源，不作为最终入口。 |

## 数据与外部资源

| 路径 | 角色 | 说明 |
|---|---|---|
| `data/` | 本地实验输出 | CSV、Parquet、SQLite 数据库、MuJoCo run 输出。已在 `.gitignore` 中，默认不提交。 |
| `build/` | 本地构建输出 | CMake/MSVC 生成物。已在 `.gitignore` 中，默认不提交。 |
| `third_party/ruckig/` | 第三方 Ruckig 源码 | 在线轨迹生成依赖。若后续改用 submodule 或包管理，可再收敛。 |
| `h1_official_mujoco/` | Unitree H1 MuJoCo 资源 | MJCF、mesh、场景、图片和许可证。 |

## 当前推荐规则

1. 新实验不要直接放根目录。优先在 `analysis_artifacts/<topic>/` 下建立：

   ```text
   docs/
   scripts/
   figures/
   configs/    # 如需要
   README.md
   ```

2. 原始大日志放 `data/`，分析脚本和小型结果图放 `analysis_artifacts/`。

3. 面向汇报的最终 Markdown 和图表放 `eid_rl_report_package/` 或新的 `reports/<topic>/`。

4. 根目录只保留构建入口、README、核心工程文件和确实需要顶层可见的文件。

5. 移动已有文件前先检查 Markdown 图片链接和脚本路径，避免破坏报告复现。

## 建议后续清理

- 将 `test.py` 重命名并移动到 `scripts/` 或删除，如果它只是临时测试。
- 判断 `eid_control.m` 是算法参考还是旧实验脚本；若是参考实现，建议移入 `analysis_artifacts/eid_matlab_reference/` 或 `docs/reference/`。
- 将 `eid-rl/` 中仍有价值的图片和文字归档到 `eid_rl_report_package/` 或 `analysis_artifacts/`，其余保留为 raw materials。
- 若 `third_party/ruckig/` 不需要直接修改，后续可考虑改为 git submodule 或记录版本来源。
