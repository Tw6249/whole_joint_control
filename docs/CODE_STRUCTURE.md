# Unitree H1 控制项目代码结构

本项目包括 C++ 控制核心、MuJoCo 仿真、实机接口，以及可复现的实验和分析代码。历史日志和分析产物已归档并退出 Git 跟踪，本地原路径保留；现有报告仍随仓库保留。详见 [清理记录](CLEANUP.md)。

## 目录职责

| 目录 | 职责 |
| --- | --- |
| `include/` | C++ 控制器接口、EID/PD、参考轨迹、配置、安全层、日志和在线策略 |
| `src/` | stepper、mock、实机多关节控制、PID bring-up、只读状态、力矩日志和原生运动服务入口 |
| `tests/` | C++ 安全、配置、参考生成回归测试，以及 Python 批处理执行与日志匹配测试 |
| `config/` | `simulation/` 通用仿真、`hardware/` bring-up、`policy/` 策略部署 |
| `scripts/` | `simulation/`、`analysis/`、`reporting/`、`hardware/` 通用工具 |
| `experiments/` | 髋膝、落地站立、悬吊站立及 bychen 专题；实机实验配置与入口同组存放 |
| `models/` | MuJoCo 模型资源及 TorchScript 权重 |
| `reference/matlab/` | EID MATLAB 参考实现 |
| `docs/` | 算法、环境、实验导航及 LaTeX 报告模板 |

## C++ 模块

`runtime_config.hpp` 解析配置；`controller_factory.hpp` 按类型构造控制器；`eid_controller.hpp` 和 `position_pd_controller.hpp` 实现控制；`reference_trajectory.hpp` 生成参考；`safety.hpp` 负责安全检查与限幅；`async_csv_logger.hpp` 提供异步日志。可选 `online_policy_eid_controller.hpp` 将 TorchScript 策略连接到 EID 流程。

| 源文件 | 目标 |
| --- | --- |
| `controller_stepper.cpp` | `h1_controller_stepper` |
| `mock_closed_loop.cpp` | `h1_mock_closed_loop` |
| `main_h1_direct.cpp` | `h1_direct` |
| `main_h1_knee_pid.cpp` | `h1_knee_pid` |
| `subscribe_knee_state.cpp` | `h1_knee_state` |
| `log_joint_torques.cpp` | `h1_torque_logger` |
| `h1_loco_cli.cpp` | `h1_loco_cli` |

MuJoCo Python 端推进物理，通过标准输入输出与 C++ stepper 通信。仿真与实机使用同一套 C++ 控制实现。本轮整理没有拆分或重写控制算法和安全逻辑。

## Python 与路径约定

从项目根目录运行。Python 包使用完整模块导入，例如 `scripts.simulation.fit_mujoco_eid_params`，避免依赖所有脚本处于同一目录。各命令入口同时支持直接文件执行和 `python -m`。

`ROOT` / `REPO_ROOT` 指向项目根目录。资源路径和 YAML 中模型路径按项目根目录解析；日志和输出仍写入原 `data/`、`analysis_artifacts/` 和 `docs/reports/analysis/`。

通用默认仿真配置：`config/simulation/h1_full_body_mujoco_fit.yaml`。在线策略参数：`config/policy/h1.yaml`；权重：`models/policies/policy_lstm_1.pt`。

## 构建与验证

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Ubuntu 实机目标通过 `H1IF_BUILD_UNITREE=ON` 启用；TorchScript 通过 `H1IF_BUILD_TORCH_POLICY=ON` 和 `H1IF_LIBTORCH_DIR` 配置。完整说明见 [环境依赖](DEPENDENCIES.md) 和 [项目入口](../README.md)。

实验入口见 [实验索引](../experiments/README.md)，通用脚本见 [工具索引](../scripts/README.md)。
