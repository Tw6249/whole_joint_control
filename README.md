# Unitree H1 Full-Body Joint Control

## 文档入口

- 纯代码版本的模块说明见 `docs/CODE_STRUCTURE.md`。
- 算法和仿真说明见 [文档索引](docs/README.md)。
- 实验入口与配置对应关系见 [实验索引](experiments/README.md)。
- Python、SDK 和 LibTorch 安装见 [环境依赖](docs/DEPENDENCIES.md)。
- 整理前后的路径对应见 [迁移说明](docs/REORGANIZATION.md)。

H1 人形机器人多关节控制器开发、仿真与实机部署。

## 构建

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

## 配置与控制器

核心配置文件：

| 文件 | 用途 |
|------|------|
| `config/simulation/h1_full_body_mujoco_fit.yaml` | MuJoCo / mock / 实机共用 |

重新拟合 MuJoCo 参数：

```powershell
python scripts/simulation/fit_mujoco_eid_params.py
```

### 控制器类型

```yaml
controller:
  kind: eid           # eid、position_pd 或 online_policy_eid
```

`online_policy_eid` 需要启用 `H1IF_BUILD_TORCH_POLICY`，配置示例见 `experiments/landing_stand/configs/`。

### 关节配置

```yaml
controller:
  kind: eid
  joints:
    2:
      name: RightKnee
      enabled: true
      policy_source: sine
      policy_center: 0.75
      policy_amplitude: 0.10
      policy_frequency_hz: 0.10
      tau_limit: 8
```

`enabled: false` 的关节不会进入控制器算法，但仍用 SafeHold 保持当前位置。

### Policy Reference

```
policy source -> policy point -> policy-period interpolation -> control reference
```

| 插值模式 | 行为 |
|----------|------|
| `open_loop` | 从上一个 policy 点插值到当前 policy 点 |
| `closed_loop` | 每个 policy 周期开始时读取当前 q/dq，向目标插值 |
| `preview_mpc` | 三参考点 soft-preview MPC |
| `preview_mpc_velocity` | 四参考点，增加差分速度目标的 MPC |

最大速度默认使用对应关节的 `joint_limits.<id>.dq_max`。
所有插值模式只把 policy source 当作离散位置点来源；即使仿真 source 是 `sine`，插值器也不会读取解析速度或解析加速度。

`startup_blend_duration_s` 用于实机启动保护，MuJoCo 配置默认 `0.0`。

## 仿真

### MuJoCo

```powershell
python scripts/simulation/run_mujoco.py --config config/simulation/h1_full_body_mujoco_fit.yaml --duration 10.0 --export-summary
```

Python 只做 MuJoCo 物理、stepper 通信和输出。控制算法由 C++ stepper 执行，YAML 中 `controller.kind` 选择控制器。

输出（`--out-dir` 目录下）：
- `mujoco_closed_loop_log.csv` — 逐帧日志
- `summary.csv` — 每关节汇总指标

视频由 `scripts/simulation/render_mujoco_joint_video.py` 单独渲染。

### Mock 快速验证

```powershell
.\build\Debug\h1_mock_closed_loop.exe config/simulation/h1_full_body_mujoco_fit.yaml 5.0
```

用 YAML 中的 plant 近似模型跑 C++ 控制器，不启动 MuJoCo。

## 代码结构

```text
include/                    C++ 控制器、配置、参考生成和安全层
src/                        C++ stepper、mock 与实机程序入口
tests/                      现有 C++ 回归测试
config/simulation/          通用仿真配置
config/hardware/            实机单关节 bring-up 配置
config/policy/              在线策略部署参数
scripts/simulation/         MuJoCo、参数拟合和视频渲染
scripts/analysis/           通用数据分析
scripts/reporting/          绘图、报告及图片引用检查
scripts/hardware/           实机力矩测试工具
experiments/                按主题归档的实验脚本和 configs
models/mujoco/h1/           H1 模型、网格与许可证
models/policies/            TorchScript 模型及说明
reference/matlab/           MATLAB 参考实现
docs/                      算法、结构和环境说明
```

控制关节：0-8, 10-19（共 19 个）。关节 9 是 MJCF 占位，不参与控制。

## 实机运行

实机程序仅在 Ubuntu + Unitree SDK2 环境下构建。

### 构建

```bash
cmake -S . -B build-h1 \
  -DH1IF_BUILD_UNITREE=ON \
  -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
cmake --build build-h1 -j
```

### 三步上机

**1. 只读检查**

```bash
./build-h1/h1_knee_state config/simulation/h1_full_body_mujoco_fit.yaml
```

确认 joint_id、q 方向、lowstate_age 正常。

**2. 单关节 PID bring-up**

```bash
sudo ./build-h1/h1_knee_pid config/simulation/h1_full_body_mujoco_fit.yaml 0.55 8 --arm \
    --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15
```

**3. 多关节 EID**

```bash
sudo ./build-h1/h1_direct config/simulation/h1_full_body_mujoco_fit.yaml
```

## 上机检查单

```
[ ] 急停可用，旁边有人看护
[ ] 机器人悬挂或有可靠机械保护
[ ] network_interface 和 domain_id 与当前环境一致
[ ] h1_knee_state 只读数据正常
[ ] q 正方向、零位、限位与实物一致
[ ] 第一次只开一个 enabled 关节
[ ] policy_amplitude、kp/kd、tau_limit 从小值开始
[ ] MuJoCo 通过不代表实机可直接大幅度运行
```

> 2026-09-17 远端同步：已补回原仓库跟踪的历史产物；当前 Git 状态、历史路径和子模块引用说明见 [远端同步记录](docs/REMOTE_SYNC.md)。

> 后续清理：已归档历史数据、移除旧构建和失效引用；当前状态见 [清理记录](docs/CLEANUP.md)。
