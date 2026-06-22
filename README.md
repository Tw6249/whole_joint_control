# Unitree H1 Full-Body Joint Control

H1 人形机器人多关节控制器开发、仿真与实机部署。

## 构建

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

## 配置

核心配置文件：

| 文件 | 用途 |
|------|------|
| `config/h1_full_body_mujoco_fit.yaml` | MuJoCo / mock / 实机共用 |

重新拟合 MuJoCo 参数：

```powershell
python scripts/fit_mujoco_eid_params.py
```

### 控制器类型

```yaml
controller:
  kind: eid           # eid 或 position_pd
```

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

最大速度默认使用对应关节的 `joint_limits.<id>.dq_max`。
所有插值模式只把 policy source 当作离散位置点来源；即使仿真 source 是 `sine`，插值器也不会读取解析速度或解析加速度。

`startup_blend_duration_s` 用于实机启动保护，MuJoCo 配置默认 `0.0`。

## 仿真

### MuJoCo

```powershell
python scripts/run_mujoco.py \
    --config config/h1_full_body_mujoco_fit.yaml \
    --duration 10.0 --export-summary
```

Python 只做 MuJoCo 物理、stepper 通信和输出。控制算法由 C++ stepper 执行，YAML 中 `controller.kind` 选择控制器。

输出（`--out-dir` 目录下）：
- `mujoco_closed_loop_log.csv` — 逐帧日志
- `summary.csv` — 每关节汇总指标
- `mujoco_closed_loop.mp4` — 渲染视频

### Mock 快速验证

```powershell
.\build\Debug\h1_mock_closed_loop.exe config/h1_full_body_mujoco_fit.yaml 5.0
```

用 YAML 中的 plant 近似模型跑 C++ 控制器，不启动 MuJoCo。

## 代码结构

```
include/
  controller_factory.hpp      控制器工厂
  eid_controller.hpp          多关节 EID
  position_pd_controller.hpp  多关节位置 PD
  reference_trajectory.hpp    Policy reference 插值器
  runtime_config.hpp          YAML 配置解析
  safety.hpp                  SafeHold、限幅、安全检查

src/
  controller_stepper.cpp      C++ stepper（stdin/stdout 与 Python 通信）
  mock_closed_loop.cpp        本地 mock 闭环
  main_h1_direct.cpp          实机 EID 入口
  main_h1_knee_pid.cpp        实机单关节 PID bring-up
  subscribe_knee_state.cpp    实机只读状态检查

scripts/
  run_mujoco.py               MuJoCo 仿真
  fit_mujoco_eid_params.py    拟合 plant 参数

tests/
  test_safety.cpp             安全层、配置、policy reference 回归测试
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
./build-h1/h1_knee_state config/h1_full_body_mujoco_fit.yaml
```

确认 joint_id、q 方向、lowstate_age 正常。

**2. 单关节 PID bring-up**

```bash
sudo ./build-h1/h1_knee_pid config/h1_full_body_mujoco_fit.yaml 0.55 8 --arm \
    --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15
```

**3. 多关节 EID**

```bash
sudo ./build-h1/h1_direct config/h1_full_body_mujoco_fit.yaml
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

