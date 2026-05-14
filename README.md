# Unitree H1 底层控制接口 — 说明书

本工程实现 Unitree H1 的 EID（Equivalent Input Disturbance）单关节力矩前馈控制器，提供离线验证、安全保护、异步日志和实机 DDS 通信入口。默认实验对象为 H1 右膝（motor index 2）。

---

## 目录

1. [工程结构](#1-工程结构)
2. [构建与运行](#2-构建与运行)
3. [架构总览](#3-架构总览)
4. [接口契约](#4-接口契约)
5. [控制器生命周期](#5-控制器生命周期)
6. [电机模式](#6-电机模式)
7. [安全保护层](#7-安全保护层)
8. [SafeHold](#8-safehold)
9. [EID 控制器](#9-eid-控制器)
10. [参考轨迹](#10-参考轨迹)
11. [逆模型](#11-逆模型)
12. [YAML 配置](#12-yaml-配置)
13. [异步 CSV 日志](#13-异步-csv-日志)
14. [离线闭环验证](#14-离线闭环验证)
15. [实机运行](#15-实机运行)
16. [DDS 通信](#16-dds-通信)
17. [安全层回归测试](#17-安全层回归测试)
18. [上实机前](#18-上实机前)

---

## 1. 工程结构

```text
include/
├── controller_interface.hpp    RobotState / RobotCommand / IController 接口
├── safety.hpp                  H1 电机 mode、限幅、SafeHold
├── eid_controller.hpp          EID 力矩前馈控制器
├── reference_trajectory.hpp    分段五次插值参考轨迹
├── demo_pd_controller.hpp      简单 PD 控制器
├── async_csv_logger.hpp        异步无锁 CSV 日志
└── runtime_config.hpp          YAML 配置解析（零外部依赖，~180 行）
src/
├── mock_closed_loop.cpp        离线闭环验证（无 DDS，本地即可运行）
├── main_h1_direct.cpp          实机 DDS 控制入口（需 Unitree SDK2）
├── main_h1_knee_pid.cpp        慢速单膝 PID bring-up 入口（需 Unitree SDK2）
└── subscribe_knee_state.cpp    只读订阅目标膝关节状态（需 Unitree SDK2）
tests/
└── test_safety.cpp             安全层回归测试
config/
└── h1_right_knee.yaml          默认右膝实验配置
scripts/
└── plot_eid_log.py             CSV 日志可视化
```

---

## 2. 构建与运行

### 本地构建（Windows/MinGW）

```bash
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build -j 4

# 构建完成后运行测试
ctest --test-dir build --output-on-failure

# 手动运行 5 秒闭环验证
./build/h1_mock_closed_loop.exe config/h1_right_knee.yaml 5.0
```

mock 运行生成 `data/YYYYMMDD_HHMMSS/h1_mock_log.csv`（`resolveLogPath` 自动按时间戳建子目录）。

退出码：`0`=正常，`1`=异常，`2`=非有限值，`3`=安全故障。

### 绘图

```bash
python scripts/plot_eid_log.py data/YYYYMMDD_HHMMSS/h1_mock_log.csv
```

### 实机构建（Ubuntu + Unitree SDK2）

```bash
cmake -S . -B build-h1 \
  -DH1IF_BUILD_UNITREE=ON \
  -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
cmake --build build-h1 -j

# 只读检查状态
./build-h1/h1_knee_state config/h1_right_knee.yaml

# 谨慎单膝 PID bring-up（target_q=0.35 rad, 持续 8 秒）
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.35 8 --arm

# 如果默认力矩太小，可以逐步提高（见下方 bring-up 硬上限）
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.55 8 --arm --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15

# 正弦轨迹：第二个参数为中心角，先慢速移动到中心，再开始 sine
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.90 30 --arm --sine --amp 0.35 --freq 0.08 --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.30

# EID 控制器（参考轨迹在 YAML 中配置）
sudo ./build-h1/h1_direct config/h1_right_knee.yaml
```

仿真环境（unitree_mujoco）使用 `network_interface: lo`、`domain_id: 1`。

注意：`h1_direct`、`h1_knee_state`、`h1_knee_pid` 三个实机目标只在 `-DH1IF_BUILD_UNITREE=ON` 时编译。每个目标对应一个独立的 .cpp 文件及其 `main()` 函数。

---

## 3. 架构总览

```
┌──────────────────────────────────────────────────────┐
│  main_h1_direct.cpp (实机入口)                        │  ← Ubuntu + Unitree SDK2
├──────────────────────────────────────────────────────┤
│  mock_closed_loop.cpp (离线验证)                      │  ← 本地即可运行
├──────────────────────────────────────────────────────┤
│  控制器层                                             │
│  ├── eid_controller.hpp         (EID 力矩前馈)        │
│  ├── reference_trajectory.hpp   (五次插值参考轨迹)     │
│  └── demo_pd_controller.hpp     (简单 PD)             │
├──────────────────────────────────────────────────────┤
│  安全层  safety.hpp                                   │  ← 所有命令必经此层
├──────────────────────────────────────────────────────┤
│  接口契约  controller_interface.hpp                   │  ← 统一接口
├──────────────────────────────────────────────────────┤
│  支撑层                                               │
│  ├── async_csv_logger.hpp  (异步日志)                 │
│  └── runtime_config.hpp   (YAML 配置)                 │
└──────────────────────────────────────────────────────┘
```

**数据流**：

```
[真机 DDS / 虚拟 Plant] → RobotState
                              │
                              ▼
                    Controller.reset()  (一次性初始化)
                              │
                    Controller.step()   (500Hz, 每 2ms 调用)
                              │
                              ▼
                    applySafety()  ← 限幅 + 有限值检查 + 超时检测
                              │
                              ▼
                    [真机 DDS / 虚拟 Plant]
                              │
                              ▼
                    AsyncCsvLogger → CSV 文件
```

核心设计原则：**控制器可以激进，安全层是最后一道防线**。

---

## 4. 接口契约

`include/controller_interface.hpp` 定义了整个工程共享的核心数据结构。

### RobotState

```cpp
struct RobotState {
    uint64_t cycle;
    double t, dt;                            // 时间和实际步长
    double lowstate_age;                     // 距上次收到 LowState 的时长
    std::array<JointState, 20> joint;        // q/dq/tau_est
    ImuState imu;                            // 四元数 + 角速度 + 加速度
    bool state_valid;
};
```

### JointCommand

```cpp
struct JointCommand {
    float q;        // 期望位置 (rad)
    float dq;       // 期望速度 (rad/s)
    float kp;       // 位置刚度
    float kd;       // 速度阻尼
    float tau;      // 前馈力矩 (N·m)
    uint8_t mode;   // 0x01=位置模式, 0x0A=MIT 模式
    bool enable;
};
```

### IController

```cpp
class IController {
    virtual std::string name() const = 0;
    virtual void reset(const RobotState& state) = 0;
    virtual void step(const RobotState& state, RobotCommand& command,
                      ControllerDebug& debug) = 0;
};
```

所有控制器实现此接口，上层代码不依赖具体控制器类型。`ControllerDebug` 包含 128 个 double 的数据槽 + 32 位 flags。

---

## 5. 控制器生命周期

### reset(const RobotState& state)

一次性初始化：
- 记录当前关节位置 `q_start_`、`dq_start_` 作为参考零点
- 记录起始时间 `t0_`
- 初始化观测器状态：`x_hat_q_`/`x_hat_dq_` 归零，`eta_q_` = 当前实测位置，`eta_dq_` = 当前实测速度
- 初始化参考轨迹规划器

### step(const RobotState& state, RobotCommand& command, ControllerDebug& debug)

每个控制周期调用一次（500Hz，2ms）：

- **入参**：当前机器人状态
- **出参**：20 个关节的命令 + 调试数据（28 个 double 写入 debug.data[0..27]）

EID 控制器内部执行：参考轨迹采样 → startup ramp 平滑 → 扰动补偿参考修正 → 逆模型前馈 → PD 反馈 → 观测器更新。

控制器在 `step()` 入口自动填充所有关节为 SafeHold 默认值（`kp=torque_safe_kp`、`kd=torque_safe_kd`、`tau=0`），然后仅覆写目标关节的命令。这样即使 EID 只控制单关节，非目标关节也保持阻尼状态。

---

## 6. 电机模式

`h1MotorMode(int joint_id)` 定义在 `include/safety.hpp:33-47`，策略为**下肢 MIT 模式 `0x0A`，手臂位置模式 `0x01`**：

| motor index | H1 SDK2 名称 | mode | 说明 |
|-------------|-------------|------|------|
| 0 | RightHipRoll | `0x0A` | 右髋侧摆 |
| 1 | RightHipPitch | `0x0A` | 右髋前摆 |
| 2 | RightKnee | `0x0A` | 右膝（默认目标关节） |
| 3-8 | 左髋/腰/髋偏航 | `0x0A` | 下肢 |
| 9 | NotUsedJoint | `0x0A` | SDK2 中标记为未用 |
| 10-11 | LeftAnkle/RightAnkle | `0x0A` | 脚踝（weak motor，本实验纳入 MIT 模式） |
| 12-15 | RightShoulder/Elbow | `0x01` | 右臂 weak motor |
| 16-19 | LeftShoulder/Elbow | `0x01` | 左臂 weak motor |

### MIT 模式（0x0A）

电机实际输出力矩：

```
τ_actual = kp*(q_des - q_actual) + kd*(dq_des - dq_actual) + τ_ff
```

EID 控制器发给 MIT 模式的命令策略为 `cmd.q = actual.q`、`cmd.dq = 0`，所以 `kd` 项提供速度阻尼，`tau` 提供主控制力矩。

编号来源：
- [H1 20DOF motor index](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/h1/low_level/motors.hpp)
- [weak/strong motor mode](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/h1/low_level/humanoid.hpp)
- [H1-2/27DOF 差异](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/h1/low_level/h1_27dof_example.cpp)
- [H1 URDF 关节限位](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/h1_description/urdf/h1.urdf)

---

## 7. 安全保护层

`include/safety.hpp` 是**不信任控制器的最后一道闸门**。`applySafety()` 对每个命令逐关节执行：

| 检查项 | 条件 | 触发动作 |
|--------|------|----------|
| 状态无效 | `state_valid == false` 或关节状态含 NaN/Inf | SafeHold |
| LowState 超时 | `lowstate_age > lowstate_timeout`（默认 50ms） | SafeHold |
| 命令含非有限值 | 命令字段为 NaN/Inf | SafeHold |
| 命令超限 | q/dq/kp/kd/tau 超出 joint_limits | 限幅到边界值 |

### 命令限幅（以右膝 joint 2 为例）

| 参数 | 限幅范围 | 物理依据 |
|------|----------|----------|
| q | -0.26 ~ 2.05 rad | H1 URDF 膝关节行程 |
| dq | ±14.0 rad/s | H1 URDF 速度参考 |
| kp | 0 ~ 120 | 保守上限 |
| kd | 0 ~ 5 | 保守上限 |
| tau | ±12 N·m | 当前保守实验上限（低于 URDF effort 300） |

配置文件已补齐 20 个 motor index 的限幅。

### 安全标志位

```cpp
enum SafetyFlags : uint32_t {
    kSafetyLowStateTimeout  = 1u << 0,   // LowState 超时
    kSafetyNonFiniteCommand = 1u << 1,   // 命令含 NaN/Inf
    kSafetyCommandSaturated = 1u << 2,   // 命令被限幅
    kSafetyInvalidState     = 1u << 3    // 状态无效
};
```

### 检查顺序

`applySafety()` 的执行顺序：先检查全局状态（`state_valid`、`lowstate_age`、`finiteState`）→ 若任何不满足则直接 SafeHold；然后逐关节检查命令含 NaN/Inf → 限幅。被安全层覆盖的命令会同时置位对应 flags。

### 四层限速/限幅参数

EID 实机入口中的限制从内到外分为 4 层。调参时优先确认最外层安全边界，再调整内层控制器参数。

**第 1 层：EID 控制器内部**

位置：`include/eid_controller.hpp` 中的 `limitTorqueCommand()` 与 `kneeForward()`。

| 参数 | YAML 路径 | 默认/当前值 | 作用 |
|------|-----------|-------------|------|
| `eid_tau_limit` | `controller.eid_tau_limit` | 12.0 N·m | 控制器输出硬限幅：`clamp(tau, -limit, +limit)` |
| `eid_tau_slew_rate` | `controller.eid_tau_slew_rate` | 60.0 N·m/s | 转矩斜率限制：每步最大变化量为 `slew_rate * dt` |
| `plant.tau_max` | `plant.tau_max` | 结构体默认 80.0 N·m；当前 YAML 为 12.0 N·m | 逆模型 `u_star` 和正模型 forward pass 的力矩限幅 |
| `plant.q_min/q_max` | `plant.q_min`, `plant.q_max` | -0.26 / 2.05 rad | 正模型中的关节位置硬限；超出时位置钳住，越界方向速度置零 |

配置位置：`config/h1_right_knee.yaml` 的 `controller.*` 和 `plant.*` 段；结构体定义在 `include/runtime_config.hpp`。

**第 2 层：安全层**

位置：`include/safety.hpp` 的 `applySafety()`，在控制器输出后再次限幅。

| 参数 | YAML 路径 | 当前右膝值 | 作用 |
|------|-----------|------------|------|
| `q_min/q_max` | `joint_limits.2.q_min/q_max` | -0.26 / 2.05 rad | 最终发往电机的位置指令限幅 |
| `dq_max` | `joint_limits.2.dq_max` | 14.0 rad/s | 最终发往电机的速度指令限幅 |
| `tau_max` | `joint_limits.2.tau_max` | 12.0 N·m | 最终下发到电机的转矩限幅 |
| `kp_max/kd_max` | `joint_limits.2.kp_max/kd_max` | 120.0 / 5.0 | 刚度/阻尼系数限幅 |

配置位置：`config/h1_right_knee.yaml` 的 `joint_limits` 段，第 0~19 号关节各有一套独立限值。

**第 3 层：运行时硬件熔断**

位置：`src/main_h1_direct.cpp` 的 `checkMeasuredTrip()`，在进入控制器之前检查；触发后直接切到 SafeHold。

| 检查项 | 值/阈值 | 来源 |
|--------|---------|------|
| `kMaxMeasuredSpeed` | 8.0 rad/s | `main_h1_direct.cpp` 硬编码 |
| `kMaxMeasuredJump` | 0.10 rad，相邻两步的位置跳变 | `main_h1_direct.cpp` 硬编码 |
| `kMaxControlDt` | 0.010 s，控制循环超时判断 | `main_h1_direct.cpp` 硬编码 |
| `lowstate_timeout` | 0.05 s | `safe_hold.lowstate_timeout` + `safety.hpp` |

**第 4 层：安全保持**

触发熔断或安全层发现无效状态后，使用 `fillSafeHoldCommand()` 生成 fallback 命令。

| 参数 | YAML 路径 | 默认/当前值 | 作用 |
|------|-----------|-------------|------|
| `hold_kp` | `safe_hold.kp` | 10.0 | SafeHold 位置保持刚度 |
| `hold_kd` | `safe_hold.kd` | 1.0 | SafeHold 阻尼 |

配置位置：`config/h1_right_knee.yaml` 的 `safe_hold` 段。

---

## 8. SafeHold

紧急安全姿态，定义在 `safety.hpp:70-80`：

```cpp
void fillSafeHoldCommand(const RobotState& state, RobotCommand& cmd,
                         const SafetyConfig& cfg) {
    for (int i = 0; i < kMaxMotors; ++i) {
        cmd.joint[i].mode = h1MotorMode(i);
        cmd.joint[i].q   = state.joint[i].q;   // 保持当前位置
        cmd.joint[i].dq  = 0.0;
        cmd.joint[i].kp  = cfg.hold_kp;        // 10.0
        cmd.joint[i].kd  = cfg.hold_kd;        // 1.0
        cmd.joint[i].tau = 0.0;
        cmd.joint[i].enable = true;
    }
}
```

**效果**：所有关节定在原地，仅用小 PD 增益维持当前位置。

触发时机：程序启动（填充初始命令）、检测到安全异常、程序退出（Ctrl+C / SIGTERM）。

---

## 9. EID 控制器

`include/eid_controller.hpp` 实现单关节 EID（Equivalent Input Disturbance）力矩前馈控制器。

### 控制流程（每周期）

```
1. SmoothSineReferenceTrajectory.sample(t, dt) → raw_ref (now/next)
2. shapeStartupReference() → 启动 ramp 平滑过渡（4 秒内从实测角度渐变到策略参考）
3. reference_mode 选择最终 ref：
   - `open_loop`：使用 ramp 后的时间参考（原行为）
   - `closed_loop`：`ref.now = measured(q,dq)`，`ref.next` 从实测状态按 `dt / closed_loop_reference_tau` 朝 ramp 后的 next 参考贴近
4. controllerStep(q, dq, ref, dt) 内部：
   a. x_bar = x_hat + eta（标称状态 + 扰动估计）
   b. 用 eta 修正参考增量：r_c_next = ref.next - eta,
      delta_r_c = r_c_next - ref.now
   c. analyticInverseModel(ref.now.q, ref.now.dq, delta_r_c.q, delta_r_c.dq, dt)
      → u_star（前馈力矩）
   d. K_dagger 修正参考：r_d = ref.now + K_dagger · u_star
      （w_q = kp / (kp²+kd²), w_dq = kd / (kp²+kd²)）
   e. PD 反馈：u_raw = kp·(r_d_q - x_bar_q) + kd·(r_d_dq - x_bar_dq)
   f. limitTorqueCommand(u_raw, dt) → u_t（限幅 + slew-rate）
   g. kneeForward(x_bar, u_t, dt) 前向仿真更新 x_hat
   h. 观测器：tilde_x = measured - x_bar, 低通滤波更新 eta
5. u_t 写入 command.joint[target_joint].tau
```

### 关键参数（当前 `config/h1_right_knee.yaml`）

| 参数 | 值 | 含义 |
|------|-----|------|
| target_joint | 2 | 目标关节 motor index（RightKnee） |
| control_dt | 0.002 | 控制周期（秒），由顶层 `control_dt` 传入 |
| kp | 65.0 | PD 反馈比例增益 |
| kd | 15.0 | PD 反馈微分增益 |
| observer_gain_q | 0.25 | 位置观测器增益 |
| observer_gain_dq | 0.25 | 速度观测器增益 |
| filter_alpha | 0.5 | 扰动估计低通滤波系数 |
| reference_mode | open_loop | 参考模式：`open_loop` 为当前开环时间参考，`closed_loop` 为每周期用实测状态重锚定参考 |
| policy_reference_dt | 0.05 | 参考轨迹段长度（秒） |
| closed_loop_reference_tau | 0.05 | 闭环参考贴近 raw 参考的时间常数（秒），仅 `closed_loop` 使用 |
| ref_center | 0.75 | 正弦参考中心（rad） |
| ref_amplitude | 0.55 | 正弦参考幅值（rad） |
| ref_frequency | 0.8 | 正弦参考频率（Hz） |
| ref_phase | -π/2 | 正弦初始相位（-1.5708 rad，从最低点启动） |
| startup_ramp_duration | 4.0 | 启动参考平滑过渡时间（秒） |
| eid_tau_limit | 12.0 | EID 输出力矩限幅（N·m） |
| eid_tau_slew_rate | 60.0 | EID 输出力矩变化率限制（N·m/s） |
| torque_safe_kp | 0.0 | 下发命令中的 kp（MIT 模式只用力矩） |
| torque_safe_kd | 0.8 | 下发命令中的 kd（速度阻尼） |
| inverse_q_weight | 0.0 | 逆模型 q 权重（0=动态默认 0.5/dt²） |
| inverse_dq_weight | 0.0 | 逆模型 dq 权重（0=动态默认 1.0） |

### tau 命名说明

工程里有几类 `tau`，含义不同：

| 名称 | 位置/日志列 | 含义 |
|------|-------------|------|
| `u_star` | `debug_6` | 逆模型计算出的前馈力矩候选值，受 `plant.tau_max` 限制 |
| `u_raw` | `debug_25` | PD/EID 合成后的未限幅控制力矩 |
| `u_t` / `cmd.tau` | `debug_8` / `tau_cmd` | 最终写入目标关节命令的前馈力矩，受 `eid_tau_limit`、`eid_tau_slew_rate` 和安全层 `joint_limits.*.tau_max` 约束 |
| `tau_est` | LowState / CSV `tau_est` | 电机状态反馈中的估计力矩 |
| `tau0` | `plant.tau0` | 被控对象模型的偏置力矩 |
| `closed_loop_reference_tau` | `controller.closed_loop_reference_tau` | 闭环参考贴近 raw 参考的时间常数，单位是秒；它不是力矩 |

### 决策周期与控制周期

| 参数 | YAML 路径 | 当前值 | 作用 |
|------|-----------|--------|------|
| 控制周期 `control_dt` | 顶层 `control_dt`，加载后写入 `controller.control_dt` | 0.002 s | 实时控制循环周期，`controller.step()` 每 2 ms 调一次，约 500 Hz |
| 决策/策略周期 `policy_reference_dt` | `controller.policy_reference_dt` | 0.05 s | 参考策略分段周期，正弦 raw policy 每 50 ms 生成一个策略段端点 |
| 闭环参考时间常数 `closed_loop_reference_tau` | `controller.closed_loop_reference_tau` | 0.05 s | 仅 `reference_mode: closed_loop` 使用；每个控制周期贴近比例约为 `control_dt / closed_loop_reference_tau` |

当前配置下，`policy_reference_dt / control_dt = 25`，即每个策略段包含 25 个 2 ms 控制周期。`SmoothSineReferenceTrajectory` 会在策略段内生成五次插值节点，控制器每个控制周期取 `ref.now/ref.next` 用于逆模型前馈。

### 被控对象模型（PlantModelConfig）

```cpp
struct PlantModelConfig {
    double Jeff = 0.238;       // 等效转动惯量 (kg·m²)
    double b = 1.0;            // 粘性阻尼系数
    double gravityA = 4.2835;  // 重力项 sin 系数
    double gravityB = 0.0;     // 重力项 cos 系数
    double tau0 = -0.2711;     // 偏置力矩 (N·m)
    double q_min = -0.26;      // 关节位置下限 (rad)
    double q_max = 2.05;       // 关节位置上限 (rad)
    double tau_max = 12.0;     // 最大力矩 (N·m)
};
```

重力补偿：`τ_gravity = gravityA · sin(q) + gravityB · cos(q)`

注意：`plant.tau_max` 在 YAML 中仅为 12 N·m，而结构体默认值为 80 N·m，YAML 覆盖生效。

---

## 10. 参考轨迹

`include/reference_trajectory.hpp` 实现了 `SmoothSineReferenceTrajectory`，以**分段五次插值**生成平滑参考轨迹。

### 工作原理

1. 将时间按 `policy_reference_dt`（默认 0.05s）分段
2. 每个段的首尾节点由正弦策略计算：`q(t) = center + amplitude · sin(2π · frequency · t + phase)`
3. 段内用五次多项式连接，保证位置、速度、加速度连续
4. 当检测到 `t` 回退或参数变化时自动重新初始化规划器
5. `sample(t, dt)` 返回当前和下一时刻的参考值（用于逆模型前馈）

### 当前 YAML 参数

| 参数 | 值 | 含义 |
|------|-----|------|
| policy_reference_dt | 0.05 s | 轨迹段长度 |
| reference_mode | open_loop | `open_loop` 直接使用时间参考；`closed_loop` 每周期用当前实测状态生成局部参考 |
| closed_loop_reference_tau | 0.05 s | 闭环参考从实测状态贴近时间参考的时间常数 |
| ref_center | 0.75 rad | 正弦中心 |
| ref_amplitude | 0.55 rad | 正弦幅值 |
| ref_frequency | 0.80 Hz | 正弦频率 |
| ref_phase | -π/2 rad | 正弦初始相位（从最低点启动） |

正弦轨迹范围为 [0.20, 1.30] rad，在右膝 soft limit 内。

### makeReferenceConfig 安全校验

`EidSingleJointController::makeReferenceConfig()` 在实际使用配置前做保护性校验：
- `ref_center` 被 clamp 到 `[model.q_min, model.q_max]` 与 `[0, 1.5]` 的交集内
- `ref_amplitude` 被 clamp 到 `[0, min(center-q_min, q_max-center)]` 内，确保不超出关节限位
- `ref_frequency` 上限 0.8 Hz
- `ref_phase` 若为非有限值则默认 -π/2

---

## 11. 逆模型

`analyticInverseModel()` 计算前馈力矩 `u_star`。

给定当前位置 (q, dq) 和目标增量 (Δq, Δdq)，分别从位置通道和速度通道反算所需力矩，再按权重融合：

```
τ_from_q  = bias + Jeff · ((Δq - dt·dq) / dt²)
τ_from_dq = bias + Jeff · (Δdq / dt)
u_star    = (w_q · a_q² · τ_from_q + w_dq · a_dq² · τ_from_dq) / den
```

其中：
- `bias = b · dq + gravityA·sin(q) + gravityB·cos(q) + tau0`
- `a_q = dt² / Jeff`，`a_dq = dt / Jeff`
- `w_q = inverse_q_weight`（默认 `0.5 / dt²`），`w_dq = inverse_dq_weight`（默认 `1.0`）
- `den = w_q · a_q² + w_dq · a_dq²`

结果 clamp 到 `[-tau_max, tau_max]`。

`K_dagger` 修正将 `u_star` 映射为参考偏移：
```
w_q_修正 = kp / (kp² + kd²)
w_dq_修正 = kd / (kp² + kd²)
r_d_q  = ref.now.q + w_q_修正 · u_star
r_d_dq = ref.now.dq + w_dq_修正 · u_star
```

---

## 12. YAML 配置

`include/runtime_config.hpp` 是零外部依赖的手写 YAML 解析器（~180 行），将 YAML 映射为 C++ 结构体。包含输入校验（`validateRuntimeConfig`），在解析完成后检查所有参数的有效性。

**为什么用配置文件**：调参不需重新编译；不同实验用不同配置；参数集中管理。

### 解析规则

- `#` 开头为注释
- 缩进表示层级（每层 2 空格）
- `key: value` 为键值对
- 顶层无 value 的 key 是 section 标记

### 当前配置（`config/h1_right_knee.yaml`）

```yaml
robot: H1
domain_id: 0
network_interface: eth0
control_dt: 0.002
mock_duration: 5.0
log_path: data/h1_mock_log.csv

safe_hold:
  kp: 10.0
  kd: 1.0
  lowstate_timeout: 0.05

controller:
  target_joint: 2
  kp: 65.0
  kd: 15.0
  observer_gain_q: 0.25
  observer_gain_dq: 0.25
  filter_alpha: 0.5
  reference_mode: open_loop
  policy_reference_dt: 0.05
  closed_loop_reference_tau: 0.05
  ref_center: 0.75
  ref_amplitude: 0.55
  ref_frequency: 0.8
  ref_phase: -1.5707963267948966
  startup_ramp_duration: 4.0
  eid_tau_limit: 12.0
  eid_tau_slew_rate: 60.0
  torque_safe_kp: 0.0
  torque_safe_kd: 0.8
  inverse_q_weight: 0.0
  inverse_dq_weight: 0.0

plant:
  Jeff: 0.238
  b: 1.0
  gravityA: 4.2835
  gravityB: 0.0
  tau0: -0.2711
  q_min: -0.26
  q_max: 2.05
  tau_max: 12.0

joint_limits:
  2:  # RightKnee
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 12.0
    kp_max: 120.0
    kd_max: 5.0
  # ... 全部 20 个 motor index 已补齐
```

### 日志路径自动解析

`resolveLogPath()` 将 `log_path` 中的文件名放入按当前时间戳命名的子目录，例如 `log_path: data/h1_mock_log.csv` → `data/20260514_093015/h1_mock_log.csv`。

---

## 13. 异步 CSV 日志

`include/async_csv_logger.hpp` 实现生产者-消费者模式：

```
控制线程 (生产者)              写文件线程 (消费者)
      │                              │
   push(sample) ──→ 环形缓冲 ──→ pop() → 写磁盘
      │              (8192)           │
    无锁 CAS           │          每 20ms drain
```

- 容量 8192 条（约 16 秒数据），满时丢弃新样本（计数器累加，不阻塞）
- 使用 `std::atomic` 实现无锁 head/tail
- 控制循环 500Hz 下 `push()` 仅做一次原子 CAS

### CSV 输出列

```
cycle, t, dt, lowstate_age, joint_id,
q, dq, tau_est,             ← 测量值
q_cmd, dq_cmd, kp_cmd, kd_cmd, tau_cmd,  ← 命令值
flags,                      ← 安全标志位
debug_0 ~ debug_31          ← 控制器内部变量（32 列）
```

其中 `debug_0~debug_27` 被 EID 控制器填充（28 个有效值），包括：参考位置/速度、实测位置/速度、跟踪误差、前馈力矩、反馈力矩、最终输出力矩、观测器变量等。

---

## 14. 离线闭环验证

`src/mock_closed_loop.cpp` 在无需真机的情况下验证完整控制链路。

### 流程

1. 加载 YAML 配置，`resolveLogPath` 生成带时间戳的日志路径
2. 根据策略参考的初始相位计算虚拟 Plant 的初始位置：`q0 = center + amplitude·sin(phase)`
3. 初始化虚拟 Plant（使用 PlantModelConfig 参数和半隐式欧拉前向仿真）
4. 初始化 EID 控制器
5. 循环（500Hz，默认 5 秒）：
   - Plant 状态 → RobotState
   - `controller.step()` → 命令
   - `applySafety()` → 安全检查
   - `stepPlant()` → 前向仿真（含关节限位碰撞）
   - 计算 RMSE、记录最大力矩
   - 推送日志样本
6. 输出汇总：位置 RMSE、最大力矩、安全标志、日志丢弃数
7. 生成 `data/YYYYMMDD_HHMMSS/h1_mock_log.csv`

退出码：`0`=正常，`1`=异常，`2`=非有限值，`3`=安全故障。

### 与实机的差异

| 方面 | mock_closed_loop | main_h1_direct（实机） |
|------|------------------|------------------------|
| 关节动力学 | 简化单摆模型（半隐式欧拉） | 真实物理关节 |
| 传感器 | 模型直接计算 | DDS LowState |
| 命令执行 | `stepPlant()` 函数 | DDS LowCmd → 电机 |
| 延迟/噪声/摩擦 | 无 | 真实 |
| 实时调度 | 无 | SCHED_FIFO + mlockall |
| 初始位置 | 由 `ref_center + amplitude·sin(phase)` 计算 | 机器人当前实际位置 |

**mock 通过是必要条件，不是充分条件。**

---

## 15. 实机运行

`src/main_h1_direct.cpp`、`src/subscribe_knee_state.cpp` 和 `src/main_h1_knee_pid.cpp` 仅在 `-DH1IF_BUILD_UNITREE=ON` 时编译。每个文件有独立的 `main()` 函数。

### 只读检查目标膝关节状态

在发送任何控制命令之前，先运行只读订阅器确认 DDS、网卡、domain 和目标关节编号正确：

```bash
./build-h1/h1_knee_state config/h1_right_knee.yaml
```

该程序**只订阅 `rt/lowstate`，不会发布 `rt/lowcmd`**（代码中根本没有创建任何 Publisher）。它读取 YAML 中的 `controller.target_joint`，默认是 `2`（RightKnee），并以 CSV 行打印：

```text
t_s,joint_id,q_rad,dq_rad_s,tau_est_nm,lowstate_age_s
```

### 慢速单膝 PID bring-up

`h1_knee_pid` 是单独的保守实机入口，用于小步验证右膝闭环。它只控制 YAML 中的 `controller.target_joint`，默认 `2`（RightKnee）。目标角必须显式给出并带 `--arm`：

```bash
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.35 8 --arm
```

参数解析：`<config.yaml>` → `<target_q_rad>` → `<run_seconds>` → `--arm`（确认武装，防止手滑）。

如果默认参数太保守，可以显式提高 PID 与力矩上限。程序仍会限制最大值：

```bash
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.55 8 --arm --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15
```

正弦轨迹模式下，第二个位置参数是正弦中心角。程序会先从当前实测角度慢速移动到中心角，然后开始正弦参考：

```bash
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.90 30 --arm --sine --amp 0.35 --freq 0.08 --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.30
```

停止命令：

```bash
sudo pkill -INT h1_knee_pid
```

也可以在运行终端按 `Ctrl+C`。程序收到 SIGINT/SIGTERM 后会退出 PID 循环，并继续发送约 1 秒的低刚度 hold/damping 命令。

内置 bring-up 硬限制（代码常量，不可被命令行覆盖）：

| 限制项 | 值 | 说明 |
|--------|-----|------|
| 目标角距 soft limit 边距 | ≥ 0.03 rad | 防止碰撞机械限位 |
| 目标角距启动角度 | ≤ 1.50 rad | 防止大步长跳跃 |
| 默认参考速度 | 0.10 rad/s | 保守起步 |
| 最大允许 CLI 参考速度 | 0.80 rad/s | 命令行可覆盖上限 |
| 默认力矩限幅 | 6 N·m | 保守起步 |
| 最大允许 CLI 力矩限幅 | 18 N·m | 命令行可覆盖上限 |
| kp 上限 | 40 | 命令行可覆盖上限 |
| ki 上限 | 5 | 积分增益上限 |
| kd 上限 | 5 | 微分增益上限 |
| 积分力矩贡献上限 | 4 N·m | anti-windup |
| 正弦幅值上限 | 0.80 rad | 命令行可覆盖 |
| 正弦频率上限 | 0.35 Hz | 命令行可覆盖 |
| 实测速度跳变阈值 | 2 rad/s | 超限立即退出 |
| 采样角度跳变阈值 | 0.10 rad | 超限立即退出 |
| 控制循环抖动阈值 | 10 ms | 超限立即退出 |
| LowState 超时 | 50 ms | 超限立即退出 |

### UnitreeH1DirectInterface 类结构（h1_direct）

```
UnitreeH1DirectInterface
├── init()
│   ├── initRealtimeMemory()         页面锁定 (mlockall)
│   ├── 初始化 DDS Channel
│   ├── 初始化 LowCmd（head/level_flag, pos/vel stop）
│   ├── 创建 rt/lowcmd Publisher + rt/lowstate Subscriber
│   ├── MotionSwitcher 释放高层服务
│   ├── 启动异步日志
│   └── 等待首个 LowState → controller.reset()
└── run()
    ├── setThreadRealtime()          SCHED_FIFO, priority 80
    ├── while(running):
    │   ├── readRobotState()         从 AtomicRobotCache 读取
    │   ├── checkMeasuredTrip()      测量值合理性检查（限位/跳变/抖动）
    │   ├── fillSafeHoldCommand()    兜底初始化
    │   ├── controller.step()        EID 控制器计算
    │   ├── applySafety()            安全层最后把关
    │   ├── writeLowCmd()            发布 DDS + CRC32
    │   └── pushLog()                异步日志
    └── sendSafeHold()               安全退出
```

**h1_direct 与 h1_knee_pid 的区别**：`h1_direct` 使用 EID 控制器（`eid_controller.hpp`）且参考轨迹在 YAML 中静态配置，目标关节运行正弦轨迹直到 Ctrl+C；`h1_knee_pid` 使用独立的内置 PID 控制器（非 `eid_controller.hpp`），目标角度和持续时间由命令行参数指定。

### 实时调度（SCHED_FIFO, priority 80）

Linux 默认 `SCHED_OTHER` 下任何进程都可能抢占控制线程。2ms 周期下，一次 5ms 抢占意味着丢失 2-3 个周期。`SCHED_FIFO` 将控制线程提升为实时线程。

### 内存锁定（mlockall）

防止操作系统将进程内存换出到 swap——换回延迟可达几十毫秒。

### MotionSwitcher 释放

H1 的高层运动服务和底层 LowCmd 不能同时工作。程序启动时自动释放高层服务（通过 MotionSwitcherClient）。

### 信号处理

SIGINT/SIGTERM 触发：发送最后一次 SafeHold → 关闭日志 → 退出。防止电机"断联"导致重力坠落。

---

## 16. DDS 通信

Unitree SDK2 使用 DDS 发布/订阅通信中间件。

| Topic | 方向 | 频率 | 内容 |
|-------|------|------|------|
| `rt/lowstate` | H1 → 程序 | ~500Hz | 20 关节状态 + IMU |
| `rt/lowcmd` | 程序 → H1 | ~500Hz | 20 关节命令 |

`rt/` 前缀表示实时性优先。真机通过以太网电缆传输（YAML 中配置 `network_interface`）。

### 无锁缓存

DDS 回调线程接收 LowState 后 `store()` 到 `AtomicRobotCache` 的 `std::atomic<double>` 变量；控制线程 `load()` 读取。无锁避免控制周期抖动；缓存让最新状态常驻，不需每次查 DDS。

```cpp
struct AtomicRobotCache {
    std::array<std::atomic<double>, 20> q;
    std::array<std::atomic<double>, 20> dq;
    std::array<std::atomic<double>, 20> tau_est;
    std::atomic<uint64_t> last_state_ns{0};
};
```

---

## 17. 安全层回归测试

`tests/test_safety.cpp` 覆盖四个场景：

- **测试 1 - 命令超限**：验证限幅生效，`kSafetyCommandSaturated` 置位
- **测试 2 - NaN 力矩**：验证触发 `kSafetyNonFiniteCommand`，进入 SafeHold（命令被替换为保持当前位置）
- **测试 3 - LowState 超时**：`lowstate_age=1.0s`，验证 SafeHold 覆盖有效力矩
- **测试 4 - 电机模式**：验证关节 0-11 返回 `0x0A`，关节 12 返回 `0x01`

```bash
ctest --test-dir build --output-on-failure
```

或直接运行测试二进制：
```bash
./build/h1_safety_tests.exe
```

---

## 18. 上实机前

在任何真实机器人实验前，务必逐项确认：

```text
[ ] H1 / H1-2 的 IDL 类型选择正确
[ ] target_joint 与真实关节编号一致
[ ] 20 个 motor index 映射与当前机器人型号一致
[ ] h1MotorMode() 中的 mode 与目标关节匹配
[ ] q/dq/tau 限幅与当前实验条件匹配
[ ] 机器人已悬挂、躺卧或处于可靠机械保护状态
[ ] 高层运动服务已关闭
[ ] 急停路径已验证
[ ] mock 闭环和安全测试通过
[ ] 首次实机只做 SafeHold 和极小幅单关节动作（先用 h1_knee_state 确认通信，再用 h1_knee_pid 做慢速 PID）
```

**mock 或 unitree_mujoco 通过不等于可以直接大幅度上实机。** 真实机器人仍应从悬挂、低增益、短时、小幅实验开始。
