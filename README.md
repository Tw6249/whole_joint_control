# Unitree H1 底层控制接口 — 说明书

本工程实现 Unitree H1 的 EID 单关节力矩前馈控制器，提供离线验证、安全保护、异步日志和实机 DDS 通信入口。默认实验对象为 H1 右膝（motor index 2）。

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
└── runtime_config.hpp          YAML 配置解析
src/
├── mock_closed_loop.cpp        离线闭环验证
├── main_h1_direct.cpp          实机 DDS 控制入口（需 Unitree SDK2）
├── main_h1_knee_pid.cpp        慢速单膝 PID bring-up 入口（需 Unitree SDK2）
└── subscribe_knee_state.cpp    只读订阅目标膝关节状态（需 Unitree SDK2）
tests/
└── test_safety.cpp             安全层回归测试
config/
└── h1_right_knee.yaml          默认右膝实验配置
```

---

## 2. 构建与运行

### 本地构建（Windows/MinGW）

```bash
cmake -S unitree_h1_direct_interface -B unitree_h1_direct_interface/build -G "MinGW Makefiles"
cmake --build unitree_h1_direct_interface/build -j 4
ctest --test-dir unitree_h1_direct_interface/build --output-on-failure

# 5 秒闭环验证
./build/h1_mock_closed_loop.exe config/h1_right_knee.yaml 5.0
```

mock 运行生成 `h1_mock_log.csv`，退出码 0=正常，1=异常，2=非有限值，3=安全故障。

### 实机构建（Ubuntu + Unitree SDK2）

```bash
cmake -S unitree_h1_direct_interface -B build-h1 \
  -DH1IF_BUILD_UNITREE=ON \
  -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
cmake --build build-h1 -j
./build-h1/h1_knee_state config/h1_right_knee.yaml
# 谨慎单膝 PID：target 必须靠近启动角度，且必须显式 --arm
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.35 8 --arm
# 如果默认力矩太小，可以逐步提高，但仍受程序硬上限约束
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.55 8 --arm --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15
# 正弦轨迹：第二个参数为中心角，先慢速移动到中心，再开始 sine
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.90 30 --arm --sine --amp 0.35 --freq 0.08 --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.30
sudo ./build-h1/h1_direct config/h1_right_knee.yaml
```

仿真环境（unitree_mujoco）使用 `network_interface: lo`、`domain_id: 1`。

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
                    Controller.step()   (每周期调用)
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
    virtual void reset(const RobotState& state) = 0;
    virtual void step(const RobotState& state, RobotCommand& command,
                      ControllerDebug& debug) = 0;
};
```

所有控制器实现此接口，上层代码不依赖具体控制器类型。

---

## 5. 控制器生命周期

### reset(const RobotState& state)

一次性初始化：
- 记录当前关节位置作为参考零点
- 记录起始时间
- 初始化观测器状态变量（`eta_q/dq`、`eta_lpf`、`x_hat_q/dq`）
- 重置参考轨迹规划器

### step(const RobotState& state, RobotCommand& command, ControllerDebug& debug)

每个控制周期调用一次（500Hz，2ms）：

- **入参**：当前机器人状态
- **出参**：要下发的关节命令 + 调试数据（128 个 double + 32 位 flags）

EID 控制器内部执行：参考轨迹采样 → 扰动估计 → 逆模型前馈 → PD 反馈 → 观测器更新。

---

## 6. 电机模式

`h1MotorMode(int joint_id)` 定义在 `include/safety.hpp:33-47`，根据 H1 20DOF motor index 返回控制模式。

策略：**下肢（含脚踝、腰部）使用 MIT 模式 `0x0A`，手臂使用 `0x01`**。

| motor index | H1 SDK2 名称 | mode | 说明 |
|-------------|-------------|------|------|
| 0 | RightHipRoll | `0x0A` | 右髋侧摆 |
| 1 | RightHipPitch | `0x0A` | 右髋前摆 |
| 2 | RightKnee | `0x0A` | 右膝（默认目标关节） |
| 3 | LeftHipRoll | `0x0A` | 左髋侧摆 |
| 4 | LeftHipPitch | `0x0A` | 左髋前摆 |
| 5 | LeftKnee | `0x0A` | 左膝 |
| 6 | WaistYaw | `0x0A` | 腰部偏航 |
| 7 | LeftHipYaw | `0x0A` | 左髋偏航 |
| 8 | RightHipYaw | `0x0A` | 右髋偏航 |
| 9 | NotUsedJoint | `0x0A` | SDK2 中标记为未用 |
| 10 | LeftAnkle | `0x0A` | 左脚踝 |
| 11 | RightAnkle | `0x0A` | 右脚踝 |
| 12-15 | RightShoulder/Elbow | `0x01` | 右臂 weak motor |
| 16-19 | LeftShoulder/Elbow | `0x01` | 左臂 weak motor |

### MIT 模式（0x0A）

电机实际输出力矩：

```
τ_actual = kp*(q_des - q_actual) + kd*(dq_des - dq_actual) + τ_ff
```

### 0x01 与 weak motor

手臂关节（12-19）使用位置模式 `0x01`。脚踝（10-11）虽被官方归为 weak motor，但本工程实验策略将其纳入 MIT 模式以保持整个下肢控制一致性。**真机前必须用悬挂/机械保护状态验证脚踝响应。**

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
| 状态无效 | `state_valid == false` 或含 NaN/Inf | SafeHold |
| LowState 超时 | `lowstate_age > 50ms` | SafeHold |
| 命令含非有限值 | 命令字段为 NaN/Inf | SafeHold |
| 命令超限 | q/dq/kp/kd/tau 超出 joint_limits | 限幅到边界值 |

### 命令限幅（joint_limits）

以右膝（关节 2）为例，来自 `config/h1_right_knee.yaml`：

| 参数 | 限幅范围 | 物理依据 |
|------|----------|----------|
| q | -0.26 ~ 2.05 rad | H1 URDF 膝关节行程 |
| dq | ±14.0 rad/s | H1 URDF 速度参考 |
| kp | 0 ~ 120 | 保守上限 |
| kd | 0 ~ 5 | 保守上限 |
| tau | ±80 N·m | 保守上限，低于 URDF effort (300) |

配置文件已补齐 20 个 motor index 的限幅，力矩/kp/kd 为首次实机联调的保守软件上限。

### 安全标志位

```cpp
kSafetyLowStateTimeout  = 1 << 0  // LowState 超时
kSafetyNonFiniteCommand = 1 << 1  // 命令含 NaN/Inf
kSafetyCommandSaturated = 1 << 2  // 命令被限幅
kSafetyInvalidState     = 1 << 3  // 状态无效
```

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

触发时机：程序启动、检测到安全异常、程序退出（Ctrl+C / SIGTERM）。

---

## 9. EID 控制器

`include/eid_controller.hpp` 实现单关节 EID（Equivalent Input Disturbance）力矩前馈控制器。

### 控制流程（每周期）

```
1. SmoothSineReferenceTrajectory.sample(t, dt) → ref.now, ref.next
2. 计算 x_bar = x_hat + eta（标称状态 + 扰动估计）
3. r_c_next = ref.next - eta（去除扰动后的参考）
4. analyticInverseModel() → u_star（前馈力矩）
5. r_d = ref.now + K_dagger * u_star（修正参考）
6. e = r_d - x_bar，PD 反馈：u_t = kp*e_q + kd*e_dq
7. kneeForward(x_bar, u_t, dt) 前向仿真更新 x_hat
8. 观测器更新 eta（低通滤波）
9. u_t 写入 command.joint[target_joint].tau
```

### EID 模式下的命令

EID 在 MIT 模式下以下发力矩前馈为主：

- **tau** = EID 计算的前馈力矩 u_t（主力控制信号，经过启动 ramp、限幅和 slew-rate 限制）
- **kp** = `torque_safe_kp`（0.0），**kd** = `torque_safe_kd`（默认 0.8，用作底层阻尼）
- **q** = 当前实际位置，**dq** = 0

实际电机输出力矩 = `kp*(cmd.q - actual.q) + kd*(cmd.dq - actual.dq) + tau`。当前默认 `cmd.q = actual.q`、`cmd.dq = 0`，所以 `kd` 提供速度阻尼，`tau` 提供主控制力矩。

为避免实机启动时的猛跳，EID 参考会先从当前实测角度平滑过渡到策略参考；输出力矩还会经过 `eid_tau_limit` 和 `eid_tau_slew_rate` 约束。

### 关键参数（来自 YAML）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| kp | 35.0 | PD 反馈比例增益 |
| kd | 4.0 | PD 反馈微分增益 |
| observer_gain_q | 0.25 | 位置观测器增益 |
| observer_gain_dq | 0.25 | 速度观测器增益 |
| filter_alpha | 0.08 | 扰动估计低通滤波系数 |
| startup_ramp_duration | 4.0 | 启动参考平滑过渡时间 |
| eid_tau_limit | 12.0 | EID 输出力矩限幅 |
| eid_tau_slew_rate | 60.0 | EID 输出力矩变化率限制 N·m/s |
| torque_safe_kp | 0.0 | 下发命令中的 kp |
| torque_safe_kd | 0.8 | 下发命令中的 kd 阻尼 |
| inverse_q_weight | 0.0 | 逆模型 q 通道权重（0=动态默认） |
| inverse_dq_weight | 0.0 | 逆模型 dq 通道权重（0=动态默认） |

### 被控对象模型（PlantModelConfig）

```cpp
struct PlantModelConfig {
    double Jeff = 0.238;       // 等效转动惯量 (kg·m²)
    double b = 1.0;            // 粘性阻尼系数
    double gravityA = 4.2835;  // 重力项 sin 系数
    double gravityB = 0.0;     // 重力项 cos 系数
    double tau0 = -0.2711;     // 偏置力矩 (N·m)
    double q_min = -0.26;      // 关节位置下限
    double q_max = 2.05;       // 关节位置上限
    double tau_max = 80.0;     // 最大力矩
};
```

重力补偿：`τ_gravity = gravityA * sin(q) + gravityB * cos(q)`

---

## 10. 参考轨迹

`include/reference_trajectory.hpp` 实现了 `SmoothSineReferenceTrajectory`，以**分段五次插值**生成平滑参考轨迹。

### 工作原理

1. 将时间按 `policy_reference_dt`（默认 0.05s）分段
2. 每个段的首尾节点由正弦策略计算：`q = center + amplitude * sin(2π * frequency * t)`
3. 段内用五次多项式连接，保证位置、速度、加速度连续
4. `sample(t, dt)` 返回当前和下一时刻的参考值（用于逆模型前馈）

### 默认参数

| 参数 | 值 | 含义 |
|------|-----|------|
| policy_dt | 0.05 s | 轨迹段长度 |
| center | 0.9 rad | 正弦中心 |
| amplitude | 0.08 rad | 正弦幅值 |
| frequency | 0.10 Hz | 正弦频率 |

以 0.9 rad 为中心、±0.08 rad 幅度、0.10 Hz 的小幅正弦摆动，适合首次实机 bring-up。

---

## 11. 逆模型

`analyticInverseModel()` 计算前馈力矩 `u_star`。

给定当前状态 (q, dq) 和目标增量 (Δq, Δdq)，分别从位置通道和速度通道反算所需力矩，再按权重融合：

```
τ_from_q  = bias + Jeff * ((Δq - dt*dq) / dt²)
τ_from_dq = bias + Jeff * (Δdq / dt)
u_star    = (w_q * a_q² * τ_from_q + w_dq * a_dq² * τ_from_dq) / den
```

其中 `a_q = dt²/Jeff`，`a_dq = dt/Jeff`，权重动态默认值为 `w_q = 0.5/dt²`, `w_dq = 1.0`。结果 clamp 到 `[-tau_max, tau_max]`。

`K_dagger` 修正轨迹将 `u_star` 映射为参考偏移：`r_d = ref.now + K_dagger * u_star`，其中 `K_dagger` 由 kp/kd 决定。

---

## 12. YAML 配置

`include/runtime_config.hpp` 是零外部依赖的手写 YAML 解析器（~180 行），将 YAML 映射为 C++ 结构体。

**为什么用配置文件**：调参不需重新编译；不同实验用不同配置；参数集中管理。

### 解析规则

- `#` 开头为注释
- 缩进表示层级
- `key: value` 为键值对
- 顶层无 value 的 key 是 section 标记

### 配置示例（`config/h1_right_knee.yaml`）

```yaml
robot: H1
domain_id: 0
network_interface: enp3s0
control_dt: 0.002
mock_duration: 5.0
log_path: h1_mock_log.csv

safe_hold:
  kp: 10.0
  kd: 1.0
  lowstate_timeout: 0.05

controller:
  target_joint: 2
  kp: 35.0
  kd: 4.0
  observer_gain_q: 0.25
  observer_gain_dq: 0.25
  filter_alpha: 0.08
  policy_reference_dt: 0.05
  ref_center: 0.75
  ref_amplitude: 0.35
  ref_frequency: 0.03
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

- 容量 8192 条（约 16 秒数据），满时丢弃新样本
- 使用 `std::atomic` 实现无锁 head/tail
- 控制循环 500Hz 下 `push()` 仅做一次原子操作

### CSV 输出列

```
cycle, t, dt, lowstate_age, joint_id,
q, dq, tau_est,           ← 测量值
q_cmd, dq_cmd, kp_cmd, kd_cmd, tau_cmd,  ← 命令值
flags,                     ← 安全标志位
debug_0 ~ debug_15         ← 控制器内部变量
```

---

## 14. 离线闭环验证

`src/mock_closed_loop.cpp` 在无需真机的情况下验证完整控制链路。

### 流程

1. 加载 YAML 配置
2. 初始化虚拟 Plant（使用 PlantModelConfig 参数）
3. 循环（500Hz，默认 5 秒）：
   - Plant 状态 → RobotState
   - `controller.step()` → 命令
   - `applySafety()` → 安全检查
   - `stepPlant()` → 前向仿真（半隐式欧拉 + 关节限位）
   - 推送日志样本
4. 输出汇总：位置 RMSE、最大力矩、安全标志、日志丢弃数
5. 生成 `h1_mock_log.csv`

退出码：0=正常，1=异常，2=非有限值，3=安全故障。

### 与实机的差异

| 方面 | mock_closed_loop | main_h1_direct（实机） |
|------|------------------|------------------------|
| 关节动力学 | 简化单摆模型 | 真实物理关节 |
| 传感器 | 模型计算 | DDS LowState |
| 命令执行 | stepPlant() | DDS LowCmd → 电机 |
| 延迟/噪声/摩擦 | 无 | 真实 |
| 实时调度 | 无 | SCHED_FIFO + mlockall |

**mock 通过是必要条件，不是充分条件。**

---

## 15. 实机运行

`src/main_h1_direct.cpp` 和 `src/subscribe_knee_state.cpp` 仅在 `-DH1IF_BUILD_UNITREE=ON` 时编译。

### 慢速单膝 PID bring-up

`h1_knee_pid` 是单独的保守实机入口，用于小步验证右膝闭环。它只控制 YAML 中的 `controller.target_joint`，默认 `2`（RightKnee）。目标角必须显式给出并带 `--arm`：

```bash
sudo ./build-h1/h1_knee_pid config/h1_right_knee.yaml 0.35 8 --arm
```

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

也可以在运行终端按 `Ctrl+C`。程序收到 SIGINT/SIGTERM 后会退出 PID，并继续发送约 1 秒的低刚度 hold/damping 命令。

内置 bring-up 限制：

- 目标角必须在 YAML 关节限位内，并距离上下限至少 `0.03 rad`
- 目标角距离启动测得角度不得超过 `1.50 rad`
- 默认参考速度 `0.10 rad/s`、默认力矩限幅 `6 N m`
- 命令行最大允许参考速度 `0.80 rad/s`、力矩限幅 `18 N m`、`kp <= 40`、`ki <= 5`、`kd <= 5`
- 积分项带 anti-windup，积分力矩贡献最多 `4 N m`
- 正弦幅值最大 `0.80 rad`、频率最大 `0.35 Hz`，完整正弦范围必须在 soft limit 内
- 实测角度越过 YAML 限位、速度超过 `2 rad/s`、采样跳变、LowState 超时或控制循环严重抖动时立即退出 PID

### 只读检查目标膝关节状态

在发送任何控制命令之前，先运行只读订阅器确认 DDS、网卡、domain 和目标关节编号正确：

```bash
./build-h1/h1_knee_state config/h1_right_knee.yaml
```

该程序只订阅 `rt/lowstate`，不会发布 `rt/lowcmd`。它读取 YAML 中的 `controller.target_joint`，默认是 `2`（RightKnee），并以 CSV 行打印：

```text
t_s,joint_id,q_rad,dq_rad_s,tau_est_nm,lowstate_age_s
```

### UnitreeH1DirectInterface 类结构

```
UnitreeH1DirectInterface
├── init()
│   ├── initRealtimeMemory()     页面锁定
│   ├── 初始化 DDS Channel
│   ├── 订阅 rt/lowstate，发布 rt/lowcmd
│   ├── MotionSwitcher 释放高层服务
│   └── 等待首个 LowState
└── run()
    ├── setThreadRealtime()      SCHED_FIFO, priority 80
    ├── while(running):
    │   ├── readRobotState()     从 AtomicRobotCache 读取
    │   ├── fillSafeHoldCommand() 兜底初始化
    │   ├── controller.step()
    │   ├── applySafety()
    │   ├── writeLowCmd()        发布 DDS + CRC32
    │   └── pushLog()
    └── sendSafeHold()           安全退出
```

### 实时调度（SCHED_FIFO, priority 80）

Linux 默认 `SCHED_OTHER` 下任何进程都可能抢占控制线程。2ms 周期下，一次 5ms 抢占意味着丢失 2-3 个周期。`SCHED_FIFO` 将控制线程提升为实时线程。

### 内存锁定（mlockall）

防止操作系统将进程内存换出到 swap——换回延迟可达几十毫秒。

### MotionSwitcher 释放

H1 的高层运动服务和底层 LowCmd 不能同时工作。程序启动时自动释放高层服务。

### 信号处理

SIGINT/SIGTERM 触发：发送最后一次 SafeHold → 关闭日志 → 退出。防止电机"断联"导致重力坠落。

---

## 16. DDS 通信

Unitree SDK2 使用 DDS 发布/订阅通信中间件。

| Topic | 方向 | 频率 | 内容 |
|-------|------|------|------|
| `rt/lowstate` | H1 → 程序 | ~500Hz | 20 关节状态 + IMU |
| `rt/lowcmd` | 程序 → H1 | ~500Hz | 20 关节命令 |

`rt/` 前缀表示实时性优先。真机通过以太网电缆传输（`enp3s0`）。

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
- **测试 2 - NaN 力矩**：验证触发 `kSafetyNonFiniteCommand`，进入 SafeHold
- **测试 3 - LowState 超时**：`lowstate_age=1.0s`，验证 SafeHold 覆盖有效力矩
- **测试 4 - 电机模式**：验证关节 0-11 返回 `0x0A`，关节 12 返回 `0x01`

```bash
ctest --test-dir build --output-on-failure
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
[ ] 首次实机只做 SafeHold 和极小幅单关节动作
```

**mock 或 unitree_mujoco 通过不等于可以直接大幅度上实机。** 真实机器人仍应从悬挂、低增益、短时、小幅实验开始。
