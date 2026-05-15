# Unitree H1 Full-Body EID Control

这个仓库用于在 Unitree H1 上开发和验证多关节 EID 控制器。当前推荐先在本地 mock 和 MuJoCo 里验证控制效果，再把确认过的配置逐步搬到实机模板里。

## 最常用命令

### 1. 构建控制器和测试程序

如果 `build-h1` 已经配置过，直接构建：

```powershell
cmake --build build-h1 --config Debug --target h1_eid_stepper h1_safety_tests h1_mock_closed_loop
ctest --test-dir build-h1 -C Debug --output-on-failure
```

如果第一次在这台机器上使用，先配置工程：

```powershell
cmake -S . -B build-h1
```

然后再运行上面的构建命令。

### 2. 跑 MuJoCo 闭环、视频和图表

这是现在最推荐看的命令：

```powershell
python scripts\run_lower_body_mujoco_test.py --config config\h1_full_body_mujoco_fit.yaml --duration 15.0 --dt 0.002 --fps 24
```

这条命令做三件事：

1. 读取 `config\h1_full_body_mujoco_fit.yaml`，里面包含每个关节是否启用、参考轨迹、EID 增益、力矩限制和 MuJoCo 拟合出来的简化动力学参数。
2. 调用 `scripts\render_mujoco_eid_closed_loop.py`。这个脚本会启动 `h1_eid_stepper`，让 C++ 的 `EidMultiJointController` 每 `0.002 s` 根据 MuJoCo 当前关节状态输出一次力矩命令，然后把力矩送回 MuJoCo 推进一步。
3. 调用 `scripts\plot_mujoco_lower_body_results.py`，根据闭环日志生成每个关节的跟踪误差、力矩曲线和汇总 CSV。

重要参数含义：

```text
--config    使用哪份控制器配置
--duration  仿真总时长，单位秒；15.0 表示跑 15 秒
--dt        控制和 MuJoCo 步进周期；0.002 表示 500 Hz
--fps       输出视频帧率；24 表示每秒 24 帧
```

它不是在跑实机，也不走 Unitree SDK/DDS。这里的链路是：

```text
MuJoCo 关节状态 -> h1_eid_stepper -> C++ EID 控制器 -> 力矩命令 -> MuJoCo
```

浮动基座在空中固定住，主要目的是检查“关节参考轨迹能不能被控制器跟上、力矩有没有顶到限制、有没有安全 flags”。

输出目录默认长这样：

```text
data/mujoco_fit/runs/YYYYMMDD_HHMMSS_h1_full_body_mujoco_fit/
```

主要结果文件：

```text
eid_mujoco_closed_loop.gif 或 eid_mujoco_closed_loop.mp4  # 闭环运动视频
eid_mujoco_closed_loop_log.csv                           # 每个关节的 q、dq、q_ref、误差、力矩日志
input_config.yaml                                        # 本次运行使用的配置副本
run_manifest.txt                                         # 本次运行的参数记录
h1_eid_summary.csv                                       # 每个关节的 RMSE、最大误差、最大力矩、warning
h1_eid_tracking_grid.png                                 # q_ref 和实际 q 对比图
h1_eid_error_grid.png                                    # 跟踪误差图
h1_eid_torque_grid.png                                   # 力矩命令图
```

最近一次结果目录会写到：

```text
data/mujoco_fit/LATEST.txt
```

默认会删除中间 PNG 帧，只保留视频。需要调试逐帧渲染时加：

```powershell
python scripts\run_lower_body_mujoco_test.py --config config\h1_full_body_mujoco_fit.yaml --duration 15.0 --dt 0.002 --fps 24 --keep-frames
```

### 3. 跑简化 mock 闭环

```powershell
.\build-h1\Debug\h1_mock_closed_loop.exe config\h1_full_body_mujoco_fit.yaml 5.0
```

这个命令不启动 MuJoCo，也不渲染视频。它只用 YAML 里的简化单关节模型：

```text
tau = Jeff*qacc + b*dq + gravityA*sin(q) + gravityB*cos(q) + tau0
```

来快速跑一遍 C++ EID 控制器。第二个参数 `5.0` 是 mock 仿真时长，单位秒。日志路径由配置里的 `log_path` 决定，当前是 `data/h1_mock_log.csv`。

## 重新拟合 MuJoCo EID 参数

```powershell
python scripts\fit_mujoco_eid_params.py
```

这个脚本会读取 `h1_official_mujoco/h1.xml`，用 MuJoCo `mj_inverse` 对每个关节做局部单关节拟合，并生成或更新：

```text
config/h1_full_body_mujoco_fit.yaml
config/h1_full_body_real_template.yaml
```

拟合出来的参数适合当 mock/MuJoCo 的初值，不是实机最终辨识结果。实机仍然需要逐关节、小幅度、低增益验证方向、零位、摩擦、阻尼和力矩上限。

## 当前配置

当前默认面向 H1 的 19 个可用执行器：

```text
0-8, 10-19
```

关节 `9` 是官方 MJCF 里的 `not_use_joint` 占位关节，不应该进入控制。

核心配置文件：

```text
config/h1_full_body_mujoco_fit.yaml     # mock / MuJoCo 使用，19 个可用关节默认启用
config/h1_full_body_real_template.yaml  # 实机模板，腰和手臂默认更保守
```

`eid_controllers.<joint_id>.enabled` 控制某个关节是否进入 EID：

```text
enabled: true   该关节进入参考轨迹、EID 正逆模型和力矩命令
enabled: false  该关节不进入 EID，但仍用 SafeHold 保持当前测得位置
```

也就是说，`enabled: false` 不是完全断控，而是退出 EID 后进入安全保持。如果要让关节完全松开，需要另外实现 passive/disable 模式。

## 控制器和入口

核心代码：

```text
include/controller_interface.hpp    RobotState / RobotCommand / IController
include/eid_controller.hpp          多关节 EID 控制器
include/runtime_config.hpp          YAML 配置解析与校验
include/safety.hpp                  H1 电机模式、限幅、SafeHold、安全检查
include/reference_trajectory.hpp    参考轨迹
src/mock_closed_loop.cpp            本地 mock 闭环
src/eid_stepper.cpp                 MuJoCo 调用的 C++ 控制器 stepper
src/main_h1_direct.cpp              实机 DDS EID 入口
src/main_h1_knee_pid.cpp            实机保守 PID bring-up
src/subscribe_knee_state.cpp        实机只读状态检查
tests/test_safety.cpp               安全层和配置回归测试
```

`h1_direct` 和 `render_mujoco_eid_closed_loop.py` 使用同一个 C++ `EidMultiJointController`。区别是 MuJoCo 渲染不走 SDK/DDS，实机入口仍然走 Unitree SDK2 DDS。

## 实机运行说明

实机相关程序只在 Ubuntu + Unitree SDK2 环境下构建和运行。本地 Windows/MuJoCo 测试通过之后，再进入这一部分。

先看清三个入口的区别：

```text
h1_knee_state  只读订阅 rt/lowstate，不发布 rt/lowcmd，不会动机器人
h1_knee_pid    发布 rt/lowcmd，只控制一个 primary EID 关节，用来做低风险 bring-up
h1_direct      发布 rt/lowcmd，按 enabled 列表运行多关节 EID 控制器
```

`primary EID 关节` 的选择规则是：如果 `eid_controllers.2.enabled: true`，优先用右膝 joint 2；否则使用第一个 enabled 的关节。所以做单关节 PID bring-up 时，建议先在 `config/h1_full_body_real_template.yaml` 里只打开你要测试的那个关节。

### 1. 实机配置先改这里

实机建议从 `config/h1_full_body_real_template.yaml` 开始，不要直接拿 MuJoCo 配置上机。

需要重点确认：

```yaml
domain_id: 0
network_interface: enp3s0   # 按实机电脑网卡名修改
control_dt: 0.002
```

如果是 `unitree_mujoco` 仿真 DDS，一般是：

```yaml
domain_id: 1
network_interface: lo
```

第一次上实机时，建议只保留一个关节：

```yaml
eid_controllers:
  2:
    name: RightKnee
    enabled: true
    ref_amplitude: 0.05
    eid_tau_limit: 4
```

其他关节先设成：

```yaml
enabled: false
```

注意：这里的 `enabled: false` 不是完全断电松开，而是不进入 EID；程序仍会用 SafeHold 对该关节发低增益保持命令。

### 2. 构建实机目标

```bash
cmake -S . -B build-h1 \
  -DH1IF_BUILD_UNITREE=ON \
  -DCMAKE_PREFIX_PATH=/opt/unitree_robotics

cmake --build build-h1 -j
```

会生成的主要实机程序：

```text
h1_knee_state  只读状态检查
h1_knee_pid    保守单关节 PID bring-up
h1_direct      多关节 EID 实机入口
```

### 3. 先做只读检查

```bash
./build-h1/h1_knee_state config/h1_full_body_real_template.yaml
```

这个程序只订阅 `rt/lowstate`。它不会创建 `rt/lowcmd` publisher，所以不会给机器人发控制命令。

它会打印类似：

```text
t_s,joint_id,q_rad,dq_rad_s,tau_est_nm,lowstate_age_s
```

你要确认：

```text
joint_id 是你准备测试的关节
q_rad 和实际姿态方向一致
dq_rad_s 静止时接近 0
lowstate_age_s 很小，没有持续超时
```

如果这里关节编号、方向、网卡、domain 不对，不要继续。

### 4. 再做单关节 PID bring-up

```bash
sudo ./build-h1/h1_knee_pid config/h1_full_body_real_template.yaml 0.35 8 --arm
```

这条命令的意思是：

```text
config/h1_full_body_real_template.yaml  使用这份实机配置
0.35                                    目标关节位置，单位 rad
8                                       最多运行 8 秒
--arm                                   明确确认要发控制命令；没有它程序拒绝运行
```

`h1_knee_pid` 会先读取当前关节位置，再用速度限制慢慢把参考位置推向目标值。它只给 primary EID 关节输出 PID 力矩，其他关节走 SafeHold。

更完整的命令格式：

```bash
sudo ./build-h1/h1_knee_pid <config.yaml> <target_q_rad> <run_seconds> --arm \
  [--kp K] [--ki K] [--kd K] [--tau-limit N_M] [--speed RAD_S]
```

例子：

```bash
sudo ./build-h1/h1_knee_pid config/h1_full_body_real_template.yaml 0.55 8 --arm --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15
```

也支持小幅正弦：

```bash
sudo ./build-h1/h1_knee_pid config/h1_full_body_real_template.yaml 0.90 30 --arm --sine --amp 0.35 --freq 0.08 --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.30
```

这个 bring-up 程序内置硬限制：

```text
目标角必须在 YAML 关节限位内，并且离边界至少 0.03 rad
目标角离启动角不能超过 1.50 rad
run_seconds 必须在 (0, 120] 秒
参考速度 <= 0.80 rad/s
力矩限制 <= 18 N m
kp <= 40, ki <= 5, kd <= 5
正弦幅值 <= 0.80 rad，频率 <= 0.35 Hz
实测速度超过 2 rad/s、角度跳变过大、LowState 超时或控制周期超过 10 ms 会停机
```

触发停机或 Ctrl+C 后，程序会继续发送约 1 秒低增益 hold 命令。

### 5. 最后再运行实机 EID

```bash
sudo ./build-h1/h1_direct config/h1_full_body_real_template.yaml
```

`h1_direct` 会：

```text
读取 YAML 中所有 enabled: true 的关节
订阅 rt/lowstate
发布 rt/lowcmd
启动 C++ EidMultiJointController
每 control_dt 秒生成一次命令
对所有命令再跑 safety.hpp 的限幅和 SafeHold 保护
把日志写到 log_path 对应的时间戳目录
```

启动时程序会打印安全警告并等待你按 Enter。这是最后一道人工确认，不要在机器人没有悬挂、躺稳或机械保护时按 Enter。

`h1_direct` 的运行边界：

```text
只控制 enabled: true 的关节
active 关节出现 LowState 超时、非有限值、角度越界、速度超过 8 rad/s、角度跳变超过 0.10 rad 或控制周期超过 10 ms，会切 SafeHold 并退出控制循环
Ctrl+C / SIGTERM 会发送 SafeHold 后退出
```

### 6. 上机前检查单

```text
[ ] 急停可用，旁边有人看护
[ ] 机器人悬挂、躺稳，或有可靠机械保护
[ ] network_interface 和 domain_id 与当前环境一致
[ ] h1_knee_state 只读数据正常
[ ] joint_id 和真实关节对应关系确认过
[ ] q 正方向、零位、限位和实际机器人一致
[ ] 第一次只打开一个 enabled 关节
[ ] ref_amplitude、kp/kd、eid_tau_limit 都从小值开始
[ ] MuJoCo 通过不代表实机可直接大幅度运行
```

## 推荐调参流程

1. 修改 `config/h1_full_body_mujoco_fit.yaml` 中的参考轨迹、增益或力矩限制。
2. 运行 MuJoCo 闭环命令。
3. 看 `h1_eid_summary.csv`、tracking/error/torque 图和视频。
4. 如果某个关节误差大或力矩顶到限制，先在 MuJoCo 配置里缩小幅度、降低增益或调整力矩限制。
5. 准备实机时，把需要控制的关节逐个在 `config/h1_full_body_real_template.yaml` 中改成 `enabled: true`，再用 `h1_knee_state` 和 `h1_knee_pid` 做保守检查。
