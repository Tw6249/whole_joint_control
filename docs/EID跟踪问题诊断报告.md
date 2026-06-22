# EID 跟踪问题诊断报告

生成时间：2026-06-22
对象：H1 hip/knee MuJoCo 闭环实验
基准配置：`config/h1_hip_knee_dual_tuned.yaml`
主要实验输出：

- `analysis_artifacts/eid_tracking_diagnostics/`
- `analysis_artifacts/eid_tracking_diagnostics_alpha090/`
- `analysis_artifacts/eid_tracking_counterfactual/`

## 1. 问题概述

当前现象是：在 MuJoCo 闭环实验中，EID 控制相对 PD 基线通常产生更大的跟踪误差。该问题在无扰动场景中也存在，因此不能简单解释为扰动估计不足或外部扰动补偿方向错误。

本次诊断重点回答三个问题：

1. EID 误差变大是否由 `filter_alpha` 过小造成？
2. EID 误差变大是否由 `Ku` 的符号、尺度或扰动补偿注入造成？
3. 若以上两者都不是主因，真正导致误差放大的代码路径是什么？

最终结论：

> 当前 EID 大误差的主因是 `u_star` 这条逆模型前馈路径，以及它进入 EID 闭环的方式。`analyticInverseModel / forwardModel` 使用的逐关节简化模型与当前 MuJoCo 全身 plant 不充分一致，导致 `u_star` 在无扰动下也形成较大的等效参考偏移，从而把原本较好的 PD 跟踪拉偏。

## 2. 当前控制结构

### 2.1 PD 基线

`position_pd` 控制器在 `include/position_pd_controller.hpp` 中将参考位置、参考速度、`kp`、`kd` 直接写入电机命令：

```cpp
c.q = ref.now.q;
c.dq = ref.now.dq;
c.kp = cfg_.controller.kp;
c.kd = cfg_.controller.kd;
c.tau = 0.0f;
```

MuJoCo 侧最终将命令统一转成力矩：

```python
tau_controller =
    kp * (q_cmd - q)
  + kd * (dq_cmd - dq)
  + tau
```

因此 PD 实验的实际结构是：

```text
tau_PD = kp * (q_ref - q) + kd * (dq_ref - dq)
```

### 2.2 EID 控制器

EID 控制器在 `include/eid_controller.hpp` 中先计算：

```cpp
eta_u = ku_q * eta_q + ku_dq * eta_dq;
u_star_comp = inv.u_star - eta_u;
```

随后将 `u_star_comp` 等效折算进修正参考：

```cpp
r_d_q  = ref.now.q  + w_q  * u_star_comp;
r_d_dq = ref.now.dq + w_dq * u_star_comp;
u_raw  = kp * (r_d_q - x_bar_q) + kd * (r_d_dq - x_bar_dq);
```

代数上等价于：

```text
u_raw = u_star - eta_u + kp * (ref_q - x_bar_q) + kd * (ref_dq - x_bar_dq)
```

同时 EID 写出的电机命令是 torque mode：

```cpp
c.q = q;
c.dq = 0.0f;
c.kp = torque_safe_kp;
c.kd = torque_safe_kd;
c.tau = result.u_t;
```

当前配置里 `torque_safe_kp = 0.0`，`torque_safe_kd = 0.0`，因此 EID 与 PD 并不是完全同一种闭环形态：

- PD：使用电机位置伺服形式，`q_cmd/dq_cmd/kp/kd` 在 MuJoCo 侧形成力矩。
- EID：内部自行计算 torque，外部 `kp/kd` 基本为 0。

这个差异本身不是错误，但意味着若 EID 内部模型不一致，误差会直接进入力矩通道。

## 3. 实验设计

### 3.1 关节和轨迹

本轮诊断只启用两个关节：

| joint id | 名称 | 角色 |
|---:|---|---|
| 1 | RightHipPitch | hip |
| 2 | RightKnee | knee |

无扰动场景 `S0_no_disturbance`：

| 关节 | center | amplitude | frequency | phase |
|---|---:|---:|---:|---:|
| hip | -0.30 | 0.10 | 0.25 Hz | -pi/2 |
| knee | 0.75 | 0.08 | 0.25 Hz | -pi/2 |

扰动场景 `S1_hip_pos` 和 `S3_knee_pos` 使用 knee 反相轨迹：

| 关节 | center | amplitude | frequency | phase |
|---|---:|---:|---:|---:|
| hip | -0.30 | 0.10 | 0.25 Hz | -pi/2 |
| knee | 0.75 | 0.08 | 0.25 Hz | +pi/2 |

### 3.2 主要 EID 参数

`alpha=0.90` 复现实验中，hip/knee 的 `filter_alpha` 均被强制覆盖为 `0.90`。

| 关节 | kp | kd | filter_alpha | Ko_q | Ko_dq | Ku_q | Ku_dq | tau_limit |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| RightHipPitch | 120.0 | 3.0 | 0.90 | 0.8 | 0.2 | 6.0 | 0.6 | 200 |
| RightKnee | 70.0 | 4.0 | 0.90 | 0.6 | 0.2 | 3.5 | 0.8 | 300 |

### 3.3 扰动设置

| 场景 | 扰动关节 | 扰动扭矩 | 时间窗 | 波形 |
|---|---|---:|---|---|
| S1_hip_pos | RightHipPitch | +12 Nm | 1.5 s 到 2.0 s | smooth_rect, ramp 0.08 s |
| S3_knee_pos | RightKnee | +10 Nm | 1.5 s 到 2.0 s | smooth_rect, ramp 0.08 s |

### 3.4 方法组

| 方法 | 含义 |
|---|---|
| `pd` | position PD 基线 |
| `eid_ku0` | EID observer 开启，但 `Ku=0` |
| `eid_full` | 完整 EID |
| `eid_ku_half` | `Ku` 缩小到 0.5 倍 |
| `eid_ku_neg` | `Ku` 取负 |
| `eid_q_only` | 仅位置 EID 通道 |
| `eid_dq_only` | 仅速度 EID 通道 |
| `eid_alpha_low/high` | alpha 变体，用于确认滤波影响 |

## 4. 实验结果

### 4.1 无扰动下 alpha=0.90 结果

`analysis_artifacts/eid_tracking_diagnostics_alpha090/eid_tracking_diagnostics_metrics.csv`

| 方法 | hip RMSE | knee RMSE | coord RMSE | tau RMS | tau rate RMS | hip eta_u RMS | knee eta_u RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| PD | 0.001444 | 0.001550 | 0.000456 | 0.206047 | 0.609982 | 0.000000 | 0.000000 |
| EID Ku=0 | 0.019449 | 0.022042 | 0.038584 | 0.493929 | 12.7052 | 0.000000 | 0.000000 |
| EID full | 0.019416 | 0.021903 | 0.038419 | 0.494930 | 40.3238 | 0.053772 | 0.053235 |
| EID Ku 0.5x | 0.019432 | 0.021973 | 0.038502 | 0.492949 | 22.9725 | 0.026886 | 0.026617 |
| EID Ku -1x | 0.019482 | 0.022181 | 0.038750 | 0.504653 | 40.3070 | 0.053771 | 0.053235 |
| EID q-only | 0.076942 | 0.124547 | 0.198162 | 0.560550 | 40.2736 | 0.060049 | 0.053875 |
| EID dq-only | 0.073613 | 0.161993 | 0.233724 | 0.514396 | 12.4963 | 0.003008 | 0.008615 |

关键观察：

1. `Ku=0` 与 `EID full` 的误差几乎相同。
2. `EID full` 相对 PD 的误差大约高一个数量级。
3. `eta_u RMS` 只有约 `0.05 Nm`，但误差已经很大。
4. 单独 q-only 或 dq-only 都更差，说明不是简单关掉某一路就能修复。

结论：

> 无扰动下的主要误差不是 `eta_u` 注入力矩造成的。即使 `Ku=0`，EID 仍然明显偏离 PD。

### 4.2 alpha 从低值改为 0.90 的影响

旧实验中，`eid_alpha_low = 0.005` 曾造成严重恶化：

| 方法 | coord RMSE | tau rate RMS | hip eta_u RMS | knee eta_u RMS |
|---|---:|---:|---:|---:|
| EID full, 原 alpha | 0.038502 | 44.6995 | 0.053858 | 0.053313 |
| EID alpha low=0.005 | 1.17386 | 5410.92 | 0.656553 | 0.362476 |
| EID alpha high=0.08 | 0.038426 | 40.6704 | 0.053779 | 0.053249 |

alpha 统一设为 `0.90` 后：

| 方法 | coord RMSE | tau rate RMS | hip eta_u RMS | knee eta_u RMS |
|---|---:|---:|---:|---:|
| EID full | 0.038419 | 40.3238 | 0.053772 | 0.053235 |
| EID alpha low 分支，实际 alpha=0.90 | 0.038419 | 40.3238 | 0.053772 | 0.053235 |
| EID alpha high 分支，实际 alpha=0.90 | 0.038419 | 40.3238 | 0.053772 | 0.053235 |

结论：

> 过小 alpha 确实会严重恶化 EID，但把 alpha 统一改成 0.90 后，只消除了低 alpha 的灾难性滞后，并没有解决 EID 相对 PD 的核心误差。

### 4.3 扰动窗口结果

#### Hip 正扰动 S1

| 方法 | hip RMSE during | knee RMSE during | coord RMSE during | S_coord vs PD | cross ratio | S_cross vs PD |
|---|---:|---:|---:|---:|---:|---:|
| PD | 0.096584 | 0.011151 | 0.092405 | 1.000 | 0.115458 | 1.000 |
| EID full | 0.094626 | 0.022686 | 0.075881 | 0.821 | 0.239745 | 2.076 |
| EID Ku 0.5x | 0.094687 | 0.022755 | 0.075888 | 0.821 | 0.240319 | 2.081 |
| EID Ku -1x | 0.094869 | 0.022962 | 0.075910 | 0.821 | 0.242039 | 2.096 |
| EID q-only | 0.110967 | 0.105157 | 0.086064 | 0.931 | 0.947649 | 8.208 |
| EID dq-only | 0.082922 | 0.126742 | 0.084788 | 0.918 | 1.52845 | 13.238 |

观察：

- EID full 对 hip 正扰动的协调误差有一定改善。
- 但非扰动 knee 的误差明显增大，串扰比约为 PD 的 2.08 倍。

#### Knee 正扰动 S3

| 方法 | hip RMSE during | knee RMSE during | coord RMSE during | S_coord vs PD | cross ratio | S_cross vs PD |
|---|---:|---:|---:|---:|---:|---:|
| PD | 0.009338 | 0.122688 | 0.119519 | 1.000 | 0.076113 | 1.000 |
| EID full | 0.012416 | 0.157636 | 0.159615 | 1.335 | 0.078763 | 1.035 |
| EID Ku 0.5x | 0.012448 | 0.158097 | 0.160070 | 1.339 | 0.078735 | 1.034 |
| EID Ku -1x | 0.012543 | 0.159481 | 0.161436 | 1.351 | 0.078652 | 1.033 |
| EID q-only | 0.079017 | 0.529991 | 0.593476 | 4.966 | 0.149090 | 1.959 |
| EID dq-only | 0.060441 | 0.344992 | 0.399611 | 3.343 | 0.175197 | 2.302 |

观察：

- Knee 正扰动下，EID full 明显劣于 PD。
- 这说明当前逐关节对角式 EID 对 hip-knee 耦合和方向不对称问题处理不足。

## 5. 关键反事实实验：关闭 u_star

为了定位主因，构造一个反事实实验：

```yaml
inverse_q_weight: 1.0e-12
inverse_dq_weight: 1.0e-12
```

由于 `analyticInverseModel()` 中 `den < 1.0e-12` 时 `u_star` 保持为 0，这可以近似得到：

```text
u_star ≈ 0
```

注意：这不是最终建议的长期配置，只是用于定位因果路径。

### 5.1 无扰动反事实

| 方法 | hip RMSE | knee RMSE | hip mean error | knee mean error | hip tau RMS | knee tau RMS |
|---|---:|---:|---:|---:|---:|---:|
| PD | 0.001444 | 0.001550 | -0.000057 | -0.000018 | 0.174414 | 0.109703 |
| EID full | 0.019416 | 0.021903 | 0.016674 | -0.021229 | 0.463375 | 0.173893 |
| EID, u_star≈0 | 0.001729 | 0.003537 | 0.000444 | -0.002896 | 0.185027 | 0.123940 |

整体 `q_rmse`：

| 方法 | q_rmse |
|---|---:|
| PD | 0.001498 |
| EID full | 0.020697 |
| EID, u_star≈0 | 0.002783 |

结论：

> 只关闭 `u_star` 后，EID 无扰动误差从约 `0.0207` 下降到约 `0.00278`，已经接近 PD。这是最强证据，说明主误差来自 `u_star` / 逆模型前馈路径。

### 5.2 扰动场景反事实

#### Hip 正扰动

| 方法 | hip RMSE during | knee RMSE during | coord RMSE during |
|---|---:|---:|---:|
| PD | 0.096584 | 0.011151 | 0.092405 |
| EID full | 0.094626 | 0.022686 | 0.075881 |
| EID, u_star≈0 | 0.099529 | 0.013738 | 0.092806 |

#### Knee 正扰动

| 方法 | hip RMSE during | knee RMSE during | coord RMSE during |
|---|---:|---:|---:|
| PD | 0.009338 | 0.122688 | 0.119519 |
| EID full | 0.012416 | 0.157636 | 0.159615 |
| EID, u_star≈0 | 0.010952 | 0.144937 | 0.141439 |

结论：

- 关闭 `u_star` 后，knee 扰动场景明显好于原 EID full，但仍差于 PD。
- hip 扰动场景中，EID full 的协调误差改善一部分来自当前 `u_star` 结构，但也伴随更强 cross transfer。
- 这说明 `u_star` 是主灾源；扰动估计和逐关节补偿本身仍有方向性、耦合性问题。

## 6. 为什么不是 Ku 符号或尺度问题

如果主要问题是 `Ku` 符号反了或尺度过大，那么应该看到：

```text
Ku=0 明显优于 EID full
Ku=-1 明显改变误差方向或显著改善
Ku=0.5 明显缓解过补偿
```

实际结果：

| 方法 | 无扰动 coord RMSE |
|---|---:|
| EID Ku=0 | 0.038584 |
| EID full | 0.038419 |
| EID Ku 0.5x | 0.038502 |
| EID Ku -1x | 0.038750 |

四者几乎相同。

因此：

> `Ku` 不是无扰动大误差的主因。`eta_u` 的方向和尺度需要继续验证，但它不是当前最大的错误来源。

## 7. 为什么不是参考轨迹生成问题

检查 `q_ref` 的数值微分与 `dq_ref` 的一致性：

| 场景 | 关节 | dq_ref RMS | finite difference - dq_ref RMS |
|---|---|---:|---:|
| S0 PD | hip | 0.111082 | 0.00000766 |
| S0 PD | knee | 0.088865 | 0.00000613 |
| S0 EID Ku=0 | hip | 0.111082 | 0.00000766 |
| S0 EID Ku=0 | knee | 0.088865 | 0.00000613 |

参考位置和参考速度在数值上是一致的，因此当前主问题不是 `q_ref / dq_ref` 本身不连续或生成错误。

## 8. u_star 路径的具体问题

### 8.1 u_star 在无扰动下过大

无扰动 `S0_no_disturbance__eid_ku0` 中：

| 关节 | u_star RMS | u_feedback RMS | u_star + u_feedback RMS | u_star 与 feedback 相关性 |
|---|---:|---:|---:|---:|
| hip | 2.200 | 2.278 | 0.463 | -0.922 |
| knee | 1.311 | 1.322 | 0.171 | -0.844 |

这表示：

- `u_star` 很大。
- feedback 项大幅反向抵消 `u_star`。
- 最终力矩不算极端，但闭环工作点被 `u_star` 等效参考偏移拉偏。

### 8.2 单关节模型与 MuJoCo plant 不充分一致

EID 的 `analyticInverseModel()` 和 `forwardModel()` 使用：

```cpp
qacc = (tau - b * dq - gravityTorque(q) - tau0) / Jeff;
```

这是逐关节、对角、简化模型。当前 MuJoCo 是全身多体模型，并且 hip/knee 的真实响应包含：

- 浮动基座影响；
- 多关节耦合；
- MuJoCo 关节 armature/damping；
- 被固定/未控自由度的动力学影响；
- 与逐关节拟合模型不完全一致的惯量、重力和阻尼。

用日志中的实际 `q,dq,tau` 代入简化模型反推加速度，和 MuJoCo 实际加速度 RMS 差异很大：

| 运行 | 关节 | 实际 ddq RMS | 简化模型预测 ddq RMS | 预测误差 RMS |
|---|---|---:|---:|---:|
| S0 PD | hip | 0.177 | 2.315 | 2.230 |
| S0 PD | knee | 0.140 | 5.303 | 5.269 |
| S0 EID Ku=0 | hip | 0.961 | 2.651 | 2.573 |
| S0 EID Ku=0 | knee | 0.976 | 5.718 | 5.702 |

这说明：

> 当前用于 EID 的 `Jeff/b/gravity` 单关节模型不能准确描述 MuJoCo 闭环 plant。由它产生的 `u_star` 和预测状态会污染 EID 控制律。

### 8.3 当前逆模型的精确形式

当前 `analyticInverseModel()` 并不是 MuJoCo 全身逆动力学，而是每个关节各自使用一个标量二阶模型：

```text
tau = Jeff * qdd + b * dq + gravityA * sin(q) + gravityB * cos(q) + tau0
```

其中：

```text
bias(q, dq) = b * dq + gravityA * sin(q) + gravityB * cos(q) + tau0
```

然后根据当前参考和下一步参考构造两个候选力矩：

```text
tau_from_q  = bias + Jeff * (q_next - q - dt * dq) / dt^2
tau_from_dq = bias + Jeff * (dq_next - dq) / dt
```

默认配置里：

```yaml
inverse_q_weight: 0.0
inverse_dq_weight: 0.0
```

代码会把它们替换为：

```text
q_weight  = 0.5 / dt^2
dq_weight = 1.0
```

在当前 `dt = 0.002 s` 下，最终 `u_star` 近似为：

```text
u_star ≈ 1/3 * tau_from_q + 2/3 * tau_from_dq
```

也就是说，当前 `u_star` 本质上是：

```text
单关节 bias + 参考加速度前馈
```

它不是一个考虑全身惯量矩阵、关节耦合、浮动基座和所有自由度约束的完整逆动力学解。

当前 hip/knee 使用的参数为：

| 关节 | Jeff | b | gravity(q) |
|---|---:|---:|---|
| RightHipPitch | 1.00508532 | 1.0 | `15.7100627*sin(q) + 2.79723089*cos(q)` |
| RightKnee | 0.2501484 | 1.0 | `4.14117407*sin(q) - 2.09365203*cos(q)` |

### 8.4 与 MuJoCo `mj_inverse` 的对比

为了区分“逆模型完全错误”和“逆模型作为局部近似不适合当前闭环”，对同一条无扰动参考轨迹使用 MuJoCo `mj_inverse` 做了固定基座姿态下的逆动力学对比。

结果如下：

| 关节 | 当前 u_star RMS | MuJoCo `mj_inverse` RMS | 差值 RMS | 相关性 |
|---|---:|---:|---:|---:|
| hip | 2.200 | 2.481 | 0.287 | 0.9997 |
| knee | 1.311 | 1.780 | 0.491 | 0.9818 |

这说明：

- 当前 `u_star` 的波形和 MuJoCo fixed-base inverse dynamics 比较接近；
- hip 幅值偏差约 `0.29 Nm RMS`；
- knee 幅值偏差约 `0.49 Nm RMS`，相对更明显；
- 因此它不是完全随机错误，但只是一个局部单关节近似。

更重要的是，当前 `run_mujoco.py` 的闭环仿真并不是严格 fixed-base inverse dynamics。仿真循环每步都会执行：

```python
data.qpos[0:3] = [0.0, 0.0, height]
data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
data.qvel[0:6] = 0.0
```

也就是将悬浮基座位置、姿态和速度重置。这会让实际闭环 plant 与单关节二阶模型、甚至与普通 fixed-base `mj_inverse` 比较都不同。

因此应该区分两个层次：

| 对比对象 | 结论 |
|---|---|
| 当前 `u_star` vs MuJoCo fixed-base `mj_inverse` | 波形相近，但 knee 幅值已有明显偏差 |
| 当前 `u_star/forwardModel` vs 实际闭环 plant | 差距很大，足以造成 EID 工作点偏移 |

这也解释了为什么 `u_star` 不是简单“数值算错”，而是“作为强前馈注入当前闭环时不适配”。

## 9. 根因排序

### 主因：逆模型前馈 u_star 路径

证据：

- `Ku=0` 仍然和 EID full 一样差。
- `eta_u RMS` 很小，不足以解释大误差。
- `u_star RMS` 明显更大。
- 反事实关闭 `u_star` 后，无扰动误差接近 PD。

判断：

> `u_star` 通过 `r_d_q/r_d_dq` 等效改变参考，使 EID 在无扰动下也形成稳定偏置。这个偏置是当前大误差的主要来源。

### 次因 1：逐关节模型不匹配

证据：

- 简化模型预测加速度与 MuJoCo 实际加速度误差很大。
- EID `forwardModel()` 依赖同一套简化 plant 参数。

判断：

> 当前 `analyticInverseModel()` 不适合作为强前馈直接进入 MuJoCo 全身闭环。

### 次因 2：对角式 EID 不足以处理 hip-knee 耦合

证据：

- hip 扰动时 EID full 的协调误差下降，但 knee 串扰显著增加。
- knee 扰动时 EID full 明显劣于 PD。

判断：

> 当前逐关节对角补偿存在方向性收益，但不具备稳定处理 hip-knee 耦合的能力。

### 次因 3：速度通道有振荡风险

证据：

- `dq-only` 在无扰动和扰动下均明显差。
- 频谱中出现更强振荡成分。

判断：

> 速度通道需要谨慎降增益或过滤，但它不是本轮最大根因。

### 已排除或降级的原因

| 假设 | 当前判断 | 原因 |
|---|---|---|
| alpha 太小是主因 | 否 | alpha=0.90 后核心误差仍存在 |
| Ku 符号反了是主因 | 否 | Ku=0、Ku=0.5、Ku=-1 和 full 误差几乎一致 |
| 参考轨迹 dq 生成错误 | 否 | `q_ref` 数值微分与 `dq_ref` 高度一致 |
| 力矩限幅/安全 flag 是主因 | 否 | alpha=0.90 全部主要运行 `combined_flags=0`, `fatal_flags=0` |

## 10. 建议修改方向

### 10.1 先将 u_star 变成可控开关

建议增加配置项：

```yaml
inverse_feedforward_scale: 0.0
```

或：

```yaml
enable_inverse_feedforward: false
```

控制律改成：

```text
u_star_used = inverse_feedforward_scale * inv.u_star
u_raw = u_star_used - eta_u + kp(ref_q - x_bar_q) + kd(ref_dq - x_bar_dq)
```

默认建议：

```yaml
inverse_feedforward_scale: 0.0
```

理由：

- 反事实实验表明关闭 `u_star` 后无扰动误差显著下降。
- 这能把 EID 诊断从“逆模型前馈 + 扰动估计 + 中心状态反馈”拆成更干净的问题。

### 10.2 增加真正的 shadow observer 模式

当前 `Ku=0` 不是纯 shadow mode，因为 EID 仍然使用：

```text
x_bar
u_star
forwardModel
torque-mode feedback
```

建议新增一个真正的 shadow 模式：

```yaml
controller.kind: position_pd
eid_shadow:
  enabled: true
  log_eta: true
  inject_torque: false
```

该模式下：

- 实际命令仍然由 PD 发出；
- EID observer 并行计算 `eta_q/eta_dq/eta_u`；
- 不改变实际力矩；
- 只用于判断 EID 在无扰动下是否估计出假扰动。

### 10.3 暂时使用 PD-compatible EID 结构

在验证阶段，建议优先测试：

```text
tau = PD_tau - eta_u
```

而不是：

```text
tau = u_star - eta_u + PD_on_x_bar
```

也就是先把 EID 当成 PD 的扰动补偿附加项，而不是同时引入逆模型前馈和中心状态反馈。

### 10.4 重新拟合或替换 plant 模型

如果后续仍希望使用 `u_star`，需要重新确认：

- `Jeff` 是否包含 MuJoCo armature；
- `b` 是否和 MuJoCo damping 一致；
- gravity fitting 是否在当前 base posture、当前自由度固定方式下有效；
- hip/knee 是否需要二自由度耦合模型，而不是逐关节对角模型；
- `forwardModel()` 是否应使用 MuJoCo 或离线辨识出的局部线性模型。

### 10.5 对 hip-knee 使用耦合补偿

当前 EID 形式：

```text
eta_u_h = Ku_hq * eta_q_h + Ku_hdq * eta_dq_h
eta_u_k = Ku_kq * eta_q_k + Ku_kdq * eta_dq_k
```

建议后续尝试：

```text
[eta_u_h, eta_u_k]^T = K_u * [eta_q_h, eta_dq_h, eta_q_k, eta_dq_k]^T
```

即非对角、多输入补偿矩阵。

## 11. 推荐下一轮实验

### E1：u_star scale sweep

固定 `filter_alpha=0.90`，扫描：

```text
inverse_feedforward_scale = 0.0, 0.1, 0.25, 0.5, 1.0
```

指标：

- no-disturbance RMSE；
- disturbance-window RMSE；
- tau RMS；
- tau rate RMS；
- cross transfer ratio。

判断：

- 若 scale 越大误差越大，说明 `u_star` 确认不适合当前 plant。
- 若存在小 scale 最优点，则可保留弱前馈。

### E2：PD-compatible EID

比较：

```text
PD
PD - eta_u
PD - 0.5 eta_u
PD + eta_u
```

目标：

- 单独验证 `eta_u` 的符号和有效性；
- 不让 `u_star/x_bar` 干扰判断。

### E3：shadow observer

在实际 PD 控制下记录：

```text
eta_q
eta_dq
eta_u
q_ref - q
dq_ref - dq
tau_PD
```

目标：

- 看无扰动下 EID 是否估计出周期性假扰动；
- 看扰动窗口内 `eta_u` 是否同向、及时、幅值合理。

### E4：hip/knee coupling experiment

继续使用：

```text
hip + torque
hip - torque
knee + torque
knee - torque
```

额外记录：

```text
S_coord
S_cross
eta_lag
eta_disturbance_mean_ratio
```

目标：

- 判断耦合方向；
- 为非对角 `K_u` 提供数据。

## 12. 总结

本轮实验排除了几个容易误判的方向：

- 不是简单因为 `filter_alpha` 太小；
- 不是简单因为 `Ku` 符号或尺度错误；
- 不是参考速度生成错误；
- 不是力矩限幅或安全 flag 主导。

最强证据来自反事实实验：

```text
EID full q_rmse      ≈ 0.02070
EID u_star≈0 q_rmse ≈ 0.00278
PD q_rmse           ≈ 0.00150
```

因此当前应优先处理：

1. 将 `u_star` 前馈变成可配置开关或缩放项；
2. 默认先关闭或极小化 `u_star`；
3. 增加真正 shadow observer；
4. 在 PD-compatible 结构下重新验证 `eta_u` 的符号、幅值和滞后；
5. 后续再考虑重新辨识 plant 或引入 hip-knee 耦合补偿。

一句话结论：

> 当前 EID 跟踪问题的核心不是估计器滤波速度，而是错误或不一致的逆模型前馈 `u_star` 被强行注入闭环，改变了原本稳定的 PD 工作点。
