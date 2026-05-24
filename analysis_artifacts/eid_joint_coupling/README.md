# EID Joint Coupling Analysis

本专题整理 EID 控制器在单关节右膝、右髋-右膝双关节耦合、不同控制周期下的实验分析。

## 入口

- 报告：`docs/eid_joint_coupling_analysis.md`
- 分析脚本：
  - `scripts/analyze_eid_joint_coupling.py`
  - `scripts/analyze_eid_control_dt_sweep.py`
  - `scripts/plot_eid_position_tracking.py`
  - `scripts/test_eid_right_leg_tracking.py`
- 派生配置：`configs/*.yaml`

## 说明

该专题的核心用途是区分 EID 在低维单关节场景中的稳定表现，以及在髋膝强耦合场景中的速度/力矩高频振荡问题。
