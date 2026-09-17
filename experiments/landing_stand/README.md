# 落地站立实验

本目录保存两组入口：

- `run_real_landing_stand_preview_mpc_10j_eid_ku*.py`：10 关节 Preview-MPC + EID，包含 Ko=(0.8,0.2) 变体。
- `run_real_landing_stand_online_policy_lstm_eid_ku_ko08*.py`：在线 LSTM + EID，包含默认、`knee80`、`knee80_jump020` 和 `max_tau` 变体。

每个入口的默认配置在 `configs/`，保留与脚本对应的原文件名。在线策略模型位于 `models/policies/policy_lstm_1.pt`，部署参数位于 `config/policy/h1.yaml`。

```bash
python experiments/landing_stand/run_real_landing_stand_online_policy_lstm_eid_ku_ko08.py --help
python experiments/landing_stand/run_real_landing_stand_online_policy_lstm_eid_ku_ko08.py
```

默认仅打印计划。可通过 `--config` 指定完整 YAML；同时按实验需要设置 `--condition` 和 `--out-dir`。现有参数变体脚本仍作为复现入口保留，本轮未合并其控制流程或修改控制参数。
