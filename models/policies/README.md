# 在线策略模型

`policy_lstm_1.pt` 从整理前的项目根目录原样迁入，没有重新训练或转换。现有材料未提供可核验的训练提交、训练配置或模型来源，因此不推断其训练来源。

配套部署参数：`config/policy/h1.yaml`。使用该模型的实验配置位于 `experiments/landing_stand/configs/`，通过 `online_policy.model_path` 与 `online_policy.deploy_yaml` 指定资源。

从项目根目录启动程序，所有上述相对路径以该目录为基准。C++ 在线推理需启用 LibTorch；基础 EID/PD、mock 与 MuJoCo stepper 构建不要求启用策略推理。
