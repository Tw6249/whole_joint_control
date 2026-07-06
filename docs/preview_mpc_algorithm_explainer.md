# Preview-MPC 算法说明

当前工程中 `policy_interpolation: preview_mpc` 只有一种含义：3 参考点 soft-preview MPC，无 terminal 速度/加速度惩罚。完整公式见 [MPC方法梳理.md](MPC方法梳理.md)。

最小配置：

```yaml
policy_interpolation: preview_mpc
policy_reference_points: 3
```

旧字段 `policy_mpc_variant` 已移除；继续使用该字段会触发配置错误。

