# 实验数据库使用指南

## 快速开始

```powershell
# 1. 安装依赖（一次性）
pip install -r requirements-dashboard.txt

# 2. 迁移旧数据（如果是从旧版升级）
python scripts/db_manager.py migrate

# 3. 导入实验
python scripts/db_manager.py import-all

# 4. 打开分析面板
streamlit run scripts/db_dashboard.py
```

---

## 架构

```
EID 实验 → eid_mujoco_closed_loop_log.csv → plot 脚本 → eid_summary.csv
                                                             │
PD 实验  → pd_mujoco_closed_loop_log.csv  → plot 脚本 → pd_summary.csv
                                                             │
                              db_manager.py import ──────────┤
                                                             │
                                                    experiments.db (SQLite)
                                                    ├── experiments
                                                    ├── joint_configs
                                                    ├── joint_summaries
                                                    └── comparison_pairs
                                                             │
                              db_dashboard.py ───────────────┘  (Streamlit 界面)
```

- 每个实验运行一个控制器（EID 或 PD），独立产生日志和汇总
- 跨控制器对比在分析层面通过 `comparison_pairs` 表 JOIN 完成
- 旧的 `eid_vs_pd_summary.csv` 导入时会自动拆分为两个实验 + 配对

---

## 运行实验的标准流程

### 运行 EID 实验

```powershell
python scripts/render_mujoco_eid_closed_loop.py \
    --config config/h1_full_body_mujoco_fit.yaml \
    --duration 10.0 \
    --export-summary
```

输出到 `data/mujoco_fit/latest/`：
- `eid_mujoco_closed_loop_log.csv` — 逐帧日志
- `eid_summary.csv` — 每个关节的汇总指标

### 运行 PD 实验

```powershell
python scripts/run_pd_mujoco.py \
    --config config/h1_full_body_mujoco_fit.yaml \
    --duration 10.0 \
    --disturb-type sinusoidal
```

输出到 `data/mujoco_fit/runs/<timestamp>_pd_sinusoidal/`：
- `pd_mujoco_closed_loop_log.csv` — 逐帧日志
- `pd_summary.csv` — 每个关节的汇总指标

### 手动配对 EID 和 PD 实验

```powershell
python scripts/db_manager.py pair <eid_exp_id> <pd_exp_id>
```

---

## db_manager.py 命令参考

| 命令 | 说明 |
|------|------|
| `init` | 创建所有表 |
| `import <dir>` | 导入单个实验目录 |
| `import-all` | 扫描并导入所有实验 |
| `rebuild` | 清空数据库，重新导入 |
| `stats` | 查看统计信息 |
| `migrate` | 将旧的 EID_vs_PD 实验拆分为独立实验并创建配对 |
| `pair <eid_id> <pd_id>` | 手动创建 EID/PD 实验配对 |
| `csv-to-parquet` | CSV 日志转为 Parquet 格式 |

---

## 数据库表结构速查

| 表名 | 内容 | 关键列 |
|------|------|--------|
| `experiments` | 实验运行记录 | run_id, timestamp, controller_method, disturb_type |
| `joint_configs` | 每个关节的控制器参数 | kp, kd, eid_tau_limit, observer_gain_q, plant_* |
| `joint_summaries` | 单控制器汇总指标 | q_rmse, q_max_error, tau_abs_max, tau_mean_abs |
| `comparison_pairs` | EID/PD 实验配对 | eid_experiment_id, pd_experiment_id, disturb_type |
| `comparison_results` | (遗留) 旧的 EID vs PD 合并数据 | eid_rmse, pd_rmse, rmse_ratio |
| `timeseries_files` | Parquet 时序文件索引 | path, rows, sample_rate_hz |
| `control_metrics` | 控制工程诊断指标长表 | q_iae, tau_energy, tracking_gain, phase_lag_deg |

---

## Dashboard 标签页说明

### 1. Overview — 实验总览

- 显示 EID/PD 实验数量、配对数
- 实验列表可按控制器方法和扰动类型筛选
- 显示所有配对关系

### 2. Joint Analysis — 关节性能对比

- 从 `comparison_pairs` + `joint_summaries` 计算 EID vs PD RMSE 柱状图
- `PD_RMSE / EID_RMSE` 称为 EID improvement factor；`>1` 表示 EID RMSE 更低，`<1` 表示 PD RMSE 更低

### 3. Control Metrics — 控制指标诊断

- 按关节和控制器展示 q_rmse, q_iae, tau_energy, tau_saturation_duty 等指标
- 用于判断误差、控制代价和饱和程度之间的权衡

### 4. Sine Tracking — 正弦跟踪诊断

- 展示 tracking_gain, phase_lag_deg, amplitude_error, bias_error
- 支持 EID/PD 配对对比

### 5. Timeseries — 时序曲线

- 选择任意实验 → 自由勾选信号 → 绘图
- 优先读取 `timeseries.parquet`；缺失时回退到 per-controller CSV 或旧 combined CSV

### 6. SQL Query — 自定义查询

- 直接写 SQL 查询 experiments.db

---

## 常用 SQL 查询

### 对比 EID 和 PD 性能

```sql
SELECT es.joint_id, jc.joint_name,
       AVG(es.q_rmse) AS eid_rmse,
       AVG(ps.q_rmse) AS pd_rmse,
       AVG(ps.q_rmse) / NULLIF(AVG(es.q_rmse), 0) AS eid_improvement_factor
FROM comparison_pairs cp
JOIN joint_summaries es ON cp.eid_experiment_id = es.experiment_id
JOIN joint_summaries ps ON cp.pd_experiment_id = ps.experiment_id AND es.joint_id = ps.joint_id
JOIN joint_configs jc ON es.experiment_id = jc.experiment_id AND es.joint_id = jc.joint_id
WHERE es.q_rmse IS NOT NULL
GROUP BY es.joint_id
ORDER BY eid_improvement_factor DESC;
```

### 找某个关节的最佳 EID 实验

```sql
SELECT e.run_id, js.q_rmse, js.tau_mean_abs
FROM joint_summaries js
JOIN experiments e ON js.experiment_id = e.experiment_id
WHERE e.controller_method = 'EID' AND js.joint_id = 2
ORDER BY js.q_rmse ASC LIMIT 5;
```

### 查看某个实验的配对

```sql
SELECT eid.run_id AS eid_run, pd.run_id AS pd_run, cp.disturb_type
FROM comparison_pairs cp
JOIN experiments eid ON cp.eid_experiment_id = eid.experiment_id
JOIN experiments pd ON cp.pd_experiment_id = pd.experiment_id
WHERE cp.eid_experiment_id = ? OR cp.pd_experiment_id = ?;
```
