# H1 全身关节控制 — 实验数据库设计方案

## 1. 背景与目标

当前项目每次实验运行产生以下数据，全部以文件形式散落在时间戳目录中：

| 层级 | 数据内容 | 当前存储 |
|------|---------|---------|
| 配置层 | YAML（关节参数、增益、轨迹、力矩限制） | `config/*.yaml` |
| 时序层 | 每步(500Hz) 19个关节×32维debug信号 | `eid_mujoco_closed_loop_log.csv` |
| 汇总层 | 每关节 RMSE、max error、max torque | `h1_eid_summary.csv` |
| 对比层 | EID vs PD 的 RMSE ratio、误差对比 | `eid_vs_pd_summary.csv` |
| 索引层 | 所有 run 的元数据扫描结果 | `runs_index.csv` (由 `index_runs.py` 生成) |
| 可视化层 | PNG图表、MP4/GIF视频 | 时间戳目录下 |

**当前痛点**：

1. **跨 run 查询困难** — 如"kp=200 且 RMSE<0.01 的所有 RightKnee run"，需要手动解析多个 CSV
2. **配置参数和结果分离** — YAML 配置和 CSV 结果独立存储，关联需要解析两端
3. **大规模时序数据用 CSV 低效** — 19关节 × 7500步 × 32字段 = 数百万数据点/run，CSV 读写慢、占用大
4. **参数扫描不可追溯** — 批量修改 kp/kd 后无法直接查询参数→性能关系
5. **图表是二进制文件** — PNG/GIF 无法被查询，需要已知 run_id 才能定位

**目标**：构建一个轻量级、零服务器依赖的数据库系统，支持：

- 按任意配置参数组合筛选实验（如 kp > 150, reference_mode = closed_loop）
- 按关节汇总指标排序和比较（RMSE, max error, max torque）
- 跨实验对比时序信号（如叠加多个 run 的 q_error 曲线）
- 参数→性能关系分析和可视化
- 消融实验追踪

---

## 2. 架构总览

```
SQLite (experiments.db)          Parquet 文件                  DuckDB (查询层)
┌──────────────────────┐    ┌─────────────────────┐      ┌──────────────────────┐
│ experiments          │    │ timeseries.parquet  │      │ 跨库 SQL 联合查询    │
│ joint_configs        │    │ (每 run 一个文件)    │      │ SQLite + Parquet     │
│ joint_summaries      │    │                     │      │ DataFrame 输出       │
│ comparison_results   │    │ 列: cycle, t,       │      │ Streamlit 可视化     │
│ ablation_configs     │    │ joint_id, q, dq,    │      └──────────────────────┘
│                      │    │ q_ref, tau_cmd,     │
│ 存储: 元数据/配置/汇总│    │ u_star, eta_q, ...  │
└──────────────────────┘    └─────────────────────┘
```

**选型理由**：

| 方案 | 不选原因 |
|------|---------|
| 纯 SQLite 存储时序 | 时序数据量大（百万行/run），SQLite BLOB 效率低 |
| TimescaleDB / InfluxDB | 需要额外部署服务器，对单机研究过重 |
| HDF5 | 不如 Parquet 在 Python 生态中通用，并发读取差 |
| MongoDB | 时序查询弱，不适合数值分析 |

---

## 3. SQLite 表设计

### 3.1 `experiments` — 实验运行记录

```sql
CREATE TABLE experiments (
    experiment_id     INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id            TEXT NOT NULL UNIQUE,
    timestamp         DATETIME NOT NULL,
    object_type       TEXT NOT NULL CHECK (object_type IN ('mujoco', 'mock', 'real')),
    controller_method TEXT NOT NULL CHECK (controller_method IN ('EID', 'PD', 'PID', 'EID_vs_PD')),
    duration_s        REAL NOT NULL,
    control_dt        REAL NOT NULL,
    config_path       TEXT,
    config_snapshot   TEXT,          -- 完整 YAML 内容的 JSON 序列化
    run_dir           TEXT,          -- 原始输出目录相对路径
    notes             TEXT,
    created_at        DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### 3.2 `joint_configs` — 每个实验中每个关节的控制器配置

```sql
CREATE TABLE joint_configs (
    config_id       INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
    joint_id        INTEGER NOT NULL CHECK (joint_id >= 0 AND joint_id < 20),
    joint_name      TEXT NOT NULL,
    enabled         INTEGER NOT NULL DEFAULT 1,

    -- EID 控制器参数
    kp                  REAL,
    kd                  REAL,
    observer_gain_q     REAL,
    observer_gain_dq    REAL,
    filter_alpha        REAL,
    reference_mode      TEXT CHECK (reference_mode IN ('open_loop', 'closed_loop')),
    reference_signal    TEXT CHECK (reference_signal IN ('sine', 'step')),
    ref_center          REAL,
    ref_amplitude       REAL,
    ref_frequency       REAL,
    ref_phase           REAL,
    ref_step_time       REAL,
    startup_ramp_duration REAL,
    eid_tau_limit       REAL,
    eid_tau_slew_rate   REAL,
    torque_safe_kp      REAL,
    torque_safe_kd      REAL,
    inverse_q_weight    REAL,
    inverse_dq_weight   REAL,
    closed_loop_reference_tau REAL,

    -- 被控对象模型参数 (Plant Model)
    plant_Jeff      REAL,
    plant_b         REAL,
    plant_gravityA  REAL,
    plant_gravityB  REAL,
    plant_tau0      REAL,
    plant_q_min     REAL,
    plant_q_max     REAL,
    plant_tau_max   REAL,

    UNIQUE(experiment_id, joint_id)
);
```

### 3.3 `joint_summaries` — 每个关节的汇总性能指标

```sql
CREATE TABLE joint_summaries (
    summary_id      INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
    joint_id        INTEGER NOT NULL,

    -- 跟踪性能
    q_rmse          REAL,           -- 位置跟踪 RMSE (rad)
    q_max_error     REAL,           -- 最大绝对位置误差 (rad)
    dq_rmse         REAL,           -- 速度跟踪 RMSE (rad/s)
    q_error_ref_type TEXT,          -- 'shaped' 或 'raw'

    -- 力矩统计
    tau_abs_max     REAL,           -- 最大绝对力矩命令 (N·m)
    tau_mean_abs    REAL,           -- 平均绝对力矩 (N·m)
    tau_rms         REAL,           -- 力矩 RMS (N·m)

    -- 安全标志
    safety_flags    INTEGER DEFAULT 0,

    -- 状态范围
    q_min_actual    REAL,
    q_max_actual    REAL,
    dq_max_actual   REAL,
    lowstate_age_max REAL,

    UNIQUE(experiment_id, joint_id),
    FOREIGN KEY (experiment_id) REFERENCES experiments(experiment_id)
);
```

### 3.4 `comparison_results` — EID vs PD 对比结果

```sql
CREATE TABLE comparison_results (
    comparison_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
    joint_id        INTEGER NOT NULL,

    eid_rmse        REAL,
    pd_rmse         REAL,
    rmse_ratio      REAL,           -- PD_RMSE / EID_RMSE (>1 表示 EID 更优；<1 表示 PD 更优)
    eid_max_error   REAL,
    pd_max_error    REAL,
    eid_mean_abs_tau REAL,
    pd_mean_abs_tau REAL,

    -- 扰动条件
    disturb_type        TEXT DEFAULT 'none',
    disturb_magnitude   REAL,       -- 占 tau_limit 的比例
    disturb_frequency   REAL,       -- Hz

    -- PD 参数
    pd_kp_used      REAL,
    pd_kd_used      REAL,

    UNIQUE(experiment_id, joint_id),
    FOREIGN KEY (experiment_id) REFERENCES experiments(experiment_id)
);
```

### 3.5 `ablation_configs` — 消融实验标记

```sql
CREATE TABLE ablation_configs (
    ablation_id     INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
    no_feedforward  INTEGER DEFAULT 0,    -- 切除前馈
    no_observer     INTEGER DEFAULT 0,    -- 切除扰动观测器
    no_ref_mod      INTEGER DEFAULT 0,    -- 切除参考轨迹修正
    no_feedback     INTEGER DEFAULT 0,    -- 切除反馈 (纯前馈)
    UNIQUE(experiment_id)
);
```

### 3.6 `timeseries_files` 与 `control_metrics` — 控制工程诊断

```sql
CREATE TABLE timeseries_files (
    file_id        INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id  INTEGER NOT NULL REFERENCES experiments(experiment_id),
    path           TEXT NOT NULL,
    format         TEXT NOT NULL DEFAULT 'parquet',
    rows           INTEGER,
    sample_rate_hz REAL,
    schema_version TEXT NOT NULL DEFAULT '1',
    created_at     DATETIME DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(experiment_id, path)
);

CREATE TABLE control_metrics (
    metric_id         INTEGER PRIMARY KEY AUTOINCREMENT,
    experiment_id     INTEGER NOT NULL REFERENCES experiments(experiment_id),
    joint_id          INTEGER NOT NULL,
    metric_name       TEXT NOT NULL,
    value             REAL,
    unit              TEXT,
    window_start_s    REAL,
    window_end_s      REAL,
    source            TEXT,
    algorithm_version TEXT NOT NULL DEFAULT 'control_metrics_v1',
    created_at        DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

第一版指标包括 `q_rmse`, `q_mae`, `q_iae`, `q_max_abs_error`,
`tau_mean_abs`, `tau_rms`, `tau_abs_max`, `tau_energy`,
`tau_saturation_duty`, `joint_flag_any`。对非零幅值正弦轨迹额外计算
`tracking_gain`, `phase_lag_rad`, `phase_lag_deg`, `amplitude_error`,
`bias_error`。

---

## 4. 时序数据：Parquet 存储

### 4.1 存储路径

```text
data/experiments/{run_id}/timeseries.parquet
```

### 4.2 Schema

采用"长格式"(tall format) 存储，每行是一个 (时间步, 关节) 组合：

| 列名 | 类型 | 说明 |
|------|------|------|
| `cycle` | int64 | 控制周期编号 |
| `t` | float64 | 仿真时间 (s) |
| `dt` | float64 | 控制周期 (s) |
| `lowstate_age` | float32 | LowState 消息延迟 (s) |
| `joint_id` | int8 | 关节编号 (0-19) |
| `q` | float32 | 测量位置 (rad) |
| `dq` | float32 | 测量速度 (rad/s) |
| `tau_est` | float32 | 估计力矩 (N·m) |
| `q_ref` | float32 | 参考位置 shaped (debug_0) |
| `dq_ref` | float32 | 参考速度 shaped (debug_1) |
| `q_error_shaped` | float32 | q_ref - q (debug_4) |
| `dq_error_shaped` | float32 | dq_ref - dq (debug_5) |
| `u_star` | float32 | 前馈力矩 (debug_6) |
| `u_feedback` | float32 | 反馈力矩 (debug_7) |
| `u_t` | float32 | 总力矩输出 (debug_8) |
| `eta_q` | float32 | 扰动估计位置分量 (debug_9) |
| `eta_dq` | float32 | 扰动估计速度分量 (debug_10) |
| `x_hat_q` | float32 | 名义模型预测位置 (debug_11) |
| `x_hat_dq` | float32 | 名义模型预测速度 (debug_12) |
| `rho_q` | float32 | 逆模型位置残差 (debug_13) |
| `rho_dq` | float32 | 逆模型速度残差 (debug_14) |
| `x_bar_q` | float32 | 补偿后预测位置 (debug_15) |
| `q_ref_next` | float32 | 下一步参考位置 (debug_16) |
| `dq_ref_next` | float32 | 下一步参考速度 (debug_17) |
| `x_bar_dq` | float32 | 补偿后预测速度 (debug_18) |
| `r_d_q` | float32 | 修正参考位置 (debug_19) |
| `r_d_dq` | float32 | 修正参考速度 (debug_20) |
| `e_q` | float32 | 反馈误差位置 (debug_21) |
| `e_dq` | float32 | 反馈误差速度 (debug_22) |
| `observer_qacc` | float32 | 观测器加速度 (debug_23) |
| `observer_tau_applied` | float32 | 观测器施加力矩 (debug_24) |
| `u_raw` | float32 | 限幅前力矩 (debug_25) |
| `q_ref_raw` | float32 | 原始参考位置 (debug_26) |
| `dq_ref_raw` | float32 | 原始参考速度 (debug_27) |
| `q_error_raw` | float32 | q_ref_raw - q (debug_28) |
| `dq_error_raw` | float32 | dq_ref_raw - dq (debug_29) |
| `q_error_shaped2` | float32 | ref.now.q - q 用于闭环模式 (debug_30) |
| `dq_error_shaped2` | float32 | ref.now.dq - dq 用于闭环模式 (debug_31) |
| `flags` | int32 | 安全标志位 |

### 4.3 Parquet 优势

- **压缩率高**：相比 CSV 文件体积缩小 5-10 倍
- **列式读取**：只需 `q_error` 时不会加载全部 33 列
- **快速 I/O**：`pd.read_parquet()` 比 `pd.read_csv()` 快 5-10 倍
- **DuckDB 兼容**：可以直接用 SQL 查询 Parquet 文件
- **跨语言**：Python (pandas/polars)、R、Julia 都可读取

---

## 5. 查询层：DuckDB

DuckDB 可以同时连接 SQLite 和 Parquet，实现统一的 SQL 查询：

### 5.1 参数→性能分析

```sql
-- 查询所有 RightKnee 的 RMSE 随 kp 变化
SELECT
    e.run_id,
    jc.kp,
    jc.kd,
    js.q_rmse,
    js.tau_abs_max
FROM sqlite_scan('data/experiments.db', 'experiments') e
JOIN sqlite_scan('data/experiments.db', 'joint_configs') jc
    ON e.experiment_id = jc.experiment_id
JOIN sqlite_scan('data/experiments.db', 'joint_summaries') js
    ON e.experiment_id = js.experiment_id AND jc.joint_id = js.joint_id
WHERE jc.joint_name = 'RightKnee'
  AND e.controller_method = 'EID'
ORDER BY jc.kp;
```

### 5.2 EID vs PD 全关节对比

```sql
SELECT
    ejc.joint_name,
    AVG(em.value) AS avg_eid_rmse,
    AVG(pm.value) AS avg_pd_rmse,
    AVG(pm.value / NULLIF(em.value, 0)) AS eid_improvement_factor,
    COUNT(*) AS n_runs
FROM sqlite_scan('data/experiments.db', 'comparison_pairs') cp
JOIN sqlite_scan('data/experiments.db', 'control_metrics') em
    ON em.experiment_id = cp.eid_experiment_id AND em.metric_name = 'q_rmse'
JOIN sqlite_scan('data/experiments.db', 'control_metrics') pm
    ON pm.experiment_id = cp.pd_experiment_id
   AND pm.metric_name = em.metric_name AND pm.joint_id = em.joint_id
JOIN sqlite_scan('data/experiments.db', 'joint_configs') ejc
    ON ejc.experiment_id = cp.eid_experiment_id AND ejc.joint_id = em.joint_id
GROUP BY ejc.joint_name
ORDER BY eid_improvement_factor DESC;
```

### 5.3 跨实验时序叠加

```sql
-- 叠加多个 run 的 RightKnee q_error 曲线
SELECT t, q_error_shaped, run_id
FROM read_parquet('data/experiments/*/timeseries.parquet')
WHERE joint_id = 2
  AND run_id IN (
      SELECT run_id FROM sqlite_scan('data/experiments.db', 'experiments')
      WHERE experiment_id IN (1, 5, 10)
  );
```

### 5.4 消融实验影响

```sql
SELECT
    a.no_feedforward, a.no_observer, a.no_ref_mod, a.no_feedback,
    js.joint_id,
    AVG(js.q_rmse) AS avg_rmse
FROM sqlite_scan('data/experiments.db', 'ablation_configs') a
JOIN sqlite_scan('data/experiments.db', 'joint_summaries') js
    ON a.experiment_id = js.experiment_id
GROUP BY a.no_feedforward, a.no_observer, a.no_ref_mod, a.no_feedback, js.joint_id
ORDER BY avg_rmse DESC;
```

---

## 6. 实施计划

### Phase 1: 数据库初始化与写入器

**文件**：`scripts/db_manager.py`

功能：
- `init_db(db_path)` — 创建 SQLite 表
- `import_experiment(run_dir)` — 解析一个 run 目录下的所有文件并写入数据库
  - 解析 `input_config.yaml` → `experiments` + `joint_configs`
  - 解析 `h1_eid_summary.csv` 或 `eid_vs_pd_summary.csv` → `joint_summaries` / `comparison_results`
  - 转换 `eid_mujoco_closed_loop_log.csv` → `timeseries.parquet`
- `import_all_runs(data_root)` — 扫描 `data/` 下所有 run 目录并批量导入
- `rebuild_index(data_root)` — 全量重建（扫描 → 清库 → 重新导入）

### Phase 2: 实验脚本集成

修改以下脚本，在实验结束后自动写入数据库：

- `scripts/run_lower_body_mujoco_test.py` — MuJoCo 单控制器实验
- `scripts/compare_eid_pd_mujoco.py` — EID vs PD 对比实验

改动方式：在输出 CSV/图表后追加调用 `db_manager.import_experiment(out_dir)`。

### Phase 3: Dashboard 升级

**文件**：`scripts/db_dashboard.py`（Streamlit 应用，已基于 SQLite 实现）

改动：
- 从 CSV index 切换到 DuckDB 查询 SQLite + Parquet
- 新增分析页面：
  - **参数扫描热力图** — kp × kd → RMSE 的二维热力图
  - **消融对比视图** — 并排显示不同消融配置的误差曲线
  - **时序叠加比较** — 选择多个 run 叠加同一信号
  - **关节排名** — 按 RMSE / max torque 排名各关节的控制性能

### Phase 4: CLI 查询工具

**文件**：`scripts/query_db.py`

```bash
# 列出所有 EID 实验中 RightKnee RMSE < 0.01 的 run
python scripts/query_db.py --joint RightKnee --controller EID --max-rmse 0.01

# 比较两个 run 的 q_error 统计
python scripts/query_db.py --compare run_001 run_002 --field q_error

# 导出指定 run 的时序数据为 CSV
python scripts/query_db.py --export run_001 --format csv
```

---

## 7. 目录结构

实施完成后的推荐目录结构：

```text
data/
├── experiments.db                    # SQLite 数据库
├── experiments/                      # 时序数据 (Parquet)
│   └── {run_id}/
│       └── timeseries.parquet
├── mujoco_fit/
│   └── runs/                         # 原始输出保留 (向后兼容)
│       └── {timestamp}_{run_id}/
│           ├── input_config.yaml
│           ├── eid_mujoco_closed_loop_log.csv
│           ├── h1_eid_summary.csv
│           ├── eid_vs_pd_summary.csv
│           ├── eid_vs_pd_comparison.png
│           ├── eid_vs_pd_summary.png
│           └── ...
└── runs_index.csv                    # 保留作为向后兼容
```

---

## 8. 依赖

```
# requirements-dashboard.txt (追加)
duckdb>=1.0
pyarrow>=15.0
```

SQLite 是 Python 标准库，无需额外安装。
