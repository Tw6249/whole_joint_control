const fs = require("fs");
const path = require("path");

const repoRoot = path.resolve(__dirname, "..", "..", "..");
const outDir = path.join(repoRoot, "analysis_artifacts", "right_knee_interpolation", "figures");
fs.mkdirSync(outDir, { recursive: true });

const runs = [
  {
    key: "position_only_open_loop",
    label: "Open-loop quintic",
    color: "#1f77b4",
    file: "data/mujoco_fit/track_position_only_open_loop/mujoco_closed_loop_log.csv",
  },
  {
    key: "ruckig_position_only",
    label: "Ruckig",
    color: "#d62728",
    file: "data/mujoco_fit/track_position_only_ruckig/mujoco_closed_loop_log.csv",
  },
  {
    key: "rl_smoothed",
    label: "RL-smoothed quintic",
    color: "#2ca02c",
    file: "data/mujoco_fit/track_position_only_rl_smoothed/mujoco_closed_loop_log.csv",
  },
];

function parseCsv(text) {
  const lines = text.trim().split(/\r?\n/);
  const header = lines[0].split(",");
  return lines.slice(1).map((line) => {
    const parts = line.split(",");
    const row = {};
    header.forEach((key, i) => {
      const value = parts[i];
      const num = Number(value);
      row[key] = Number.isFinite(num) && value !== "" ? num : value;
    });
    return row;
  });
}

function readCsv(relPath) {
  return parseCsv(fs.readFileSync(path.join(repoRoot, relPath), "utf8"));
}

function rightKneeRows(run) {
  return readCsv(run.file)
    .filter((row) => row.joint_id === 2)
    .map((row) => ({
      t: row.t,
      qRef: row.q_ref_shaped,
      dqRef: row.dq_ref_shaped,
      qActual: row.q_actual,
      dqActual: row.dq_actual,
      tau: row.u_t,
    }));
}

function deriveKinematics(rows) {
  return rows.map((row, i) => {
    const prev = rows[Math.max(0, i - 1)];
    const dt = Math.max(row.t - prev.t, 1e-9);
    const ddqRef = i === 0 ? 0 : (row.dqRef - prev.dqRef) / dt;
    const prevDdq =
      i <= 1 ? 0 : (prev.dqRef - rows[i - 2].dqRef) / Math.max(prev.t - rows[i - 2].t, 1e-9);
    const jerkRef = i <= 1 ? 0 : (ddqRef - prevDdq) / dt;
    return { ...row, ddqRef, jerkRef, qErr: row.qRef - row.qActual };
  });
}

const series = runs.map((run) => ({
  ...run,
  rows: deriveKinematics(rightKneeRows(run)),
}));

const referenceMetrics = readCsv(
  "data/mujoco_fit/right_knee_position_only_interpolation_comparison_metrics.csv"
);
const trackingMetrics = readCsv(
  "data/mujoco_fit/right_knee_closed_loop_tracking_position_only_metrics.csv"
);

function fmt(x, digits = 3) {
  if (!Number.isFinite(x)) return "";
  if (Math.abs(x) >= 100) return x.toFixed(1);
  if (Math.abs(x) >= 10) return x.toFixed(2);
  return x.toFixed(digits);
}

function escapeXml(s) {
  return String(s)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function linePath(points, xScale, yScale) {
  return points
    .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
    .map((p, i) => `${i === 0 ? "M" : "L"} ${xScale(p.x).toFixed(2)} ${yScale(p.y).toFixed(2)}`)
    .join(" ");
}

function minMax(values, padFrac = 0.08) {
  let min = Infinity;
  let max = -Infinity;
  for (const v of values) {
    if (Number.isFinite(v)) {
      min = Math.min(min, v);
      max = Math.max(max, v);
    }
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [0, 1];
  if (Math.abs(max - min) < 1e-12) {
    min -= 1;
    max += 1;
  }
  const pad = (max - min) * padFrac;
  return [min - pad, max + pad];
}

function ticks(min, max, count = 5) {
  const out = [];
  for (let i = 0; i < count; i++) {
    out.push(min + (i * (max - min)) / (count - 1));
  }
  return out;
}

function drawLineFigure(filename, title, panels) {
  const width = 980;
  const panelHeight = 185;
  const margin = { left: 78, right: 22, top: 62, bottom: 55 };
  const gap = 44;
  const height = margin.top + margin.bottom + panels.length * panelHeight + (panels.length - 1) * gap;
  const plotWidth = width - margin.left - margin.right;
  const tMax = Math.max(...series.flatMap((s) => s.rows.map((r) => r.t)));
  const xScale = (x) => margin.left + (x / tMax) * plotWidth;

  let svg = [];
  svg.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">`);
  svg.push(`<rect width="100%" height="100%" fill="white"/>`);
  svg.push(`<style>
    text { font-family: Arial, Helvetica, sans-serif; fill: #111827; }
    .title { font-size: 22px; font-weight: 700; }
    .axis { stroke: #1f2937; stroke-width: 1.1; }
    .grid { stroke: #e5e7eb; stroke-width: 1; }
    .tick { font-size: 12px; fill: #374151; }
    .label { font-size: 14px; font-weight: 600; fill: #111827; }
    .legend { font-size: 13px; fill: #111827; }
  </style>`);
  svg.push(`<text x="${margin.left}" y="32" class="title">${escapeXml(title)}</text>`);

  panels.forEach((panel, pi) => {
    const y0 = margin.top + pi * (panelHeight + gap);
    const yValues = series.flatMap((s) => s.rows.map((r) => panel.value(r)));
    const [yMin, yMax] = panel.domain || minMax(yValues, panel.padFrac ?? 0.08);
    const yScale = (y) => y0 + panelHeight - ((y - yMin) / (yMax - yMin)) * panelHeight;

    for (const tv of ticks(0, tMax, 7)) {
      const x = xScale(tv);
      svg.push(`<line x1="${x.toFixed(2)}" y1="${y0}" x2="${x.toFixed(2)}" y2="${y0 + panelHeight}" class="grid"/>`);
      if (pi === panels.length - 1) {
        svg.push(`<text x="${x.toFixed(2)}" y="${y0 + panelHeight + 23}" text-anchor="middle" class="tick">${fmt(tv, 1)}</text>`);
      }
    }
    for (const yv of ticks(yMin, yMax, 5)) {
      const y = yScale(yv);
      svg.push(`<line x1="${margin.left}" y1="${y.toFixed(2)}" x2="${margin.left + plotWidth}" y2="${y.toFixed(2)}" class="grid"/>`);
      svg.push(`<text x="${margin.left - 10}" y="${(y + 4).toFixed(2)}" text-anchor="end" class="tick">${fmt(yv, panel.tickDigits ?? 2)}</text>`);
    }
    svg.push(`<line x1="${margin.left}" y1="${y0 + panelHeight}" x2="${margin.left + plotWidth}" y2="${y0 + panelHeight}" class="axis"/>`);
    svg.push(`<line x1="${margin.left}" y1="${y0}" x2="${margin.left}" y2="${y0 + panelHeight}" class="axis"/>`);
    svg.push(`<text transform="translate(20 ${(y0 + panelHeight / 2).toFixed(2)}) rotate(-90)" text-anchor="middle" class="label">${escapeXml(panel.yLabel)}</text>`);

    for (const s of series) {
      const points = s.rows.map((r) => ({ x: r.t, y: panel.value(r) }));
      svg.push(`<path d="${linePath(points, xScale, yScale)}" fill="none" stroke="${s.color}" stroke-width="2.1"/>`);
    }
  });

  const legendY = height - 20;
  let legendX = margin.left;
  for (const s of series) {
    svg.push(`<line x1="${legendX}" y1="${legendY - 5}" x2="${legendX + 34}" y2="${legendY - 5}" stroke="${s.color}" stroke-width="3"/>`);
    svg.push(`<text x="${legendX + 42}" y="${legendY}" class="legend">${escapeXml(s.label)}</text>`);
    legendX += 245;
  }
  svg.push(`<text x="${margin.left + plotWidth / 2}" y="${height - 2}" text-anchor="middle" class="label">Time (s)</text>`);
  svg.push(`</svg>`);
  fs.writeFileSync(path.join(outDir, filename), svg.join("\n"), "utf8");
}

function drawGroupedBars(filename, title, groups, rows, modeLabels) {
  const width = 980;
  const height = 520;
  const margin = { left: 86, right: 30, top: 70, bottom: 112 };
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const colors = ["#1f77b4", "#d62728", "#2ca02c"];
  const groupGap = 34;
  const groupWidth = (plotWidth - groupGap * (groups.length - 1)) / groups.length;
  const barGap = 7;
  const barWidth = (groupWidth - barGap * (rows.length - 1)) / rows.length;
  const yMax = Math.max(...groups.flatMap((g) => rows.map((r) => g.value(r)))) * 1.18;
  const yScale = (y) => margin.top + plotHeight - (y / yMax) * plotHeight;

  let svg = [];
  svg.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">`);
  svg.push(`<rect width="100%" height="100%" fill="white"/>`);
  svg.push(`<style>
    text { font-family: Arial, Helvetica, sans-serif; fill: #111827; }
    .title { font-size: 22px; font-weight: 700; }
    .axis { stroke: #1f2937; stroke-width: 1.1; }
    .grid { stroke: #e5e7eb; stroke-width: 1; }
    .tick { font-size: 12px; fill: #374151; }
    .label { font-size: 14px; font-weight: 600; }
    .legend { font-size: 13px; }
  </style>`);
  svg.push(`<text x="${margin.left}" y="36" class="title">${escapeXml(title)}</text>`);
  for (const yv of ticks(0, yMax, 6)) {
    const y = yScale(yv);
    svg.push(`<line x1="${margin.left}" y1="${y.toFixed(2)}" x2="${margin.left + plotWidth}" y2="${y.toFixed(2)}" class="grid"/>`);
    svg.push(`<text x="${margin.left - 10}" y="${(y + 4).toFixed(2)}" text-anchor="end" class="tick">${fmt(yv, 2)}</text>`);
  }
  svg.push(`<line x1="${margin.left}" y1="${margin.top + plotHeight}" x2="${margin.left + plotWidth}" y2="${margin.top + plotHeight}" class="axis"/>`);
  svg.push(`<line x1="${margin.left}" y1="${margin.top}" x2="${margin.left}" y2="${margin.top + plotHeight}" class="axis"/>`);
  svg.push(`<text transform="translate(24 ${margin.top + plotHeight / 2}) rotate(-90)" text-anchor="middle" class="label">Metric value</text>`);

  groups.forEach((g, gi) => {
    const gx = margin.left + gi * (groupWidth + groupGap);
    rows.forEach((row, ri) => {
      const value = g.value(row);
      const x = gx + ri * (barWidth + barGap);
      const y = yScale(value);
      const h = margin.top + plotHeight - y;
      svg.push(`<rect x="${x.toFixed(2)}" y="${y.toFixed(2)}" width="${barWidth.toFixed(2)}" height="${h.toFixed(2)}" fill="${colors[ri]}"/>`);
      svg.push(`<text x="${(x + barWidth / 2).toFixed(2)}" y="${(y - 5).toFixed(2)}" text-anchor="middle" class="tick">${fmt(value, g.digits ?? 3)}</text>`);
    });
    svg.push(`<text x="${(gx + groupWidth / 2).toFixed(2)}" y="${height - 76}" text-anchor="middle" class="label">${escapeXml(g.label)}</text>`);
  });

  let legendX = margin.left;
  rows.forEach((row, ri) => {
    svg.push(`<rect x="${legendX}" y="${height - 42}" width="16" height="16" fill="${colors[ri]}"/>`);
    svg.push(`<text x="${legendX + 23}" y="${height - 29}" class="legend">${escapeXml(modeLabels[row.mode] || row.mode)}</text>`);
    legendX += 260;
  });
  svg.push(`</svg>`);
  fs.writeFileSync(path.join(outDir, filename), svg.join("\n"), "utf8");
}

drawLineFigure("right_knee_reference_timeseries.svg", "Right Knee Reference Kinematics", [
  { yLabel: "q_ref (rad)", value: (r) => r.qRef, tickDigits: 3 },
  { yLabel: "dq_ref (rad/s)", value: (r) => r.dqRef, tickDigits: 3 },
  { yLabel: "ddq_ref (rad/s^2)", value: (r) => r.ddqRef, tickDigits: 2 },
  { yLabel: "jerk_ref (rad/s^3)", value: (r) => r.jerkRef, tickDigits: 1, padFrac: 0.04 },
]);

drawLineFigure("right_knee_tracking_timeseries.svg", "Right Knee Closed-loop Tracking", [
  { yLabel: "q_ref (rad)", value: (r) => r.qRef, tickDigits: 3 },
  { yLabel: "q_actual (rad)", value: (r) => r.qActual, tickDigits: 3 },
  { yLabel: "q error (rad)", value: (r) => r.qErr, tickDigits: 3 },
  { yLabel: "tau_cmd (N m)", value: (r) => r.tau, tickDigits: 1, padFrac: 0.04 },
]);

const modeLabels = {
  position_only_open_loop: "Open-loop quintic",
  ruckig_position_only: "Ruckig",
  rl_smoothed: "RL-smoothed quintic",
};

drawGroupedBars(
  "right_knee_reference_metric_bars.svg",
  "Reference Smoothness Metrics",
  [
    { label: "max |dq_ref|", value: (r) => r.max_abs_dq_ref, digits: 3 },
    { label: "max |ddq_ref|", value: (r) => r.max_abs_ddq_ref, digits: 3 },
    { label: "dq_ref RMS", value: (r) => r.dq_ref_rms, digits: 3 },
  ],
  referenceMetrics,
  modeLabels
);

drawGroupedBars(
  "right_knee_tracking_metric_bars.svg",
  "Closed-loop Tracking Metrics",
  [
    { label: "q RMSE", value: (r) => r.q_rmse, digits: 4 },
    { label: "dq RMSE", value: (r) => r.dq_rmse, digits: 4 },
    { label: "mean |tau|", value: (r) => r.tau_abs_mean, digits: 3 },
  ],
  trackingMetrics,
  modeLabels
);

console.log(`Wrote figures to ${path.relative(repoRoot, outDir)}`);
