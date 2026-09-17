#!/usr/bin/env python3
"""Summarize pure MPC optimization solve time from h1_direct CSV logs."""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import csv
import math
from pathlib import Path


KIND_NAME = {
    1: "preview_mpc",
    2: "preview_mpc_velocity",
}


def finite_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        out = float(value)
    except ValueError:
        return math.nan
    return out if math.isfinite(out) else math.nan


def percentile(values: list[float], p: float) -> float:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * p
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return clean[lo]
    alpha = rank - lo
    return (1.0 - alpha) * clean[lo] + alpha * clean[hi]


def mean(values: list[float]) -> float:
    clean = [v for v in values if math.isfinite(v)]
    return sum(clean) / len(clean) if clean else math.nan


def fmt_ms(value_s: float) -> str:
    return "nan" if not math.isfinite(value_s) else f"{1.0e3 * value_s:.4f}"


def load_solves(path: Path, drop_start: float) -> dict[tuple[int, int], dict[str, object]]:
    rows_by_key: dict[tuple[int, int], dict[str, object]] = {}
    first_t = math.nan
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if "mpc_solve_s" not in (reader.fieldnames or []):
            raise RuntimeError(f"{path} does not contain mpc_solve_s; rebuild h1_direct and rerun the log")
        for row in reader:
            t = finite_float(row.get("t"))
            if not math.isfinite(first_t) and math.isfinite(t):
                first_t = t
            if drop_start > 0.0 and math.isfinite(t) and math.isfinite(first_t) and t - first_t < drop_start:
                continue
            try:
                cycle = int(row.get("cycle", ""))
                joint_id = int(row.get("joint_id", ""))
                ran = int(float(row.get("mpc_solve_ran", "0") or "0"))
                success = int(float(row.get("mpc_solve_success", "0") or "0"))
                kind = int(float(row.get("mpc_solve_kind", "0") or "0"))
            except ValueError:
                continue
            solve_s = finite_float(row.get("mpc_solve_s"))
            if ran <= 0 or not math.isfinite(solve_s):
                continue
            rows_by_key[(cycle, joint_id)] = {
                "cycle": cycle,
                "joint_id": joint_id,
                "solve_s": solve_s,
                "success": success,
                "kind": kind,
            }
    return rows_by_key


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--drop-start", type=float, default=0.0)
    parser.add_argument("--deadline-ms", type=float, default=2.0)
    args = parser.parse_args()

    deadline_s = args.deadline_ms * 1.0e-3
    for path in args.logs:
        solves = list(load_solves(path, args.drop_start).values())
        values = [float(r["solve_s"]) for r in solves]
        failures = [r for r in solves if int(r["success"]) == 0]
        over = [v for v in values if v > deadline_s]
        kinds = sorted({int(r["kind"]) for r in solves})
        kind_label = ",".join(KIND_NAME.get(k, str(k)) for k in kinds) if kinds else "none"

        print(f"\n{path}")
        print(f"mpc_kind={kind_label} solves={len(values)} failures={len(failures)} deadline_ms={args.deadline_ms:g}")
        print(
            "mpc_solve_ms "
            f"mean={fmt_ms(mean(values))} "
            f"p95={fmt_ms(percentile(values, 0.95))} "
            f"p99={fmt_ms(percentile(values, 0.99))} "
            f"max={fmt_ms(max(values) if values else math.nan)} "
            f"over_deadline={len(over)}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
