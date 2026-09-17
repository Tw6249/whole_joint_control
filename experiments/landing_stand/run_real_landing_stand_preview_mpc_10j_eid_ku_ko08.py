#!/usr/bin/env python3
"""Run the Ko=(0.8, 0.2) constant-stand landing test."""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import sys
from pathlib import Path

import experiments.landing_stand.run_real_landing_stand_preview_mpc_10j_eid_ku as base

REPO_ROOT = Path(__file__).resolve().parents[2]
KO08_CONFIG = REPO_ROOT / "experiments" / "landing_stand" / "configs" / "h1_real_landing_stand_preview_mpc_10j_eid_ku_ko08.yaml"
KO08_CONDITION = "landing_stand_const_preview_mpc_10j_eid_ku_ko08"
KO08_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_landing_stand_preview_mpc_10j_eid_ku_ko08"


def main() -> None:
    args = sys.argv[1:]
    if "--config" not in args:
        args = ["--config", str(KO08_CONFIG), *args]
    if "--condition" not in args:
        args = ["--condition", KO08_CONDITION, *args]
    if "--out-dir" not in args:
        args = ["--out-dir", str(KO08_OUT_DIR), *args]
    sys.argv = [sys.argv[0], *args]
    base.main()


if __name__ == "__main__":
    main()
