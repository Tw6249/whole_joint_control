#!/usr/bin/env python3
"""Run high-torque-limit online LSTM policy + EID landing stand test.

This wrapper keeps the online policy/EID controller unchanged and selects the
max-torque-limit landing configuration.
"""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import experiments.landing_stand.run_real_landing_stand_online_policy_lstm_eid_ku_ko08 as base

base.DEFAULT_CONFIG = (
    base.REPO_ROOT
    / "experiments" / "landing_stand" / "configs" / "h1_real_landing_stand_online_policy_lstm_eid_ku_ko08_max_tau.yaml"
)
base.DEFAULT_OUT_DIR = (
    base.REPO_ROOT
    / "analysis_artifacts"
    / "real_landing_stand_online_policy_lstm_eid_ku_ko08_max_tau"
)
base.DEFAULT_CONDITION = "landing_stand_online_policy_lstm_preview_hold3_eid_ku_ko08_max_tau"
base.CONFIRM_TOKEN = "RUN_ONLINE_POLICY_LANDING_STAND_MAX_TAU"


if __name__ == "__main__":
    print("High-torque landing variant: controller tau_limit equals each active joint plant.tau_max.")
    print("Torque slew-rate limits are unchanged from the previous online-policy landing config.")
    base.main()
