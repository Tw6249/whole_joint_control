#!/usr/bin/env python3
"""Run online LSTM policy + EID landing test with 80 Nm knee torque cap.

The controller is the same online-policy EID stack as the Ko=(0.8,0.2) landing
test. This wrapper selects a knee-focused configuration that raises knee
feedback response and caps each knee command at 80 Nm.
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
    / "experiments" / "landing_stand" / "configs" / "h1_real_landing_stand_online_policy_lstm_eid_ku_ko08_knee80.yaml"
)
base.DEFAULT_OUT_DIR = (
    base.REPO_ROOT
    / "analysis_artifacts"
    / "real_landing_stand_online_policy_lstm_eid_ku_ko08_knee80"
)
base.DEFAULT_CONDITION = "landing_stand_online_policy_lstm_preview_hold3_eid_ku_ko08_knee80"
base.CONFIRM_TOKEN = "RUN_ONLINE_POLICY_LANDING_STAND_KNEE80"


if __name__ == "__main__":
    print("Knee80 landing variant: knees use Kp/Kd=(240,10), tau_limit=80 Nm, tau_slew_rate=400 Nm/s.")
    print("This does not apply a fixed 80 Nm torque; it allows EID to reach 80 Nm when tracking/contact error demands it.")
    base.main()
