#!/usr/bin/env python3
"""Run online LSTM policy + EID landing test with 80 Nm knee cap and 0.20 rad jump trip."""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import experiments.landing_stand.run_real_landing_stand_online_policy_lstm_eid_ku_ko08 as base

base.DEFAULT_CONFIG = (
    base.REPO_ROOT
    / "experiments" / "landing_stand" / "configs" / "h1_real_landing_stand_online_policy_lstm_eid_ku_ko08_knee80_jump020.yaml"
)
base.DEFAULT_OUT_DIR = (
    base.REPO_ROOT
    / "analysis_artifacts"
    / "real_landing_stand_online_policy_lstm_eid_ku_ko08_knee80_jump020"
)
base.DEFAULT_CONDITION = "landing_stand_online_policy_lstm_preview_hold3_eid_ku_ko08_knee80_jump020"
base.CONFIRM_TOKEN = "RUN_ONLINE_POLICY_LANDING_STAND_KNEE80_JUMP020"


if __name__ == "__main__":
    print("Knee80 jump020 landing variant: knees cap at 80 Nm; measured_jump_trip is 0.20 rad.")
    print("Speed trips and joint angle limits remain enabled.")
    base.main()
