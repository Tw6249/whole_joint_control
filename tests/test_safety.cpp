#include "safety.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {

h1if::RobotState validState() {
    h1if::RobotState state;
    state.state_valid = true;
    state.lowstate_age = 0.001;
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        state.joint[i].q = 0.5;
        state.joint[i].dq = 0.0;
        state.joint[i].tau_est = 0.0;
    }
    return state;
}

}  // namespace

int main() {
    h1if::SafetyConfig cfg;
    cfg.limit[2].q_min = -0.2f;
    cfg.limit[2].q_max = 2.0f;
    cfg.limit[2].dq_max = 4.0f;
    cfg.limit[2].tau_max = 80.0f;
    cfg.limit[2].kp_max = 100.0f;
    cfg.limit[2].kd_max = 10.0f;

    {
        auto state = validState();
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, cmd, cfg);
        cmd.joint[2].q = 10.0f;
        cmd.joint[2].dq = 20.0f;
        cmd.joint[2].kp = 200.0f;
        cmd.joint[2].kd = 20.0f;
        cmd.joint[2].tau = 120.0f;
        h1if::applySafety(state, cmd, debug, cfg);

        assert(cmd.joint[2].q == cfg.limit[2].q_max);
        assert(cmd.joint[2].dq == cfg.limit[2].dq_max);
        assert(cmd.joint[2].kp == cfg.limit[2].kp_max);
        assert(cmd.joint[2].kd == cfg.limit[2].kd_max);
        assert(cmd.joint[2].tau == cfg.limit[2].tau_max);
        assert((debug.flags & h1if::kSafetyCommandSaturated) != 0);
    }

    {
        auto state = validState();
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, cmd, cfg);
        cmd.joint[2].tau = std::numeric_limits<float>::quiet_NaN();
        h1if::applySafety(state, cmd, debug, cfg);

        assert((debug.flags & h1if::kSafetyNonFiniteCommand) != 0);
        assert(cmd.joint[2].q == static_cast<float>(state.joint[2].q));
        assert(cmd.joint[2].tau == 0.0f);
    }

    {
        auto state = validState();
        state.lowstate_age = 1.0;
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, cmd, cfg);
        cmd.joint[2].tau = 20.0f;
        h1if::applySafety(state, cmd, debug, cfg);

        assert((debug.flags & h1if::kSafetyLowStateTimeout) != 0);
        assert(cmd.joint[2].tau == 0.0f);
    }

    assert(h1if::h1MotorMode(2) == 0x0A);
    assert(h1if::h1MotorMode(7) == 0x0A);
    assert(h1if::h1MotorMode(9) == 0x0A);
    assert(h1if::h1MotorMode(10) == 0x0A);
    assert(h1if::h1MotorMode(11) == 0x0A);
    assert(h1if::h1MotorMode(12) == 0x01);

    std::cout << "h1_safety_tests passed\n";
    return 0;
}
