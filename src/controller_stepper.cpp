#include "controller_factory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

h1if::RobotState parseStateLine(const std::string& line) {
    std::istringstream in(line);
    std::string tag;
    h1if::RobotState state;
    if (!(in >> tag) || tag != "state") {
        throw std::runtime_error("expected state line");
    }
    if (!(in >> state.cycle >> state.t >> state.dt >> state.lowstate_age)) {
        throw std::runtime_error("state line missing header fields");
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        if (!(in >> state.joint[i].q >> state.joint[i].dq >> state.joint[i].tau_est)) {
            throw std::runtime_error("state line missing joint fields");
        }
    }
    state.state_valid = true;
    return state;
}

void writeReady(const std::vector<int>& active_joints) {
    std::cout << "ready";
    for (int joint_id : active_joints) {
        std::cout << ' ' << joint_id;
    }
    std::cout << '\n' << std::flush;
}

void writeCommand(const h1if::RobotCommand& command, const h1if::ControllerDebug& debug) {
    std::cout << std::setprecision(17) << "cmd " << debug.flags;
    // Motor commands (5 x 20 = 100 values)
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].q;
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].dq;
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].tau;
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].kp;
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].kd;
    }
    // Full joint debug data: 32 doubles x 20 joints = 640 values
    for (int slot = 0; slot < 32; ++slot) {
        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            std::cout << ' ' << debug.joint[i].data[slot];
        }
    }
    // Joint safety flags (20 values)
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].flags;
    }
    std::cout << '\n' << std::flush;
}

void writeReference(const h1if::ControllerDebug& debug) {
    std::cout << std::setprecision(17) << "ref " << debug.flags;
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].data[0];
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].data[1];
    }
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].flags;
    }
    std::cout << '\n' << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage:\n  " << argv[0] << " <config.yaml>\n";
        return 2;
    }

    try {
        h1if::RuntimeConfig cfg = h1if::loadRuntimeConfig(argv[1]);
        const bool reference_only = argc >= 3 && std::string(argv[2]) == "--reference-only";
        const std::vector<int> active_joints = h1if::activeControllerJoints(cfg);
        const auto controller = h1if::createController(cfg);
        bool initialized = false;

        writeReady(active_joints);

        std::string line;
        while (std::getline(std::cin, line)) {
            if (line.empty()) {
                continue;
            }
            if (line == "quit") {
                return 0;
            }

            h1if::RobotState state = parseStateLine(line);
            h1if::RobotCommand command;
            h1if::ControllerDebug debug;
            if (!initialized) {
                controller->reset(state);
                initialized = true;
            }
            controller->step(state, command, debug);
            if (reference_only) {
                writeReference(debug);
            } else {
                h1if::applySafety(state, command, debug, cfg.safety);
                writeCommand(command, debug);
            }
        }
        return 0;
    } catch (const std::exception& ex) {
            std::cerr << "h1_controller_stepper failed: " << ex.what() << "\n";
        return 1;
    }
}
