#pragma once

#include "controller_factory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace h1if {

using ControllerFactoryFn = std::unique_ptr<IController> (*)(const RuntimeConfig&);

inline RobotState parseStepperStateLine(const std::string& line) {
    std::istringstream in(line);
    std::string tag;
    RobotState state;
    if (!(in >> tag) || tag != "state") {
        throw std::runtime_error("expected state line");
    }
    if (!(in >> state.cycle >> state.t >> state.dt >> state.lowstate_age)) {
        throw std::runtime_error("state line missing header fields");
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        if (!(in >> state.joint[i].q >> state.joint[i].dq >> state.joint[i].tau_est)) {
            throw std::runtime_error("state line missing joint fields");
        }
    }
    state.state_valid = true;
    return state;
}

inline void writeStepperReady(const std::vector<int>& active_joints) {
    std::cout << "ready";
    for (int joint_id : active_joints) {
        std::cout << ' ' << joint_id;
    }
    std::cout << '\n' << std::flush;
}

inline void writeStepperCommand(const RobotCommand& command, const ControllerDebug& debug) {
    std::cout << std::setprecision(17) << "cmd " << debug.flags;
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].q;
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].dq;
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].tau;
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].kp;
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << command.joint[i].kd;
    }
    for (int slot = 0; slot < kJointDebugSize; ++slot) {
        for (int i = 0; i < kMaxMotors; ++i) {
            std::cout << ' ' << debug.joint[i].data[slot];
        }
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].flags;
    }
    std::cout << '\n' << std::flush;
}

inline void writeStepperReference(const ControllerDebug& debug) {
    std::cout << std::setprecision(17) << "ref " << debug.flags;
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].data[0];
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].data[1];
    }
    for (int i = 0; i < kMaxMotors; ++i) {
        std::cout << ' ' << debug.joint[i].flags;
    }
    std::cout << '\n' << std::flush;
}

inline int runControllerStepper(
    int argc,
    char** argv,
    ControllerFactoryFn make_controller,
    const char* program_name) {
    if (argc < 2) {
        std::cerr << "Usage:\n  " << argv[0] << " <config.yaml>\n";
        return 2;
    }

    try {
        RuntimeConfig cfg = loadRuntimeConfig(argv[1]);
        const bool reference_only = argc >= 3 && std::string(argv[2]) == "--reference-only";
        const std::vector<int> active_joints = activeControllerJoints(cfg);
        const auto controller = make_controller(cfg);
        bool initialized = false;

        writeStepperReady(active_joints);

        std::string line;
        while (std::getline(std::cin, line)) {
            if (line.empty()) {
                continue;
            }
            if (line == "quit") {
                return 0;
            }

            RobotState state = parseStepperStateLine(line);
            RobotCommand command;
            ControllerDebug debug;
            if (!initialized) {
                controller->reset(state);
                initialized = true;
            }
            controller->step(state, command, debug);
            if (reference_only) {
                writeStepperReference(debug);
            } else {
                applySafety(state, command, debug, cfg.safety);
                writeStepperCommand(command, debug);
            }
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << program_name << " failed: " << ex.what() << "\n";
        return 1;
    }
}

}  // namespace h1if
