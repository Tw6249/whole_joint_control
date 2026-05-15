#include "async_csv_logger.hpp"
#include "eid_controller.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

namespace {

struct PlantState {
    double q = 0.9;
    double dq = 0.0;
    double tau_est = 0.0;
};

double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

double gravityTorque(const h1if::PlantModelConfig& model, double q) {
    return model.gravityA * std::sin(q) + model.gravityB * std::cos(q);
}

double initialMockPosition(const h1if::JointEidConfig& joint_cfg) {
    const auto& controller = joint_cfg.controller;
    const auto& plant = joint_cfg.plant;
    double q0 = controller.ref_center;
    if (controller.reference_signal == h1if::ReferenceSignal::Sine) {
        q0 = controller.ref_center + controller.ref_amplitude * std::sin(controller.ref_phase);
    } else if (controller.ref_step_time <= 0.0) {
        q0 = controller.ref_center + controller.ref_amplitude;
    }
    return clamp(q0, plant.q_min, plant.q_max);
}

void stepPlant(const h1if::PlantModelConfig& model, const h1if::JointCommand& cmd, double dt, PlantState& plant) {
    const double tau_raw =
        static_cast<double>(cmd.kp) * (static_cast<double>(cmd.q) - plant.q) +
        static_cast<double>(cmd.kd) * (static_cast<double>(cmd.dq) - plant.dq) +
        static_cast<double>(cmd.tau);

    const double tau = clamp(tau_raw, -model.tau_max, model.tau_max);
    const double qacc = (tau - model.b * plant.dq - gravityTorque(model, plant.q) - model.tau0) / model.Jeff;
    plant.dq += dt * qacc;
    plant.q += dt * plant.dq;

    if (plant.q < model.q_min) {
        plant.q = model.q_min;
        if (plant.dq < 0.0) plant.dq = 0.0;
    } else if (plant.q > model.q_max) {
        plant.q = model.q_max;
        if (plant.dq > 0.0) plant.dq = 0.0;
    }

    plant.tau_est = tau;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string config_path = argc >= 2 ? argv[1] : "config/h1_full_body_mujoco_fit.yaml";
        h1if::RuntimeConfig cfg = h1if::loadRuntimeConfig(config_path);
        if (argc >= 3) {
            cfg.mock_duration = std::atof(argv[2]);
        }
        cfg.log_path = h1if::resolveLogPath(cfg);

        const std::vector<int> active_joints = h1if::activeEidJoints(cfg);
        const double dt = cfg.control_dt;
        const int steps = static_cast<int>(std::ceil(cfg.mock_duration / dt));

        std::array<PlantState, h1if::kMaxMotors> plants{};
        for (int j : active_joints) {
            plants[j] = PlantState{initialMockPosition(*cfg.eid_controllers[j]), 0.0, 0.0};
        }

        h1if::RobotState state;
        state.dt = dt;
        state.state_valid = true;
        state.lowstate_age = 0.0;
        for (int j : active_joints) {
            state.joint[j].q = plants[j].q;
            state.joint[j].dq = plants[j].dq;
            state.joint[j].tau_est = plants[j].tau_est;
        }

        h1if::EidMultiJointController controller(cfg);
        controller.reset(state);

        h1if::AsyncCsvLogger<> logger;
        if (!logger.start(cfg.log_path)) {
            std::cerr << "Warning: cannot open log file: " << cfg.log_path << "\n";
        }

        double q_error_energy = 0.0;
        double max_abs_tau = 0.0;
        std::uint32_t combined_flags = 0;

        for (int k = 0; k < steps; ++k) {
            state.cycle = static_cast<std::uint64_t>(k);
            state.t = k * dt;
            state.dt = dt;
            state.lowstate_age = 0.0;
            state.state_valid = true;
            for (int j : active_joints) {
                state.joint[j].q = plants[j].q;
                state.joint[j].dq = plants[j].dq;
                state.joint[j].tau_est = plants[j].tau_est;
            }

            h1if::RobotCommand command;
            h1if::ControllerDebug debug;
            controller.step(state, command, debug);
            h1if::applySafety(state, command, debug, cfg.safety);

            combined_flags |= debug.flags;

            for (int j : active_joints) {
                const auto joint_command = command.joint[j];
                stepPlant(cfg.eid_controllers[j]->plant, joint_command, dt, plants[j]);

                const double q_ref = debug.joint[j].data[0];
                const double err = q_ref - state.joint[j].q;
                q_error_energy += err * err;
                max_abs_tau = std::max(max_abs_tau, std::abs(static_cast<double>(joint_command.tau)));
                combined_flags |= debug.joint[j].flags;

                h1if::LogSample sample;
                sample.cycle = state.cycle;
                sample.t = state.t;
                sample.dt = dt;
                sample.lowstate_age = state.lowstate_age;
                sample.joint_id = j;
                sample.measured = state.joint[j];
                sample.command = joint_command;
                sample.flags = debug.flags | debug.joint[j].flags;
                for (int i = 0; i < static_cast<int>(sample.debug.size()); ++i) {
                    sample.debug[i] = debug.joint[j].data[i];
                }
                logger.push(sample);
            }
        }

        logger.stop();

        const double q_rmse =
            std::sqrt(q_error_energy / std::max(1, steps * static_cast<int>(active_joints.size())));
        std::cout << "mock_closed_loop finished\n"
                  << "  samples=" << steps << "\n"
                  << "  active_joints=" << active_joints.size() << "\n"
                  << "  dt=" << dt << "\n"
                  << "  q_rmse=" << q_rmse << "\n"
                  << "  max_abs_tau=" << max_abs_tau << "\n"
                  << "  safety_flags=0x" << std::hex << combined_flags << std::dec << "\n"
                  << "  log_drops=" << logger.drops() << "\n"
                  << "  log_path=" << cfg.log_path << "\n";

        if (!std::isfinite(q_rmse) || !std::isfinite(max_abs_tau)) {
            return 2;
        }
        if (combined_flags & (h1if::kSafetyInvalidState | h1if::kSafetyLowStateTimeout | h1if::kSafetyNonFiniteCommand)) {
            return 3;
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "mock_closed_loop failed: " << ex.what() << "\n";
        return 1;
    }
}
