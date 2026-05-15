#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace h1if {

constexpr int kMaxMotors = 20;
constexpr int kDebugSize = 128;

struct JointState {
    double q = 0.0;
    double dq = 0.0;
    double tau_est = 0.0;
};

struct ImuState {
    std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> gyro{0.0, 0.0, 0.0};
    std::array<double, 3> acc{0.0, 0.0, 0.0};
};

struct RobotState {
    std::uint64_t cycle = 0;
    double t = 0.0;
    double dt = 0.002;
    double lowstate_age = 0.0;
    std::array<JointState, kMaxMotors> joint;
    ImuState imu;
    bool state_valid = false;
};

struct JointCommand {
    float q = 0.0f;
    float dq = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float tau = 0.0f;
    std::uint8_t mode = 0x01;
    bool enable = true;
};

struct RobotCommand {
    std::array<JointCommand, kMaxMotors> joint;
};

struct JointDebug {
    std::array<double, 32> data{};
    std::uint32_t flags = 0;
};

struct ControllerDebug {
    std::array<double, kDebugSize> data{};
    std::array<JointDebug, kMaxMotors> joint{};
    std::uint32_t flags = 0;
};

class IController {
public:
    virtual ~IController() = default;
    virtual std::string name() const = 0;
    virtual void reset(const RobotState& state) = 0;
    virtual void step(const RobotState& state, RobotCommand& command, ControllerDebug& debug) = 0;
};

}  // namespace h1if
