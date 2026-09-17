#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running.store(false, std::memory_order_release);
}

struct Options {
    std::string iface = "eth0";
    int domain = 0;
    std::string command;
    float vx = 0.0f;
    float vy = 0.0f;
    float vyaw = 0.0f;
    float duration = 1.0f;
    bool yes = false;
};

void printUsage(const char* argv0) {
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " [--iface IFACE] [--domain ID] status\n"
        << "  " << argv0 << " [--iface IFACE] [--domain ID] stop|damp|standup|start|balance|continuous-gait\n"
        << "  " << argv0 << " [--iface IFACE] [--domain ID] move --vx MPS --vy MPS --vyaw RADS --duration SEC --yes\n\n"
        << "Safety limits for move: |vx|<=0.20, |vy|<=0.10, |vyaw|<=0.30, 0<duration<=10.\n"
        << "This tool calls the H1 native loco service. It does not publish rt/lowcmd.\n";
}

float parseFloat(const std::string& value, const std::string& label) {
    std::size_t used = 0;
    const float parsed = std::stof(value, &used);
    if (used != value.size() || !std::isfinite(parsed)) {
        throw std::runtime_error(label + " must be finite");
    }
    return parsed;
}

int parseInt(const std::string& value, const std::string& label) {
    std::size_t used = 0;
    const int parsed = std::stoi(value, &used);
    if (used != value.size()) {
        throw std::runtime_error(label + " must be an integer");
    }
    return parsed;
}

Options parseOptions(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--iface") {
            if (++i >= argc) {
                throw std::runtime_error("missing value for --iface");
            }
            opts.iface = argv[i];
        } else if (key == "--domain") {
            if (++i >= argc) {
                throw std::runtime_error("missing value for --domain");
            }
            opts.domain = parseInt(argv[i], "--domain");
        } else if (key == "--vx") {
            if (++i >= argc) {
                throw std::runtime_error("missing value for --vx");
            }
            opts.vx = parseFloat(argv[i], "--vx");
        } else if (key == "--vy") {
            if (++i >= argc) {
                throw std::runtime_error("missing value for --vy");
            }
            opts.vy = parseFloat(argv[i], "--vy");
        } else if (key == "--vyaw") {
            if (++i >= argc) {
                throw std::runtime_error("missing value for --vyaw");
            }
            opts.vyaw = parseFloat(argv[i], "--vyaw");
        } else if (key == "--duration") {
            if (++i >= argc) {
                throw std::runtime_error("missing value for --duration");
            }
            opts.duration = parseFloat(argv[i], "--duration");
        } else if (key == "--yes") {
            opts.yes = true;
        } else if (!key.empty() && key[0] == '-') {
            throw std::runtime_error("unknown option: " + key);
        } else {
            if (!opts.command.empty()) {
                throw std::runtime_error("multiple commands provided");
            }
            opts.command = key;
        }
    }
    if (opts.command.empty()) {
        throw std::runtime_error("missing command");
    }
    return opts;
}

void validateMove(const Options& opts) {
    if (!opts.yes) {
        throw std::runtime_error("move requires --yes");
    }
    if (std::abs(opts.vx) > 0.20f) {
        throw std::runtime_error("move rejected: |vx| must be <= 0.20 m/s");
    }
    if (std::abs(opts.vy) > 0.10f) {
        throw std::runtime_error("move rejected: |vy| must be <= 0.10 m/s");
    }
    if (std::abs(opts.vyaw) > 0.30f) {
        throw std::runtime_error("move rejected: |vyaw| must be <= 0.30 rad/s");
    }
    if (!(opts.duration > 0.0f) || opts.duration > 10.0f || !std::isfinite(opts.duration)) {
        throw std::runtime_error("move rejected: duration must be in (0, 10] seconds");
    }
}

void printRet(const std::string& name, int32_t ret) {
    std::cout << name << " ret=" << ret << "\n";
}

template <typename Fn>
int32_t callSafely(const std::string& name, Fn&& fn) {
    try {
        const int32_t ret = fn();
        printRet(name, ret);
        return ret;
    } catch (const std::exception& ex) {
        std::cerr << name << " exception: " << ex.what() << "\n";
        return -1;
    }
}

int runStatus(unitree::robot::h1::LocoClient& client) {
    int fsm_id = -1;
    int fsm_mode = -1;
    int balance_mode = -1;
    float swing_height = 0.0f;
    float stand_height = 0.0f;
    int failed = 0;

    failed += callSafely("GetFsmId", [&]() { return client.GetFsmId(fsm_id); }) == 0 ? 0 : 1;
    failed += callSafely("GetFsmMode", [&]() { return client.GetFsmMode(fsm_mode); }) == 0 ? 0 : 1;
    failed += callSafely("GetBalanceMode", [&]() { return client.GetBalanceMode(balance_mode); }) == 0 ? 0 : 1;
    failed += callSafely("GetSwingHeight", [&]() { return client.GetSwingHeight(swing_height); }) == 0 ? 0 : 1;
    failed += callSafely("GetStandHeight", [&]() { return client.GetStandHeight(stand_height); }) == 0 ? 0 : 1;

    std::cout << std::fixed << std::setprecision(4)
              << "fsm_id=" << fsm_id
              << " fsm_mode=" << fsm_mode
              << " balance_mode=" << balance_mode
              << " swing_height=" << swing_height
              << " stand_height=" << stand_height << "\n";
    if (failed != 0) {
        std::cerr << "Status query failed. The H1 loco service may be unavailable on this iface/domain, "
                  << "or the robot is not running the native locomotion service.\n";
    }
    return failed == 0 ? 0 : 2;
}

int runMove(unitree::robot::h1::LocoClient& client, const Options& opts) {
    validateMove(opts);
    std::cout << std::fixed << std::setprecision(3)
              << "Move vx=" << opts.vx
              << " vy=" << opts.vy
              << " vyaw=" << opts.vyaw
              << " duration=" << opts.duration << " s\n";

    const int32_t ret = callSafely("SetVelocity", [&]() {
        return client.SetVelocity(opts.vx, opts.vy, opts.vyaw, opts.duration);
    });
    if (ret != 0) {
        return ret;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opts.duration);
    while (g_running.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    callSafely("StopMove", [&]() { return client.StopMove(); });
    return ret;
}

int runCommand(unitree::robot::h1::LocoClient& client, const Options& opts) {
    if (opts.command == "status") {
        return runStatus(client);
    }
    if (opts.command == "stop") {
        return callSafely("StopMove", [&]() { return client.StopMove(); }) == 0 ? 0 : 2;
    }
    if (opts.command == "damp") {
        if (!opts.yes) {
            throw std::runtime_error("damp requires --yes");
        }
        return callSafely("Damp", [&]() { return client.Damp(); }) == 0 ? 0 : 2;
    }
    if (opts.command == "standup") {
        if (!opts.yes) {
            throw std::runtime_error("standup requires --yes");
        }
        return callSafely("StandUp", [&]() { return client.StandUp(); }) == 0 ? 0 : 2;
    }
    if (opts.command == "start") {
        if (!opts.yes) {
            throw std::runtime_error("start requires --yes");
        }
        return callSafely("Start", [&]() { return client.Start(); }) == 0 ? 0 : 2;
    }
    if (opts.command == "balance") {
        if (!opts.yes) {
            throw std::runtime_error("balance requires --yes");
        }
        return callSafely("BalanceStand", [&]() { return client.BalanceStand(); }) == 0 ? 0 : 2;
    }
    if (opts.command == "continuous-gait") {
        if (!opts.yes) {
            throw std::runtime_error("continuous-gait requires --yes");
        }
        return callSafely("ContinuousGait", [&]() { return client.ContinuousGait(true); }) == 0 ? 0 : 2;
    }
    if (opts.command == "move") {
        return runMove(client, opts);
    }
    throw std::runtime_error("unknown command: " + opts.command);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        const Options opts = parseOptions(argc, argv);

        unitree::robot::ChannelFactory::Instance()->Init(opts.domain, opts.iface);

        unitree::robot::h1::LocoClient client;
        client.SetTimeout(5.0f);
        client.Init();

        std::cout << "H1 native loco client: iface=" << opts.iface
                  << " domain=" << opts.domain
                  << " command=" << opts.command << "\n";
        const int ret = runCommand(client, opts);

        unitree::robot::ChannelFactory::Instance()->Release();
        return ret == 0 ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "h1_loco_cli failed: " << ex.what() << "\n";
        unitree::robot::ChannelFactory::Instance()->Release();
        printUsage(argv[0]);
        return 2;
    }
}
