#pragma once

#include "controller_interface.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

namespace h1if {

struct LogSample {
    std::uint64_t cycle = 0;
    double t = 0.0;
    double dt = 0.0;
    double lowstate_age = 0.0;
    int joint_id = 0;
    JointState measured;
    JointCommand command;
    std::array<double, 32> debug{};
    std::uint32_t flags = 0;
};

template <std::size_t Capacity = 8192>
class AsyncCsvLogger {
public:
    AsyncCsvLogger() = default;
    AsyncCsvLogger(const AsyncCsvLogger&) = delete;
    AsyncCsvLogger& operator=(const AsyncCsvLogger&) = delete;

    ~AsyncCsvLogger() {
        stop();
    }

    bool start(const std::string& path) {
        out_.open(path, std::ios::out | std::ios::trunc);
        if (!out_) {
            return false;
        }
        writeHeader();
        running_.store(true, std::memory_order_release);
        worker_ = std::thread([this]() { writerLoop(); });
        return true;
    }

    void stop() {
        running_.store(false, std::memory_order_release);
        if (worker_.joinable()) {
            worker_.join();
        }
        drain();
        if (out_) {
            out_.flush();
            out_.close();
        }
    }

    bool push(const LogSample& sample) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = increment(head);
        if (next == tail_.load(std::memory_order_acquire)) {
            drops_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        (*buffer_)[head] = sample;
        head_.store(next, std::memory_order_release);
        return true;
    }

    std::uint64_t drops() const {
        return drops_.load(std::memory_order_relaxed);
    }

private:
    static std::size_t increment(std::size_t value) {
        return (value + 1) % Capacity;
    }

    bool pop(LogSample& sample) {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        sample = (*buffer_)[tail];
        tail_.store(increment(tail), std::memory_order_release);
        return true;
    }

    void writerLoop() {
        while (running_.load(std::memory_order_acquire)) {
            drain();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void drain() {
        LogSample sample;
        while (pop(sample)) {
            writeSample(sample);
        }
        if (out_) {
            out_.flush();
        }
    }

    void writeHeader() {
        out_ << "cycle,t,dt,lowstate_age,joint_id,q,dq,tau_est,"
             << "q_cmd,dq_cmd,kp_cmd,kd_cmd,tau_cmd,flags";
        for (int i = 0; i < static_cast<int>(LogSample{}.debug.size()); ++i) {
            out_ << ",debug_" << i;
        }
        out_ << "\n";
    }

    void writeSample(const LogSample& s) {
        out_ << s.cycle << ','
             << s.t << ','
             << s.dt << ','
             << s.lowstate_age << ','
             << s.joint_id << ','
             << s.measured.q << ','
             << s.measured.dq << ','
             << s.measured.tau_est << ','
             << s.command.q << ','
             << s.command.dq << ','
             << s.command.kp << ','
             << s.command.kd << ','
             << s.command.tau << ','
             << s.flags;
        for (double value : s.debug) {
            out_ << ',' << value;
        }
        out_ << "\n";
    }

    std::unique_ptr<std::array<LogSample, Capacity>> buffer_{
        std::make_unique<std::array<LogSample, Capacity>>()};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> drops_{0};
    std::thread worker_;
    std::ofstream out_;
};

}  // namespace h1if
