#pragma once

#include "ModbusProtocol.hpp"
#include "TcpSocket.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace sim {

/// Holding register map of the virtual device. Values are transmitted as scaled
/// unsigned integers, which is how most real PLCs and field devices expose analog data.
namespace registers {
constexpr std::uint16_t kTemperature = 0;  ///< Process temperature, 0.1 degC per bit.
constexpr std::uint16_t kPressure = 1;     ///< Line pressure, 0.01 bar per bit.
constexpr std::uint16_t kMotorSpeed = 2;   ///< Pump motor speed, 1 rpm per bit.
constexpr std::uint16_t kVibration = 3;    ///< Bearing vibration velocity, 0.01 mm/s per bit.
constexpr std::uint16_t kTankLevel = 4;    ///< Buffer tank level, 0.1 % per bit.
constexpr std::uint16_t kHeartbeat = 5;    ///< Incremented on every simulation step.
constexpr std::uint16_t kCount = 16;       ///< Size of the register bank; unused entries stay 0.
}  // namespace registers

struct ServerConfig {
    std::string bindAddress{"127.0.0.1"};
    std::uint16_t port{1502};  ///< 0 lets the operating system choose a free port.
    std::uint8_t unitId{1};
    std::chrono::milliseconds simulationStep{200};
    bool enableSimulation{true};       ///< Disable to keep registers fully under test control.
    std::uint32_t randomSeed{0};       ///< 0 selects a non-deterministic seed.
    double excursionProbability{0.008};  ///< Chance per step that a signal starts a process upset.
};

struct ServerStatistics {
    std::uint64_t connectionsAccepted{0};
    std::uint64_t requestsServed{0};
    std::uint64_t exceptionsReturned{0};
};

/// A self-contained Modbus TCP slave that runs on background threads.
///
/// One thread owns the listening socket and answers "Read Holding Registers"
/// requests; a second thread drives a simple process model that produces
/// realistic, slowly drifting values with occasional upsets. Both threads are
/// joined in stop() and in the destructor, so the object never outlives its work.
class VirtualModbusServer {
public:
    explicit VirtualModbusServer(ServerConfig config);
    ~VirtualModbusServer();

    VirtualModbusServer(const VirtualModbusServer&) = delete;
    VirtualModbusServer& operator=(const VirtualModbusServer&) = delete;
    VirtualModbusServer(VirtualModbusServer&&) = delete;
    VirtualModbusServer& operator=(VirtualModbusServer&&) = delete;

    void start();
    void stop() noexcept;

    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] std::uint16_t port() const noexcept;
    [[nodiscard]] const ServerConfig& config() const noexcept;
    [[nodiscard]] ServerStatistics statistics() const noexcept;
    [[nodiscard]] std::vector<std::uint16_t> registerSnapshot() const;

    /// Overwrites a single register. Useful for fault injection and tests.
    void writeRegister(std::uint16_t address, std::uint16_t value);

private:
    struct SignalModel {
        std::uint16_t address;
        double baseline;
        double amplitude;       ///< Peak deviation of the slow periodic drift.
        double periodSeconds;   ///< Period of the drift.
        double noiseStdDev;     ///< Measurement noise.
        double excursionMagnitude;
        bool bidirectional;     ///< Whether upsets may push the value downwards.
        double minValue;
        double maxValue;
        double scale;           ///< Engineering value * scale = raw register value.
    };

    struct SignalState {
        SignalModel model;
        double phase{0.0};
        double excursionOffset{0.0};
        double excursionTarget{0.0};
        int excursionStepsLeft{0};
    };

    void networkLoop() noexcept;
    void simulationLoop() noexcept;
    void serveClient(const net::TcpSocket& client);
    void stepSimulation(double elapsedSeconds);

    [[nodiscard]] std::vector<std::uint8_t> handleRequest(const modbus::MbapHeader& header,
                                                          const std::vector<std::uint8_t>& pdu);
    [[nodiscard]] bool acceptsUnit(std::uint8_t unitId) const noexcept;

    const ServerConfig config_;

    // Declared before any socket so that Winsock is torn down last.
    net::NetworkRuntime networkRuntime_;
    net::TcpSocket listener_;
    std::uint16_t boundPort_{0};

    mutable std::mutex registerMutex_;
    std::array<std::uint16_t, registers::kCount> registers_{};

    // Touched only by the simulation thread once it is running.
    std::vector<SignalState> signals_;
    std::mt19937 rng_;

    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> connectionsAccepted_{0};
    std::atomic<std::uint64_t> requestsServed_{0};
    std::atomic<std::uint64_t> exceptionsReturned_{0};

    std::mutex wakeMutex_;
    std::condition_variable wakeCondition_;

    std::thread networkThread_;
    std::thread simulationThread_;
};

}  // namespace sim
