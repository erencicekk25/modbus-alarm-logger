#pragma once

#include "ModbusProtocol.hpp"
#include "TcpSocket.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace monitor {

/// Ordered so that a higher value always means a more critical state.
enum class Severity : std::uint8_t {
    Normal = 0,
    Warning = 1,
    Alarm = 2,
};

[[nodiscard]] const char* toString(Severity severity) noexcept;

/// Four-level limit set in engineering units. Any limit may be omitted.
///
/// A limit is *entered* when the value crosses it, but it is only *left* once the
/// value has moved back by `hysteresis`. This dead band prevents an alarm from
/// chattering on and off while a noisy signal hovers around the limit.
struct ThresholdRule {
    std::optional<double> lowAlarm;
    std::optional<double> lowWarning;
    std::optional<double> highWarning;
    std::optional<double> highAlarm;
    double hysteresis{0.0};
};

/// Pure state transition function used by the client; exposed for testing.
[[nodiscard]] Severity evaluateThreshold(const ThresholdRule& rule, double value,
                                         Severity current) noexcept;

/// Maps one holding register to a named process variable.
struct ChannelConfig {
    std::string name;
    std::string unit;
    std::uint16_t address{0};
    double scale{1.0};  ///< Engineering value = raw register value / scale.
    ThresholdRule rule;
};

struct ClientConfig {
    std::string host{"127.0.0.1"};
    std::uint16_t port{1502};
    std::uint8_t unitId{1};
    std::chrono::milliseconds pollInterval{500};
    std::chrono::milliseconds connectTimeout{1000};
    std::chrono::milliseconds responseTimeout{1000};
    std::string alarmLogPath{"alarms.csv"};  ///< Empty string disables the CSV log.
    std::vector<ChannelConfig> channels;
};

/// A change of state on a process channel or on the communication link itself.
struct AlarmEvent {
    std::chrono::system_clock::time_point timestamp;
    std::string source;
    Severity previous{Severity::Normal};
    Severity current{Severity::Normal};
    std::optional<double> value;
    std::string unit;
    std::string message;
};

struct ChannelStatus {
    std::string name;
    std::string unit;
    std::optional<double> value;
    Severity severity{Severity::Normal};
};

struct ClientStatus {
    bool connected{false};
    std::uint64_t pollsTotal{0};
    std::uint64_t pollsFailed{0};
    std::uint64_t alarmEvents{0};
    std::string lastError;
    std::vector<ChannelStatus> channels;
};

/// Name used for events raised by the communication watchdog.
inline constexpr const char* kLinkSourceName = "LINK";

/// Polls a Modbus TCP device on a background thread, converts raw registers to
/// engineering values, evaluates threshold rules and reports every state change
/// to a CSV log and to an optional callback.
class ModbusAlarmClient {
public:
    using AlarmCallback = std::function<void(const AlarmEvent&)>;

    explicit ModbusAlarmClient(ClientConfig config, AlarmCallback onAlarm = {});
    ~ModbusAlarmClient();

    ModbusAlarmClient(const ModbusAlarmClient&) = delete;
    ModbusAlarmClient& operator=(const ModbusAlarmClient&) = delete;
    ModbusAlarmClient(ModbusAlarmClient&&) = delete;
    ModbusAlarmClient& operator=(ModbusAlarmClient&&) = delete;

    void start();
    void stop() noexcept;

    /// Executes one complete poll cycle on the calling thread.
    /// Intended for tests and one-shot tools; do not call while start() is active.
    void pollOnce();

    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] ClientStatus status() const;
    [[nodiscard]] const ClientConfig& config() const noexcept;

private:
    void pollLoop() noexcept;
    void ensureConnected();
    [[nodiscard]] std::vector<std::uint16_t> readHoldingRegisters(std::uint16_t startAddress,
                                                                  std::uint16_t quantity);
    void recordLinkState(bool linkUp, const std::string& reason, std::vector<AlarmEvent>& events);
    void publish(const std::vector<AlarmEvent>& events);
    void writeLogEntry(const AlarmEvent& event);

    const ClientConfig config_;
    const AlarmCallback onAlarm_;
    std::uint16_t blockStart_{0};
    std::uint16_t blockLength_{0};

    // Declared before the socket so that Winsock is torn down last.
    net::NetworkRuntime networkRuntime_;

    // Owned by the polling thread (or by the caller of pollOnce()).
    net::TcpSocket socket_;
    std::uint16_t nextTransactionId_{1};
    Severity linkSeverity_{Severity::Normal};

    mutable std::mutex statusMutex_;
    ClientStatus status_;

    std::mutex logMutex_;
    std::ofstream logFile_;

    std::atomic<bool> running_{false};
    std::mutex wakeMutex_;
    std::condition_variable wakeCondition_;
    std::thread pollThread_;
};

/// Formats a timestamp as local time "YYYY-MM-DD HH:MM:SS.mmm".
[[nodiscard]] std::string formatTimestamp(std::chrono::system_clock::time_point timestamp);

}  // namespace monitor
