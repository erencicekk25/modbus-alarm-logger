#include "ModbusAlarmClient.hpp"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace monitor {
namespace {

/// A value must at least reach `limit` to enter a state, but only needs to stay
/// within `hysteresis` of it to remain there.
[[nodiscard]] bool isAbove(const std::optional<double>& limit, double value, bool alreadyActive,
                           double hysteresis) noexcept {
    return limit.has_value() && value >= (alreadyActive ? *limit - hysteresis : *limit);
}

[[nodiscard]] bool isBelow(const std::optional<double>& limit, double value, bool alreadyActive,
                           double hysteresis) noexcept {
    return limit.has_value() && value <= (alreadyActive ? *limit + hysteresis : *limit);
}

/// Decides whether a deviation is on the high or the low side of the normal band.
[[nodiscard]] bool isHighSide(const ThresholdRule& rule, double value) noexcept {
    const std::optional<double> lowReference = rule.lowWarning ? rule.lowWarning : rule.lowAlarm;
    const std::optional<double> highReference = rule.highWarning ? rule.highWarning : rule.highAlarm;
    if (!lowReference) {
        return true;
    }
    if (!highReference) {
        return false;
    }
    return value >= (*lowReference + *highReference) / 2.0;
}

[[nodiscard]] std::string describeTransition(const ChannelConfig& channel, double value,
                                             Severity next) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(2);

    if (next == Severity::Normal) {
        text << "Back to normal at " << value << ' ' << channel.unit;
        return text.str();
    }

    const bool high = isHighSide(channel.rule, value);
    const ThresholdRule& rule = channel.rule;
    const std::optional<double> limit =
        next == Severity::Alarm ? (high ? rule.highAlarm : rule.lowAlarm)
                                : (high ? rule.highWarning : rule.lowWarning);

    text << (high ? "HIGH " : "LOW ") << (next == Severity::Alarm ? "ALARM" : "WARNING") << ": "
         << value << ' ' << channel.unit;
    if (limit) {
        text << " (limit " << *limit << ' ' << channel.unit << ')';
    }
    return text.str();
}

[[nodiscard]] std::string escapeCsv(const std::string& field) {
    std::string escaped;
    escaped.reserve(field.size() + 2);
    escaped.push_back('"');
    for (const char character : field) {
        if (character == '"') {
            escaped.push_back('"');
        }
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

void validateConfig(const ClientConfig& config) {
    if (config.channels.empty()) {
        throw std::invalid_argument("At least one channel must be configured");
    }
    if (config.pollInterval.count() <= 0) {
        throw std::invalid_argument("Poll interval must be positive");
    }
    for (const ChannelConfig& channel : config.channels) {
        if (channel.name.empty()) {
            throw std::invalid_argument("Channel name must not be empty");
        }
        if (!(channel.scale > 0.0)) {
            throw std::invalid_argument("Channel '" + channel.name + "' must have a positive scale");
        }
        if (channel.rule.hysteresis < 0.0) {
            throw std::invalid_argument("Channel '" + channel.name + "' has a negative hysteresis");
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

const char* toString(Severity severity) noexcept {
    switch (severity) {
        case Severity::Normal: return "NORMAL";
        case Severity::Warning: return "WARNING";
        case Severity::Alarm: return "ALARM";
    }
    return "UNKNOWN";
}

Severity evaluateThreshold(const ThresholdRule& rule, double value, Severity current) noexcept {
    const bool inAlarm = current >= Severity::Alarm;
    const bool inWarning = current >= Severity::Warning;
    const double band = rule.hysteresis;

    if (isAbove(rule.highAlarm, value, inAlarm, band) || isBelow(rule.lowAlarm, value, inAlarm, band)) {
        return Severity::Alarm;
    }
    if (isAbove(rule.highWarning, value, inWarning, band) ||
        isBelow(rule.lowWarning, value, inWarning, band)) {
        return Severity::Warning;
    }
    return Severity::Normal;
}

std::string formatTimestamp(std::chrono::system_clock::time_point timestamp) {
    const std::time_t seconds = std::chrono::system_clock::to_time_t(timestamp);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &seconds);
#else
    localtime_r(&seconds, &local);
#endif
    const auto millis =
        std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()).count() %
        1000;

    std::ostringstream text;
    text << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0')
         << millis;
    return text.str();
}

// ---------------------------------------------------------------------------
// ModbusAlarmClient
// ---------------------------------------------------------------------------

ModbusAlarmClient::ModbusAlarmClient(ClientConfig config, AlarmCallback onAlarm)
    : config_(std::move(config)), onAlarm_(std::move(onAlarm)) {
    validateConfig(config_);

    // All channels are fetched with a single request covering the smallest
    // contiguous block, which is far cheaper than one round trip per register.
    const auto [lowest, highest] = std::minmax_element(
        config_.channels.begin(), config_.channels.end(),
        [](const ChannelConfig& a, const ChannelConfig& b) { return a.address < b.address; });
    const std::uint32_t span = static_cast<std::uint32_t>(highest->address) - lowest->address + 1U;
    if (span > modbus::kMaxRegistersPerRead) {
        throw std::invalid_argument("Configured channels span " + std::to_string(span) +
                                    " registers; a single read is limited to " +
                                    std::to_string(modbus::kMaxRegistersPerRead));
    }
    blockStart_ = lowest->address;
    blockLength_ = static_cast<std::uint16_t>(span);

    status_.channels.reserve(config_.channels.size());
    for (const ChannelConfig& channel : config_.channels) {
        status_.channels.push_back(ChannelStatus{channel.name, channel.unit, std::nullopt, Severity::Normal});
    }

    if (!config_.alarmLogPath.empty()) {
        std::error_code error;
        const bool needsHeader = !std::filesystem::exists(config_.alarmLogPath, error) ||
                                 std::filesystem::file_size(config_.alarmLogPath, error) == 0;
        logFile_.open(config_.alarmLogPath, std::ios::out | std::ios::app);
        if (!logFile_) {
            throw std::runtime_error("Unable to open alarm log '" + config_.alarmLogPath + "'");
        }
        if (needsHeader) {
            logFile_ << "timestamp,source,previous,current,value,unit,message\n";
            logFile_.flush();
        }
    }
}

ModbusAlarmClient::~ModbusAlarmClient() { stop(); }

void ModbusAlarmClient::start() {
    if (running_.load()) {
        throw std::logic_error("ModbusAlarmClient is already running");
    }
    running_.store(true);
    try {
        pollThread_ = std::thread(&ModbusAlarmClient::pollLoop, this);
    } catch (...) {
        running_.store(false);
        throw;
    }
}

void ModbusAlarmClient::stop() noexcept {
    {
        const std::lock_guard<std::mutex> lock(wakeMutex_);
        running_.store(false);
    }
    wakeCondition_.notify_all();
    if (pollThread_.joinable()) {
        pollThread_.join();
    }
    socket_.close();
}

bool ModbusAlarmClient::isRunning() const noexcept { return running_.load(); }

ClientStatus ModbusAlarmClient::status() const {
    const std::lock_guard<std::mutex> lock(statusMutex_);
    return status_;
}

const ClientConfig& ModbusAlarmClient::config() const noexcept { return config_; }

void ModbusAlarmClient::pollLoop() noexcept {
    while (running_.load()) {
        const auto deadline = std::chrono::steady_clock::now() + config_.pollInterval;
        try {
            pollOnce();
        } catch (const std::exception&) {
            // pollOnce() already records communication errors; anything that still
            // escapes (e.g. a throwing user callback) must not kill the monitor.
        }

        std::unique_lock<std::mutex> lock(wakeMutex_);
        wakeCondition_.wait_until(lock, deadline, [this] { return !running_.load(); });
    }
    socket_.close();
}

void ModbusAlarmClient::pollOnce() {
    std::vector<AlarmEvent> events;

    try {
        ensureConnected();
        const std::vector<std::uint16_t> raw = readHoldingRegisters(blockStart_, blockLength_);
        const auto now = std::chrono::system_clock::now();

        const std::lock_guard<std::mutex> lock(statusMutex_);
        ++status_.pollsTotal;
        status_.connected = true;
        status_.lastError.clear();
        recordLinkState(true, "Communication restored", events);

        for (std::size_t i = 0; i < config_.channels.size(); ++i) {
            const ChannelConfig& channel = config_.channels[i];
            ChannelStatus& state = status_.channels[i];

            const double value = static_cast<double>(raw[channel.address - blockStart_]) / channel.scale;
            const Severity next = evaluateThreshold(channel.rule, value, state.severity);
            if (next != state.severity) {
                events.push_back(AlarmEvent{now, channel.name, state.severity, next, value, channel.unit,
                                            describeTransition(channel, value, next)});
            }
            state.value = value;
            state.severity = next;
        }
        status_.alarmEvents += events.size();
    } catch (const modbus::ModbusException& error) {
        // The device is reachable but refused the request; keep the connection.
        const std::lock_guard<std::mutex> lock(statusMutex_);
        ++status_.pollsTotal;
        ++status_.pollsFailed;
        status_.lastError = error.what();
    } catch (const std::exception& error) {
        // Network or framing failure: the stream can no longer be trusted, so start over.
        socket_.close();
        const std::lock_guard<std::mutex> lock(statusMutex_);
        ++status_.pollsTotal;
        ++status_.pollsFailed;
        status_.connected = false;
        status_.lastError = error.what();
        recordLinkState(false, error.what(), events);
        status_.alarmEvents += events.size();
    }

    publish(events);
}

void ModbusAlarmClient::ensureConnected() {
    if (socket_.isValid()) {
        return;
    }
    net::TcpSocket socket = net::TcpSocket::connect(config_.host, config_.port, config_.connectTimeout);
    socket.setIoTimeout(config_.responseTimeout);
    socket_ = std::move(socket);
}

std::vector<std::uint16_t> ModbusAlarmClient::readHoldingRegisters(std::uint16_t startAddress,
                                                                   std::uint16_t quantity) {
    const std::uint16_t transactionId = nextTransactionId_++;
    const auto request =
        modbus::buildReadHoldingRegistersRequest(transactionId, config_.unitId, startAddress, quantity);
    socket_.sendAll(request.data(), request.size());

    std::array<std::uint8_t, modbus::kMbapHeaderSize> headerBytes{};
    if (!socket_.receiveExact(headerBytes.data(), headerBytes.size())) {
        throw net::NetworkError("Connection closed by the device");
    }

    const modbus::MbapHeader header = modbus::decodeHeader(headerBytes.data());
    if (header.transactionId != transactionId) {
        throw modbus::ProtocolError("Unexpected transaction id " + std::to_string(header.transactionId) +
                                    " (expected " + std::to_string(transactionId) + ")");
    }
    if (header.protocolId != modbus::kProtocolIdentifier) {
        throw modbus::ProtocolError("Invalid protocol identifier in response");
    }
    if (header.length < 3 || header.length > modbus::kMaxPduSize + 1) {
        throw modbus::ProtocolError("Invalid response length " + std::to_string(header.length));
    }

    std::vector<std::uint8_t> pdu(static_cast<std::size_t>(header.length) - 1U);
    if (!socket_.receiveExact(pdu.data(), pdu.size())) {
        throw net::NetworkError("Connection closed by the device");
    }

    constexpr auto kReadFunction = static_cast<std::uint8_t>(modbus::FunctionCode::ReadHoldingRegisters);
    const std::uint8_t functionCode = pdu[0];
    if (functionCode == (kReadFunction | modbus::kExceptionFlag)) {
        throw modbus::ModbusException(static_cast<modbus::ExceptionCode>(pdu[1]));
    }
    if (functionCode != kReadFunction) {
        throw modbus::ProtocolError("Unexpected function code " + std::to_string(functionCode));
    }

    const std::size_t expectedBytes = static_cast<std::size_t>(quantity) * 2U;
    if (pdu[1] != expectedBytes || pdu.size() != expectedBytes + 2U) {
        throw modbus::ProtocolError("Byte count mismatch in response");
    }

    std::vector<std::uint16_t> values(quantity);
    for (std::size_t i = 0; i < values.size(); ++i) {
        values[i] = modbus::readBigEndian16(&pdu[2 + i * 2U]);
    }
    return values;
}

void ModbusAlarmClient::recordLinkState(bool linkUp, const std::string& reason,
                                        std::vector<AlarmEvent>& events) {
    const Severity next = linkUp ? Severity::Normal : Severity::Alarm;
    if (next == linkSeverity_) {
        return;
    }
    events.push_back(AlarmEvent{std::chrono::system_clock::now(), kLinkSourceName, linkSeverity_, next,
                                std::nullopt, "", reason});
    linkSeverity_ = next;
}

void ModbusAlarmClient::publish(const std::vector<AlarmEvent>& events) {
    for (const AlarmEvent& event : events) {
        writeLogEntry(event);
        if (onAlarm_) {
            onAlarm_(event);
        }
    }
}

void ModbusAlarmClient::writeLogEntry(const AlarmEvent& event) {
    const std::lock_guard<std::mutex> lock(logMutex_);
    if (!logFile_.is_open()) {
        return;
    }

    logFile_ << formatTimestamp(event.timestamp) << ',' << escapeCsv(event.source) << ','
             << toString(event.previous) << ',' << toString(event.current) << ',';
    if (event.value) {
        logFile_ << std::fixed << std::setprecision(2) << *event.value;
    }
    logFile_ << ',' << escapeCsv(event.unit) << ',' << escapeCsv(event.message) << '\n';
    logFile_.flush();  // Alarm records must survive a crash or a hard stop.
}

}  // namespace monitor
