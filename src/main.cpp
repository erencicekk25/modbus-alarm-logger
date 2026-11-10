#include "ModbusAlarmClient.hpp"
#include "VirtualModbusServer.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

volatile std::sig_atomic_t gStopRequested = 0;

extern "C" void handleStopSignal(int /*signal*/) { gStopRequested = 1; }

struct Options {
    std::uint16_t port{1502};
    std::chrono::milliseconds pollInterval{500};
    std::chrono::milliseconds statusInterval{3000};
    std::chrono::seconds duration{0};  ///< 0 runs until Ctrl+C.
    std::string logPath{"alarms.csv"};
    std::uint32_t seed{0};
};

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n\n"
              << "Options:\n"
              << "  --port <n>         TCP port of the virtual device (default 1502, 0 = any free port)\n"
              << "  --interval <ms>    Poll interval of the alarm client (default 500)\n"
              << "  --status <ms>      Status table refresh interval (default 3000)\n"
              << "  --duration <s>     Stop automatically after the given time (default 0 = run until Ctrl+C)\n"
              << "  --log <path>       CSV alarm log file (default alarms.csv)\n"
              << "  --no-log           Disable the CSV alarm log\n"
              << "  --seed <n>         Fixed random seed for a reproducible simulation\n"
              << "  --help             Show this help\n";
}

[[nodiscard]] unsigned long parseNumber(const std::string& option, const std::string& text,
                                        unsigned long maximum) {
    std::size_t consumed = 0;
    unsigned long value = 0;
    try {
        value = std::stoul(text, &consumed);
    } catch (const std::exception&) {
        consumed = 0;
    }
    if (consumed != text.size() || text.empty() || text.front() == '-' || value > maximum) {
        throw std::invalid_argument("Invalid value '" + text + "' for " + option);
    }
    return value;
}

/// Returns std::nullopt when the program should exit right away (e.g. after --help).
[[nodiscard]] std::optional<Options> parseArguments(const std::vector<std::string>& arguments,
                                                    const char* program) {
    Options options;
    for (std::size_t i = 0; i < arguments.size(); ++i) {
        const std::string& option = arguments[i];
        const auto nextValue = [&]() -> const std::string& {
            if (i + 1 >= arguments.size()) {
                throw std::invalid_argument("Missing value for " + option);
            }
            return arguments[++i];
        };

        if (option == "--help" || option == "-h") {
            printUsage(program);
            return std::nullopt;
        }
        if (option == "--port") {
            options.port = static_cast<std::uint16_t>(parseNumber(option, nextValue(), 65535));
        } else if (option == "--interval") {
            options.pollInterval = std::chrono::milliseconds(parseNumber(option, nextValue(), 60000));
            if (options.pollInterval.count() == 0) {
                throw std::invalid_argument("--interval must be greater than zero");
            }
        } else if (option == "--status") {
            options.statusInterval = std::chrono::milliseconds(parseNumber(option, nextValue(), 3600000));
        } else if (option == "--duration") {
            options.duration = std::chrono::seconds(parseNumber(option, nextValue(), 86400UL * 365));
        } else if (option == "--log") {
            options.logPath = nextValue();
        } else if (option == "--no-log") {
            options.logPath.clear();
        } else if (option == "--seed") {
            options.seed = static_cast<std::uint32_t>(
                parseNumber(option, nextValue(), std::numeric_limits<std::uint32_t>::max()));
        } else {
            throw std::invalid_argument("Unknown option '" + option + "' (see --help)");
        }
    }
    return options;
}

/// Channel definitions that match the register map of the virtual device.
[[nodiscard]] std::vector<monitor::ChannelConfig> buildChannels() {
    std::vector<monitor::ChannelConfig> channels;

    monitor::ThresholdRule temperature;
    temperature.highWarning = 78.0;
    temperature.highAlarm = 85.0;
    temperature.hysteresis = 1.0;
    channels.push_back({"Temperature", "C", sim::registers::kTemperature, 10.0, temperature});

    monitor::ThresholdRule pressure;
    pressure.lowAlarm = 2.5;
    pressure.lowWarning = 3.2;
    pressure.highWarning = 5.8;
    pressure.highAlarm = 6.5;
    pressure.hysteresis = 0.1;
    channels.push_back({"Pressure", "bar", sim::registers::kPressure, 100.0, pressure});

    monitor::ThresholdRule motorSpeed;
    motorSpeed.lowAlarm = 1100.0;
    motorSpeed.lowWarning = 1250.0;
    motorSpeed.highWarning = 1600.0;
    motorSpeed.highAlarm = 1700.0;
    motorSpeed.hysteresis = 15.0;
    channels.push_back({"MotorSpeed", "rpm", sim::registers::kMotorSpeed, 1.0, motorSpeed});

    monitor::ThresholdRule vibration;
    vibration.highWarning = 4.5;
    vibration.highAlarm = 7.1;
    vibration.hysteresis = 0.3;
    channels.push_back({"Vibration", "mm/s", sim::registers::kVibration, 100.0, vibration});

    monitor::ThresholdRule tankLevel;
    tankLevel.lowAlarm = 10.0;
    tankLevel.lowWarning = 20.0;
    tankLevel.highWarning = 85.0;
    tankLevel.highAlarm = 95.0;
    tankLevel.hysteresis = 1.0;
    channels.push_back({"TankLevel", "%", sim::registers::kTankLevel, 10.0, tankLevel});

    return channels;
}

/// Serializes console output coming from the main thread and the polling thread.
class ConsoleReporter {
public:
    void printLine(const std::string& line) {
        const std::lock_guard<std::mutex> lock(mutex_);
        std::cout << line << std::endl;
    }

    void printAlarm(const monitor::AlarmEvent& event) {
        std::ostringstream line;
        line << '[' << monitor::formatTimestamp(event.timestamp) << "] " << std::left << std::setw(8)
             << monitor::toString(event.current) << ' ' << std::setw(12) << event.source << ' '
             << event.message;
        printLine(line.str());
    }

    void printStatus(const monitor::ClientStatus& status, const sim::ServerStatistics& server) {
        std::ostringstream table;
        const std::string rule(66, '-');
        table << rule << '\n'
              << ' ' << monitor::formatTimestamp(std::chrono::system_clock::now()).substr(0, 19)
              << " | link " << (status.connected ? "ONLINE " : "OFFLINE") << " | polls "
              << status.pollsTotal << " (failed " << status.pollsFailed << ") | events "
              << status.alarmEvents << '\n'
              << " device: " << server.connectionsAccepted << " connection(s), "
              << server.requestsServed << " request(s) served\n"
              << rule << '\n'
              << ' ' << std::left << std::setw(14) << "CHANNEL" << std::right << std::setw(10)
              << "VALUE" << "  " << std::left << std::setw(6) << "UNIT" << "STATE\n";

        for (const monitor::ChannelStatus& channel : status.channels) {
            table << ' ' << std::left << std::setw(14) << channel.name << std::right << std::setw(10);
            if (channel.value) {
                table << std::fixed << std::setprecision(2) << *channel.value;
            } else {
                table << "--";
            }
            table << "  " << std::left << std::setw(6) << channel.unit
                  << monitor::toString(channel.severity) << '\n';
        }
        if (!status.connected && !status.lastError.empty()) {
            table << " last error: " << status.lastError << '\n';
        }
        table << rule;
        printLine(table.str());
    }

private:
    std::mutex mutex_;
};

int run(const Options& options) {
    std::signal(SIGINT, handleStopSignal);
    std::signal(SIGTERM, handleStopSignal);

    ConsoleReporter reporter;

    sim::ServerConfig serverConfig;
    serverConfig.port = options.port;
    serverConfig.randomSeed = options.seed;
    sim::VirtualModbusServer server(serverConfig);
    server.start();

    monitor::ClientConfig clientConfig;
    clientConfig.host = serverConfig.bindAddress;
    clientConfig.port = server.port();
    clientConfig.unitId = serverConfig.unitId;
    clientConfig.pollInterval = options.pollInterval;
    clientConfig.alarmLogPath = options.logPath;
    clientConfig.channels = buildChannels();

    // Declared after the server, so it is destroyed (and its thread joined) first.
    monitor::ModbusAlarmClient client(std::move(clientConfig),
                                      [&reporter](const monitor::AlarmEvent& event) {
                                          reporter.printAlarm(event);
                                      });

    reporter.printLine("modbus-alarm-logger | virtual device listening on " +
                       serverConfig.bindAddress + ":" + std::to_string(server.port()) +
                       " (unit " + std::to_string(serverConfig.unitId) + ")");
    reporter.printLine("Polling every " + std::to_string(options.pollInterval.count()) + " ms" +
                       (options.logPath.empty() ? std::string(", CSV log disabled")
                                                : ", alarms logged to " + options.logPath) +
                       ". Press Ctrl+C to stop.");

    client.start();

    const auto startTime = std::chrono::steady_clock::now();
    auto nextStatus = startTime + options.statusInterval;
    while (gStopRequested == 0) {
        const auto now = std::chrono::steady_clock::now();
        if (options.duration.count() > 0 && now - startTime >= options.duration) {
            break;
        }
        if (options.statusInterval.count() > 0 && now >= nextStatus) {
            reporter.printStatus(client.status(), server.statistics());
            nextStatus += options.statusInterval;
        }
        std::this_thread::sleep_for(100ms);
    }

    reporter.printLine("Shutting down...");
    client.stop();
    server.stop();

    const monitor::ClientStatus finalStatus = client.status();
    reporter.printStatus(finalStatus, server.statistics());
    reporter.printLine("Done. " + std::to_string(finalStatus.alarmEvents) + " alarm event(s) recorded.");
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::vector<std::string> arguments(argv + 1, argv + argc);
        const std::optional<Options> options = parseArguments(arguments, argv[0]);
        if (!options) {
            return EXIT_SUCCESS;
        }
        return run(*options);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
}
