// Self-contained test runner: no third-party framework is required to build or run it.

#include "ModbusAlarmClient.hpp"
#include "ModbusProtocol.hpp"
#include "VirtualModbusServer.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

#define EXPECT_TRUE(condition)                                                                   \
    do {                                                                                         \
        if (!(condition)) {                                                                      \
            ++gFailures;                                                                         \
            std::cerr << "  FAILED: " #condition " (" << __FILE__ << ':' << __LINE__ << ")\n"; \
        }                                                                                        \
    } while (false)

#define EXPECT_EQ(actual, expected) EXPECT_TRUE((actual) == (expected))

bool nearlyEqual(double a, double b) { return std::fabs(a - b) < 1e-9; }

monitor::ThresholdRule temperatureRule() {
    monitor::ThresholdRule rule;
    rule.lowAlarm = 5.0;
    rule.lowWarning = 10.0;
    rule.highWarning = 70.0;
    rule.highAlarm = 80.0;
    rule.hysteresis = 2.0;
    return rule;
}

monitor::ClientConfig testClientConfig(std::uint16_t port) {
    monitor::ClientConfig config;
    config.port = port;
    config.alarmLogPath.clear();
    config.connectTimeout = std::chrono::milliseconds(500);
    config.responseTimeout = std::chrono::milliseconds(500);
    config.channels.push_back({"Temperature", "C", sim::registers::kTemperature, 10.0, temperatureRule()});
    return config;
}

sim::ServerConfig testServerConfig() {
    sim::ServerConfig config;
    config.port = 0;  // Any free port.
    config.enableSimulation = false;
    return config;
}

// ---------------------------------------------------------------------------

void testHeaderRoundTrip() {
    const modbus::MbapHeader original{0xBEEF, 0, 6, 0x11};
    const auto bytes = modbus::encodeHeader(original);
    EXPECT_EQ(bytes[0], 0xBE);
    EXPECT_EQ(bytes[1], 0xEF);

    const modbus::MbapHeader decoded = modbus::decodeHeader(bytes.data());
    EXPECT_EQ(decoded.transactionId, original.transactionId);
    EXPECT_EQ(decoded.protocolId, original.protocolId);
    EXPECT_EQ(decoded.length, original.length);
    EXPECT_EQ(decoded.unitId, original.unitId);
}

void testReadRequestFrame() {
    const auto frame = modbus::buildReadHoldingRegistersRequest(1, 1, 0x006B, 3);
    const std::vector<std::uint8_t> expected{0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01,
                                             0x03, 0x00, 0x6B, 0x00, 0x03};
    EXPECT_TRUE(std::vector<std::uint8_t>(frame.begin(), frame.end()) == expected);
}

void testThresholdLevels() {
    using monitor::Severity;
    const monitor::ThresholdRule rule = temperatureRule();
    EXPECT_EQ(monitor::evaluateThreshold(rule, 40.0, Severity::Normal), Severity::Normal);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 70.0, Severity::Normal), Severity::Warning);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 85.0, Severity::Normal), Severity::Alarm);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 9.0, Severity::Normal), Severity::Warning);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 4.0, Severity::Normal), Severity::Alarm);
}

void testThresholdHysteresis() {
    using monitor::Severity;
    const monitor::ThresholdRule rule = temperatureRule();
    // Inside the dead band the current state is kept...
    EXPECT_EQ(monitor::evaluateThreshold(rule, 79.0, Severity::Alarm), Severity::Alarm);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 79.0, Severity::Warning), Severity::Warning);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 69.0, Severity::Warning), Severity::Warning);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 11.0, Severity::Warning), Severity::Warning);
    // ...and released only once the value leaves it.
    EXPECT_EQ(monitor::evaluateThreshold(rule, 77.9, Severity::Alarm), Severity::Warning);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 67.9, Severity::Warning), Severity::Normal);
    EXPECT_EQ(monitor::evaluateThreshold(rule, 12.1, Severity::Warning), Severity::Normal);
}

void testEndToEndAlarmCycle() {
    sim::VirtualModbusServer server(testServerConfig());
    server.start();

    std::vector<monitor::AlarmEvent> received;
    monitor::ModbusAlarmClient client(testClientConfig(server.port()),
                                      [&received](const monitor::AlarmEvent& event) {
                                          received.push_back(event);
                                      });

    server.writeRegister(sim::registers::kTemperature, 450);  // 45.0 C
    client.pollOnce();
    monitor::ClientStatus status = client.status();
    EXPECT_TRUE(status.connected);
    EXPECT_TRUE(status.channels.at(0).value.has_value());
    EXPECT_TRUE(nearlyEqual(status.channels.at(0).value.value_or(0.0), 45.0));
    EXPECT_EQ(status.channels.at(0).severity, monitor::Severity::Normal);
    EXPECT_TRUE(received.empty());

    server.writeRegister(sim::registers::kTemperature, 853);  // 85.3 C -> alarm
    client.pollOnce();
    EXPECT_EQ(client.status().channels.at(0).severity, monitor::Severity::Alarm);
    EXPECT_EQ(received.size(), 1U);
    if (!received.empty()) {
        EXPECT_EQ(received.back().source, "Temperature");
        EXPECT_EQ(received.back().previous, monitor::Severity::Normal);
        EXPECT_EQ(received.back().current, monitor::Severity::Alarm);
    }

    server.writeRegister(sim::registers::kTemperature, 600);  // 60.0 C -> normal
    client.pollOnce();
    status = client.status();
    EXPECT_EQ(status.channels.at(0).severity, monitor::Severity::Normal);
    EXPECT_EQ(status.alarmEvents, 2U);
    EXPECT_EQ(status.pollsFailed, 0U);
    EXPECT_EQ(server.statistics().requestsServed, 3U);
}

void testIllegalAddressException() {
    sim::VirtualModbusServer server(testServerConfig());
    server.start();

    monitor::ClientConfig config = testClientConfig(server.port());
    config.channels.front().address = sim::registers::kCount;  // One past the register bank.
    monitor::ModbusAlarmClient client(std::move(config));

    client.pollOnce();
    const monitor::ClientStatus status = client.status();
    EXPECT_EQ(status.pollsFailed, 1U);
    EXPECT_TRUE(status.lastError.find("Illegal data address") != std::string::npos);
    EXPECT_EQ(server.statistics().exceptionsReturned, 1U);
}

void testLinkLossRaisesAlarm() {
    std::uint16_t unusedPort = 0;
    {
        sim::VirtualModbusServer probe(testServerConfig());
        probe.start();
        unusedPort = probe.port();
    }  // The port is released here, so nothing listens on it any more.

    std::vector<monitor::AlarmEvent> received;
    monitor::ModbusAlarmClient client(testClientConfig(unusedPort),
                                      [&received](const monitor::AlarmEvent& event) {
                                          received.push_back(event);
                                      });
    client.pollOnce();
    client.pollOnce();  // A persisting outage must not produce duplicate events.

    const monitor::ClientStatus status = client.status();
    EXPECT_TRUE(!status.connected);
    EXPECT_EQ(status.pollsFailed, 2U);
    EXPECT_EQ(received.size(), 1U);
    if (!received.empty()) {
        EXPECT_EQ(received.front().source, monitor::kLinkSourceName);
        EXPECT_EQ(received.front().current, monitor::Severity::Alarm);
    }
}

void testBackgroundThreadsStopCleanly() {
    sim::ServerConfig serverConfig = testServerConfig();
    serverConfig.enableSimulation = true;
    serverConfig.randomSeed = 42;
    serverConfig.simulationStep = std::chrono::milliseconds(20);
    sim::VirtualModbusServer server(serverConfig);
    server.start();

    monitor::ClientConfig clientConfig = testClientConfig(server.port());
    clientConfig.pollInterval = std::chrono::milliseconds(20);
    monitor::ModbusAlarmClient client(std::move(clientConfig));
    client.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    client.stop();
    server.stop();

    const monitor::ClientStatus status = client.status();
    EXPECT_TRUE(status.pollsTotal >= 3U);
    EXPECT_EQ(status.pollsFailed, 0U);
    EXPECT_TRUE(!client.isRunning());
    EXPECT_TRUE(!server.isRunning());
    EXPECT_TRUE(server.registerSnapshot().at(sim::registers::kHeartbeat) > 0U);
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests{
        {"MBAP header round trip", testHeaderRoundTrip},
        {"Read request frame layout", testReadRequestFrame},
        {"Threshold levels", testThresholdLevels},
        {"Threshold hysteresis", testThresholdHysteresis},
        {"End-to-end alarm cycle", testEndToEndAlarmCycle},
        {"Illegal address exception", testIllegalAddressException},
        {"Link loss raises a single alarm", testLinkLossRaisesAlarm},
        {"Background threads stop cleanly", testBackgroundThreadsStopCleanly},
    };

    for (const auto& [name, test] : tests) {
        const int failuresBefore = gFailures;
        try {
            test();
        } catch (const std::exception& error) {
            ++gFailures;
            std::cerr << "  FAILED: unexpected exception: " << error.what() << '\n';
        }
        std::cout << (gFailures == failuresBefore ? "[ PASS ] " : "[ FAIL ] ") << name << '\n';
    }

    std::cout << (gFailures == 0 ? "All tests passed." : "Some tests failed.") << '\n';
    return gFailures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
