#include "VirtualModbusServer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

constexpr double kTwoPi = 6.283185307179586;
constexpr double kExcursionSmoothing = 0.12;  // Fraction of the gap closed per step.
constexpr std::chrono::milliseconds kAcceptPollInterval{100};
constexpr std::chrono::milliseconds kClientIdlePoll{100};
constexpr std::chrono::milliseconds kClientIoTimeout{1000};

[[nodiscard]] std::vector<std::uint8_t> buildExceptionResponse(const modbus::MbapHeader& request,
                                                               std::uint8_t functionCode,
                                                               modbus::ExceptionCode code) {
    const auto header = modbus::encodeHeader(
        modbus::MbapHeader{request.transactionId, modbus::kProtocolIdentifier, 3, request.unitId});

    std::vector<std::uint8_t> response(header.begin(), header.end());
    response.push_back(static_cast<std::uint8_t>(functionCode | modbus::kExceptionFlag));
    response.push_back(static_cast<std::uint8_t>(code));
    return response;
}

}  // namespace

VirtualModbusServer::VirtualModbusServer(ServerConfig config)
    : config_(std::move(config)),
      rng_(config_.randomSeed != 0 ? config_.randomSeed : std::random_device{}()) {
    if (config_.simulationStep.count() <= 0) {
        throw std::invalid_argument("Simulation step must be positive");
    }

    // address, baseline, amplitude, period, noise, upset size, bidirectional, min, max, scale
    const std::array<SignalModel, 5> models{{
        {registers::kTemperature, 65.0, 4.0, 60.0, 0.30, 24.0, false, -20.0, 150.0, 10.0},
        {registers::kPressure, 4.50, 0.30, 45.0, 0.05, 2.0, true, 0.0, 10.0, 100.0},
        {registers::kMotorSpeed, 1450.0, 30.0, 30.0, 8.0, 260.0, true, 0.0, 3000.0, 1.0},
        {registers::kVibration, 2.2, 0.4, 20.0, 0.15, 5.5, false, 0.0, 50.0, 100.0},
        {registers::kTankLevel, 60.0, 15.0, 120.0, 0.5, 32.0, true, 0.0, 100.0, 10.0},
    }};

    std::uniform_real_distribution<double> phaseDistribution(0.0, kTwoPi);
    signals_.reserve(models.size());
    for (const auto& model : models) {
        SignalState state{model};
        state.phase = phaseDistribution(rng_);
        signals_.push_back(state);
    }
}

VirtualModbusServer::~VirtualModbusServer() { stop(); }

void VirtualModbusServer::start() {
    if (running_.load()) {
        throw std::logic_error("VirtualModbusServer is already running");
    }

    listener_ = net::TcpSocket::listen(config_.bindAddress, config_.port);
    boundPort_ = listener_.localPort();

    if (config_.enableSimulation) {
        stepSimulation(0.0);  // Serve meaningful values from the very first request.
    }

    running_.store(true);
    try {
        networkThread_ = std::thread(&VirtualModbusServer::networkLoop, this);
        if (config_.enableSimulation) {
            simulationThread_ = std::thread(&VirtualModbusServer::simulationLoop, this);
        }
    } catch (...) {
        stop();
        throw;
    }
}

void VirtualModbusServer::stop() noexcept {
    {
        const std::lock_guard<std::mutex> lock(wakeMutex_);
        running_.store(false);
    }
    wakeCondition_.notify_all();

    if (networkThread_.joinable()) {
        networkThread_.join();
    }
    if (simulationThread_.joinable()) {
        simulationThread_.join();
    }
    listener_.close();
}

bool VirtualModbusServer::isRunning() const noexcept { return running_.load(); }

std::uint16_t VirtualModbusServer::port() const noexcept { return boundPort_; }

const ServerConfig& VirtualModbusServer::config() const noexcept { return config_; }

ServerStatistics VirtualModbusServer::statistics() const noexcept {
    return ServerStatistics{connectionsAccepted_.load(), requestsServed_.load(),
                            exceptionsReturned_.load()};
}

std::vector<std::uint16_t> VirtualModbusServer::registerSnapshot() const {
    const std::lock_guard<std::mutex> lock(registerMutex_);
    return std::vector<std::uint16_t>(registers_.begin(), registers_.end());
}

void VirtualModbusServer::writeRegister(std::uint16_t address, std::uint16_t value) {
    if (address >= registers::kCount) {
        throw std::out_of_range("Register address " + std::to_string(address) + " is out of range");
    }
    const std::lock_guard<std::mutex> lock(registerMutex_);
    registers_[address] = value;
}

// ---------------------------------------------------------------------------
// Network side
// ---------------------------------------------------------------------------

void VirtualModbusServer::networkLoop() noexcept {
    while (running_.load()) {
        try {
            const net::TcpSocket client = listener_.accept(kAcceptPollInterval);
            if (!client.isValid()) {
                continue;  // Timed out; re-check the running flag.
            }
            ++connectionsAccepted_;
            serveClient(client);
        } catch (const std::exception&) {
            // A broken or misbehaving client must never take the device down.
            // The socket is released by RAII and the server goes back to accepting.
        }
    }
}

void VirtualModbusServer::serveClient(const net::TcpSocket& client) {
    client.setIoTimeout(kClientIoTimeout);

    std::array<std::uint8_t, modbus::kMbapHeaderSize> headerBytes{};
    std::vector<std::uint8_t> pdu;

    while (running_.load()) {
        if (!client.waitReadable(kClientIdlePoll)) {
            continue;
        }
        if (!client.receiveExact(headerBytes.data(), headerBytes.size())) {
            return;  // Client disconnected cleanly.
        }

        const modbus::MbapHeader header = modbus::decodeHeader(headerBytes.data());
        if (header.protocolId != modbus::kProtocolIdentifier || header.length < 2 ||
            header.length > modbus::kMaxPduSize + 1) {
            return;  // Not a Modbus TCP frame: drop the connection, as real devices do.
        }

        pdu.resize(static_cast<std::size_t>(header.length) - 1U);
        if (!client.receiveExact(pdu.data(), pdu.size())) {
            return;
        }

        const std::vector<std::uint8_t> response = handleRequest(header, pdu);
        client.sendAll(response.data(), response.size());
        ++requestsServed_;
    }
}

bool VirtualModbusServer::acceptsUnit(std::uint8_t unitId) const noexcept {
    // 0x00 and 0xFF are commonly used to address "this device" directly over TCP.
    return unitId == config_.unitId || unitId == 0x00 || unitId == 0xFF;
}

std::vector<std::uint8_t> VirtualModbusServer::handleRequest(const modbus::MbapHeader& header,
                                                             const std::vector<std::uint8_t>& pdu) {
    const std::uint8_t functionCode = pdu.front();
    const auto reject = [&](modbus::ExceptionCode code) {
        ++exceptionsReturned_;
        return buildExceptionResponse(header, functionCode, code);
    };

    if (!acceptsUnit(header.unitId)) {
        return reject(modbus::ExceptionCode::GatewayTargetFailedToRespond);
    }
    if (functionCode != static_cast<std::uint8_t>(modbus::FunctionCode::ReadHoldingRegisters)) {
        return reject(modbus::ExceptionCode::IllegalFunction);
    }
    if (pdu.size() != 5) {
        return reject(modbus::ExceptionCode::IllegalDataValue);
    }

    const std::uint16_t startAddress = modbus::readBigEndian16(&pdu[1]);
    const std::uint16_t quantity = modbus::readBigEndian16(&pdu[3]);
    if (quantity == 0 || quantity > modbus::kMaxRegistersPerRead) {
        return reject(modbus::ExceptionCode::IllegalDataValue);
    }
    if (static_cast<std::uint32_t>(startAddress) + quantity > registers::kCount) {
        return reject(modbus::ExceptionCode::IllegalDataAddress);
    }

    const std::size_t byteCount = static_cast<std::size_t>(quantity) * 2U;
    const auto responseHeader = modbus::encodeHeader(modbus::MbapHeader{
        header.transactionId, modbus::kProtocolIdentifier,
        static_cast<std::uint16_t>(3U + byteCount), header.unitId});

    std::vector<std::uint8_t> response(modbus::kMbapHeaderSize + 2U + byteCount);
    std::copy(responseHeader.begin(), responseHeader.end(), response.begin());
    response[7] = functionCode;
    response[8] = static_cast<std::uint8_t>(byteCount);

    const std::lock_guard<std::mutex> lock(registerMutex_);
    for (std::size_t i = 0; i < quantity; ++i) {
        modbus::writeBigEndian16(&response[9 + i * 2U], registers_[startAddress + i]);
    }
    return response;
}

// ---------------------------------------------------------------------------
// Process simulation
// ---------------------------------------------------------------------------

void VirtualModbusServer::simulationLoop() noexcept {
    const auto startTime = std::chrono::steady_clock::now();
    auto nextStep = startTime;

    while (running_.load()) {
        nextStep += config_.simulationStep;
        const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - startTime;
        try {
            stepSimulation(elapsed.count());
        } catch (const std::exception&) {
            // The model only performs arithmetic; keep the last good values if anything fails.
        }

        std::unique_lock<std::mutex> lock(wakeMutex_);
        wakeCondition_.wait_until(lock, nextStep, [this] { return !running_.load(); });
    }
}

void VirtualModbusServer::stepSimulation(double elapsedSeconds) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> magnitudeFactor(0.7, 1.25);
    std::uniform_int_distribution<int> durationSteps(15, 60);
    std::normal_distribution<double> gaussian(0.0, 1.0);

    std::array<std::uint16_t, 5> rawValues{};
    for (std::size_t i = 0; i < signals_.size(); ++i) {
        SignalState& signal = signals_[i];
        const SignalModel& model = signal.model;

        // Occasionally start a process upset (overheating, pressure surge, cavitation...).
        if (signal.excursionStepsLeft == 0 && unit(rng_) < config_.excursionProbability) {
            const bool downwards = model.bidirectional && unit(rng_) < 0.5;
            const double magnitude = model.excursionMagnitude * magnitudeFactor(rng_);
            signal.excursionTarget = downwards ? -magnitude : magnitude;
            signal.excursionStepsLeft = durationSteps(rng_);
        }
        if (signal.excursionStepsLeft > 0 && --signal.excursionStepsLeft == 0) {
            signal.excursionTarget = 0.0;  // Upset is over; let the value recover gradually.
        }
        signal.excursionOffset += (signal.excursionTarget - signal.excursionOffset) * kExcursionSmoothing;

        const double drift =
            model.amplitude * std::sin(kTwoPi * elapsedSeconds / model.periodSeconds + signal.phase);
        const double noise = gaussian(rng_) * model.noiseStdDev;
        const double value = std::clamp(model.baseline + drift + noise + signal.excursionOffset,
                                        model.minValue, model.maxValue);

        const double scaled = std::clamp(value * model.scale, 0.0, 65535.0);
        rawValues[i] = static_cast<std::uint16_t>(std::lround(scaled));
    }

    const std::lock_guard<std::mutex> lock(registerMutex_);
    for (std::size_t i = 0; i < signals_.size(); ++i) {
        registers_[signals_[i].model.address] = rawValues[i];
    }
    ++registers_[registers::kHeartbeat];
}

}  // namespace sim
