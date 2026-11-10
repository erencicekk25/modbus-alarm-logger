#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

/// Minimal, allocation-free helpers for the Modbus TCP application protocol.
/// Only the subset required by this project is implemented: function code 0x03
/// (Read Holding Registers) and the standard exception responses.
namespace modbus {

constexpr std::uint16_t kProtocolIdentifier = 0x0000;
constexpr std::size_t kMbapHeaderSize = 7;
constexpr std::size_t kMaxPduSize = 253;
constexpr std::uint16_t kMaxRegistersPerRead = 125;
constexpr std::uint8_t kExceptionFlag = 0x80;

enum class FunctionCode : std::uint8_t {
    ReadHoldingRegisters = 0x03,
};

enum class ExceptionCode : std::uint8_t {
    IllegalFunction = 0x01,
    IllegalDataAddress = 0x02,
    IllegalDataValue = 0x03,
    ServerDeviceFailure = 0x04,
    GatewayTargetFailedToRespond = 0x0B,
};

[[nodiscard]] constexpr const char* toString(ExceptionCode code) noexcept {
    switch (code) {
        case ExceptionCode::IllegalFunction: return "Illegal function";
        case ExceptionCode::IllegalDataAddress: return "Illegal data address";
        case ExceptionCode::IllegalDataValue: return "Illegal data value";
        case ExceptionCode::ServerDeviceFailure: return "Server device failure";
        case ExceptionCode::GatewayTargetFailedToRespond: return "Gateway target failed to respond";
    }
    return "Unknown exception";
}

/// Modbus Application Protocol header that prefixes every Modbus TCP frame.
struct MbapHeader {
    std::uint16_t transactionId{0};
    std::uint16_t protocolId{kProtocolIdentifier};
    std::uint16_t length{0};  ///< Number of bytes that follow: unit id + PDU.
    std::uint8_t unitId{0};
};

/// The device answered with a valid Modbus exception response.
class ModbusException : public std::runtime_error {
public:
    explicit ModbusException(ExceptionCode code)
        : std::runtime_error(std::string("Modbus exception: ") + toString(code)), code_(code) {}

    [[nodiscard]] ExceptionCode code() const noexcept { return code_; }

private:
    ExceptionCode code_;
};

/// The peer sent a frame that violates the protocol (bad header, length, ids...).
class ProtocolError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Modbus transmits every 16-bit field in big-endian (network) byte order.
[[nodiscard]] constexpr std::uint16_t readBigEndian16(const std::uint8_t* source) noexcept {
    return static_cast<std::uint16_t>((static_cast<unsigned>(source[0]) << 8U) |
                                      static_cast<unsigned>(source[1]));
}

constexpr void writeBigEndian16(std::uint8_t* destination, std::uint16_t value) noexcept {
    destination[0] = static_cast<std::uint8_t>(value >> 8U);
    destination[1] = static_cast<std::uint8_t>(value & 0xFFU);
}

[[nodiscard]] inline std::array<std::uint8_t, kMbapHeaderSize> encodeHeader(
    const MbapHeader& header) noexcept {
    std::array<std::uint8_t, kMbapHeaderSize> bytes{};
    writeBigEndian16(&bytes[0], header.transactionId);
    writeBigEndian16(&bytes[2], header.protocolId);
    writeBigEndian16(&bytes[4], header.length);
    bytes[6] = header.unitId;
    return bytes;
}

[[nodiscard]] inline MbapHeader decodeHeader(const std::uint8_t* source) noexcept {
    MbapHeader header;
    header.transactionId = readBigEndian16(&source[0]);
    header.protocolId = readBigEndian16(&source[2]);
    header.length = readBigEndian16(&source[4]);
    header.unitId = source[6];
    return header;
}

constexpr std::size_t kReadRequestFrameSize = kMbapHeaderSize + 5;

/// Builds a complete "Read Holding Registers" request frame (MBAP header + PDU).
[[nodiscard]] inline std::array<std::uint8_t, kReadRequestFrameSize> buildReadHoldingRegistersRequest(
    std::uint16_t transactionId, std::uint8_t unitId, std::uint16_t startAddress,
    std::uint16_t quantity) noexcept {
    std::array<std::uint8_t, kReadRequestFrameSize> frame{};
    const auto header = encodeHeader(MbapHeader{transactionId, kProtocolIdentifier, 6, unitId});
    std::copy(header.begin(), header.end(), frame.begin());
    frame[7] = static_cast<std::uint8_t>(FunctionCode::ReadHoldingRegisters);
    writeBigEndian16(&frame[8], startAddress);
    writeBigEndian16(&frame[10], quantity);
    return frame;
}

}  // namespace modbus
