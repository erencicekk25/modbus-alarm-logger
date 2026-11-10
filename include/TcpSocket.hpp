#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace net {

/// Raised for every socket level failure: resolve, connect, send, receive or timeout.
class NetworkError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

#ifdef _WIN32
using NativeSocket = std::uintptr_t;  // Matches the width of the Winsock SOCKET type.
#else
using NativeSocket = int;
#endif

/// RAII guard for the platform socket library.
/// On Windows it pairs WSAStartup/WSACleanup; on POSIX systems it is a no-op.
/// Winsock keeps an internal reference count, so several guards may coexist safely.
class NetworkRuntime {
public:
    NetworkRuntime();
    ~NetworkRuntime();

    NetworkRuntime(const NetworkRuntime&) = delete;
    NetworkRuntime& operator=(const NetworkRuntime&) = delete;
    NetworkRuntime(NetworkRuntime&&) = delete;
    NetworkRuntime& operator=(NetworkRuntime&&) = delete;
};

/// Move-only owner of a blocking IPv4 TCP socket. The handle is closed automatically
/// when the object goes out of scope, so no code path can leak a descriptor.
class TcpSocket {
public:
    TcpSocket() noexcept = default;
    ~TcpSocket();

    TcpSocket(const TcpSocket&) = delete;
    TcpSocket& operator=(const TcpSocket&) = delete;
    TcpSocket(TcpSocket&& other) noexcept;
    TcpSocket& operator=(TcpSocket&& other) noexcept;

    /// Opens a client connection, failing if it is not established within `timeout`.
    [[nodiscard]] static TcpSocket connect(const std::string& host, std::uint16_t port,
                                           std::chrono::milliseconds timeout);

    /// Creates a listening socket. Passing port 0 lets the operating system pick a free port.
    [[nodiscard]] static TcpSocket listen(const std::string& bindAddress, std::uint16_t port,
                                          int backlog = 4);

    /// Waits up to `timeout` for an incoming connection. Returns an invalid socket on timeout.
    [[nodiscard]] TcpSocket accept(std::chrono::milliseconds timeout) const;

    /// Returns true when data (or an orderly shutdown) is ready to be read.
    [[nodiscard]] bool waitReadable(std::chrono::milliseconds timeout) const;

    /// Applies the same timeout to blocking send and receive calls.
    void setIoTimeout(std::chrono::milliseconds timeout) const;

    /// Sends the whole buffer or throws.
    void sendAll(const std::uint8_t* data, std::size_t size) const;

    /// Reads exactly `size` bytes. Returns false if the peer closed the connection
    /// cleanly before the first byte arrived; throws on any other failure.
    [[nodiscard]] bool receiveExact(std::uint8_t* data, std::size_t size) const;

    [[nodiscard]] std::uint16_t localPort() const;
    [[nodiscard]] bool isValid() const noexcept;

    void close() noexcept;

private:
    explicit TcpSocket(NativeSocket handle) noexcept;

#ifdef _WIN32
    static constexpr NativeSocket kInvalidSocket = ~NativeSocket{0};
#else
    static constexpr NativeSocket kInvalidSocket = -1;
#endif

    NativeSocket handle_{kInvalidSocket};
};

}  // namespace net
