#include "TcpSocket.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#endif

#include <memory>
#include <system_error>
#include <utility>

namespace net {
namespace {

#ifdef _WIN32
using SockLen = int;

SOCKET toOs(NativeSocket handle) noexcept { return static_cast<SOCKET>(handle); }
int lastErrorCode() noexcept { return ::WSAGetLastError(); }
bool isTimeoutError(int code) noexcept { return code == WSAETIMEDOUT || code == WSAEWOULDBLOCK; }
bool isConnectInProgress(int code) noexcept { return code == WSAEWOULDBLOCK || code == WSAEINPROGRESS; }
void closeOs(NativeSocket handle) noexcept { ::closesocket(toOs(handle)); }
constexpr int kSendFlags = 0;
#else
using SockLen = socklen_t;

int toOs(NativeSocket handle) noexcept { return handle; }
int lastErrorCode() noexcept { return errno; }
bool isTimeoutError(int code) noexcept { return code == EAGAIN || code == EWOULDBLOCK; }
bool isConnectInProgress(int code) noexcept { return code == EINPROGRESS; }
void closeOs(NativeSocket handle) noexcept { ::close(handle); }
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;  // Never raise SIGPIPE on a broken connection.
#else
constexpr int kSendFlags = 0;
#endif
#endif

[[nodiscard]] std::string describe(const std::string& action, int code) {
    return action + ": " + std::system_category().message(code) + " (code " +
           std::to_string(code) + ")";
}

struct AddrInfoDeleter {
    void operator()(addrinfo* info) const noexcept { ::freeaddrinfo(info); }
};
using AddrInfoPtr = std::unique_ptr<addrinfo, AddrInfoDeleter>;

[[nodiscard]] AddrInfoPtr resolve(const std::string& host, std::uint16_t port, bool passive) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = passive ? AI_PASSIVE : 0;

    addrinfo* raw = nullptr;
    const std::string service = std::to_string(port);
    const char* node = host.empty() ? nullptr : host.c_str();
    if (::getaddrinfo(node, service.c_str(), &hints, &raw) != 0 || raw == nullptr) {
        throw NetworkError("Unable to resolve address '" + host + ":" + service + "'");
    }
    return AddrInfoPtr(raw);
}

void setNonBlocking(NativeSocket handle, bool enable) {
#ifdef _WIN32
    u_long mode = enable ? 1UL : 0UL;
    if (::ioctlsocket(toOs(handle), FIONBIO, &mode) != 0) {
        throw NetworkError(describe("ioctlsocket(FIONBIO) failed", lastErrorCode()));
    }
#else
    int flags = ::fcntl(handle, F_GETFL, 0);
    if (flags < 0) {
        throw NetworkError(describe("fcntl(F_GETFL) failed", lastErrorCode()));
    }
    flags = enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    if (::fcntl(handle, F_SETFL, flags) != 0) {
        throw NetworkError(describe("fcntl(F_SETFL) failed", lastErrorCode()));
    }
#endif
}

void enableNoDelay(NativeSocket handle) noexcept {
    // Modbus frames are tiny request/response pairs; Nagle's algorithm only adds latency.
    const int enabled = 1;
    ::setsockopt(toOs(handle), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&enabled),
                 static_cast<SockLen>(sizeof(enabled)));
}

[[nodiscard]] timeval toTimeval(std::chrono::milliseconds timeout) noexcept {
    const auto millis = timeout.count() < 0 ? 0 : timeout.count();
    timeval tv{};
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(millis / 1000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>((millis % 1000) * 1000);
    return tv;
}

enum class WaitMode { Read, Write };

/// Returns true when the socket became ready, false on timeout.
[[nodiscard]] bool waitFor(NativeSocket handle, WaitMode mode, std::chrono::milliseconds timeout) {
    fd_set readySet;
    fd_set errorSet;
    FD_ZERO(&readySet);
    FD_ZERO(&errorSet);
    FD_SET(toOs(handle), &readySet);
    FD_SET(toOs(handle), &errorSet);  // Winsock reports failed connects through this set.

    timeval tv = toTimeval(timeout);
#ifdef _WIN32
    const int highestDescriptor = 0;  // Ignored by Winsock.
#else
    const int highestDescriptor = handle + 1;
#endif
    fd_set* readSet = mode == WaitMode::Read ? &readySet : nullptr;
    fd_set* writeSet = mode == WaitMode::Write ? &readySet : nullptr;

    const int result = ::select(highestDescriptor, readSet, writeSet, &errorSet, &tv);
    if (result < 0) {
        throw NetworkError(describe("select() failed", lastErrorCode()));
    }
    return result > 0;
}

}  // namespace

// ---------------------------------------------------------------------------
// NetworkRuntime
// ---------------------------------------------------------------------------

NetworkRuntime::NetworkRuntime() {
#ifdef _WIN32
    WSADATA data{};
    const int result = ::WSAStartup(MAKEWORD(2, 2), &data);
    if (result != 0) {
        throw NetworkError(describe("WSAStartup failed", result));
    }
#endif
}

NetworkRuntime::~NetworkRuntime() {
#ifdef _WIN32
    ::WSACleanup();
#endif
}

// ---------------------------------------------------------------------------
// TcpSocket
// ---------------------------------------------------------------------------

TcpSocket::TcpSocket(NativeSocket handle) noexcept : handle_(handle) {}

TcpSocket::~TcpSocket() { close(); }

TcpSocket::TcpSocket(TcpSocket&& other) noexcept
    : handle_(std::exchange(other.handle_, kInvalidSocket)) {}

TcpSocket& TcpSocket::operator=(TcpSocket&& other) noexcept {
    if (this != &other) {
        close();
        handle_ = std::exchange(other.handle_, kInvalidSocket);
    }
    return *this;
}

TcpSocket TcpSocket::connect(const std::string& host, std::uint16_t port,
                             std::chrono::milliseconds timeout) {
    const AddrInfoPtr info = resolve(host, port, false);

    // Wrap the raw handle immediately so every early exit below closes it.
    TcpSocket socket(static_cast<NativeSocket>(
        ::socket(info->ai_family, info->ai_socktype, info->ai_protocol)));
    if (!socket.isValid()) {
        throw NetworkError(describe("socket() failed", lastErrorCode()));
    }

    // A non-blocking connect followed by select() gives us a bounded connect time.
    setNonBlocking(socket.handle_, true);
    const std::string endpoint = host + ":" + std::to_string(port);
    if (::connect(toOs(socket.handle_), info->ai_addr, static_cast<SockLen>(info->ai_addrlen)) != 0) {
        const int code = lastErrorCode();
        if (!isConnectInProgress(code)) {
            throw NetworkError(describe("Connection to " + endpoint + " failed", code));
        }
        if (!waitFor(socket.handle_, WaitMode::Write, timeout)) {
            throw NetworkError("Connection to " + endpoint + " timed out");
        }
        int socketError = 0;
        SockLen length = static_cast<SockLen>(sizeof(socketError));
        ::getsockopt(toOs(socket.handle_), SOL_SOCKET, SO_ERROR,
                     reinterpret_cast<char*>(&socketError), &length);
        if (socketError != 0) {
            throw NetworkError(describe("Connection to " + endpoint + " failed", socketError));
        }
    }
    setNonBlocking(socket.handle_, false);
    enableNoDelay(socket.handle_);
    return socket;
}

TcpSocket TcpSocket::listen(const std::string& bindAddress, std::uint16_t port, int backlog) {
    const AddrInfoPtr info = resolve(bindAddress, port, true);

    TcpSocket socket(static_cast<NativeSocket>(
        ::socket(info->ai_family, info->ai_socktype, info->ai_protocol)));
    if (!socket.isValid()) {
        throw NetworkError(describe("socket() failed", lastErrorCode()));
    }

    const int enabled = 1;
#ifdef _WIN32
    // On Windows SO_REUSEADDR allows port hijacking; exclusive use is the safe equivalent.
    ::setsockopt(toOs(socket.handle_), SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
                 reinterpret_cast<const char*>(&enabled), static_cast<SockLen>(sizeof(enabled)));
#else
    ::setsockopt(socket.handle_, SOL_SOCKET, SO_REUSEADDR, &enabled,
                 static_cast<SockLen>(sizeof(enabled)));
#endif

    const std::string endpoint = bindAddress + ":" + std::to_string(port);
    if (::bind(toOs(socket.handle_), info->ai_addr, static_cast<SockLen>(info->ai_addrlen)) != 0) {
        throw NetworkError(describe("Unable to bind " + endpoint, lastErrorCode()));
    }
    if (::listen(toOs(socket.handle_), backlog) != 0) {
        throw NetworkError(describe("Unable to listen on " + endpoint, lastErrorCode()));
    }
    return socket;
}

TcpSocket TcpSocket::accept(std::chrono::milliseconds timeout) const {
    if (!waitFor(handle_, WaitMode::Read, timeout)) {
        return TcpSocket{};
    }
    TcpSocket client(static_cast<NativeSocket>(::accept(toOs(handle_), nullptr, nullptr)));
    if (!client.isValid()) {
        throw NetworkError(describe("accept() failed", lastErrorCode()));
    }
    enableNoDelay(client.handle_);
    return client;
}

bool TcpSocket::waitReadable(std::chrono::milliseconds timeout) const {
    return waitFor(handle_, WaitMode::Read, timeout);
}

void TcpSocket::setIoTimeout(std::chrono::milliseconds timeout) const {
#ifdef _WIN32
    const DWORD value = static_cast<DWORD>(timeout.count());
#else
    const timeval value = toTimeval(timeout);
#endif
    const auto* raw = reinterpret_cast<const char*>(&value);
    const auto size = static_cast<SockLen>(sizeof(value));
    if (::setsockopt(toOs(handle_), SOL_SOCKET, SO_RCVTIMEO, raw, size) != 0 ||
        ::setsockopt(toOs(handle_), SOL_SOCKET, SO_SNDTIMEO, raw, size) != 0) {
        throw NetworkError(describe("Unable to set socket timeout", lastErrorCode()));
    }
}

void TcpSocket::sendAll(const std::uint8_t* data, std::size_t size) const {
    std::size_t sent = 0;
    while (sent < size) {
        const auto result = ::send(toOs(handle_), reinterpret_cast<const char*>(data + sent),
#ifdef _WIN32
                                   static_cast<int>(size - sent),
#else
                                   size - sent,
#endif
                                   kSendFlags);
        if (result <= 0) {
            const int code = lastErrorCode();
            throw NetworkError(isTimeoutError(code) ? std::string("Send timed out")
                                                    : describe("send() failed", code));
        }
        sent += static_cast<std::size_t>(result);
    }
}

bool TcpSocket::receiveExact(std::uint8_t* data, std::size_t size) const {
    std::size_t received = 0;
    while (received < size) {
        const auto result = ::recv(toOs(handle_), reinterpret_cast<char*>(data + received),
#ifdef _WIN32
                                   static_cast<int>(size - received),
#else
                                   size - received,
#endif
                                   0);
        if (result == 0) {
            if (received == 0) {
                return false;  // Orderly shutdown between frames.
            }
            throw NetworkError("Connection closed in the middle of a frame");
        }
        if (result < 0) {
            const int code = lastErrorCode();
            throw NetworkError(isTimeoutError(code) ? std::string("Receive timed out")
                                                    : describe("recv() failed", code));
        }
        received += static_cast<std::size_t>(result);
    }
    return true;
}

std::uint16_t TcpSocket::localPort() const {
    sockaddr_in address{};
    SockLen length = static_cast<SockLen>(sizeof(address));
    if (::getsockname(toOs(handle_), reinterpret_cast<sockaddr*>(&address), &length) != 0) {
        throw NetworkError(describe("getsockname() failed", lastErrorCode()));
    }
    return ntohs(address.sin_port);
}

bool TcpSocket::isValid() const noexcept { return handle_ != kInvalidSocket; }

void TcpSocket::close() noexcept {
    if (isValid()) {
        closeOs(handle_);
        handle_ = kInvalidSocket;
    }
}

}  // namespace net
