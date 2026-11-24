// Internal header: shared by http.cpp and websocket_conn.cpp. Not part of the public API.
#pragma once
#include <Moss/Moss_Network.h>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace moss_net {

/** Monotonic milliseconds (64-bit, never wraps in practice). */
uint64_t now_ms();

// ---- URLs ----
struct Url {
    std::string scheme;     // lower case: http, https, ws, wss
    std::string host;       // without brackets
    uint16_t    port = 0;
    std::string path;       // path + query, always starts with '/'
    bool        tls = false;
};

/** Strict parser for http/https/ws/wss URLs. Rejects control characters, spaces, non-ASCII bytes, user:password@ credentials
 *  and malformed hosts / ports. The fragment is dropped. Returns false when the URL is not acceptable. */
bool parse_url(const std::string& text, Url& out);
/** "scheme://host[:port]" with the port omitted when it is the default. Used to compare origins. */
std::string origin_of(const Url& url);
/** Value for the Host header: host[:port], IPv6 literals bracketed, default port omitted. */
std::string host_header_of(const Url& url);
bool is_loopback_host(const std::string& host);

// ---- text helpers ----
bool ci_equals(const std::string& a, const char* b);
bool ci_equals(const char* a, const char* b);
/** True if the comma separated header value contains `token` (case-insensitive), e.g. "keep-alive, Upgrade". */
bool has_token(const std::string& headerValue, const char* token);
bool valid_header_name(const char* name);
bool valid_header_value(const char* value);
std::string trim(const std::string& s);

// ---- HTTP head parsing (shared by the HTTP client and the WebSocket handshakes) ----
struct Header { std::string name, value; };
/** "HTTP/1.x SSS [reason]". */
bool parse_status_line(const std::string& line, int& status);
/** Parses "Name: value" lines separated by CRLF, starting at `start` in `block`. Strict: valid token names, no control characters,
 *  no obsolete line folding, at most 200 headers. */
bool parse_header_lines(const std::string& block, size_t start, std::vector<Header>& out);
const char* find_header_value(const std::vector<Header>& headers, const char* name);

// ---- connections ----
struct Stream {
    ENetSocket sock = ENET_SOCKET_NULL;
    void* tls = nullptr;
    const ENetTlsBackend* backend = nullptr;
    bool is_open() const { return sock != ENET_SOCKET_NULL; }
    void close();
};

/** DNS + TCP (+ TLS when url.tls). The socket is left NON-BLOCKING. `cancel` may be null. Plaintext policy is the caller's job. */
ENetHttpError stream_connect(Stream& s, const Url& url, const ENetTransportSecurity* security, uint64_t deadlineMs, const std::atomic<bool>* cancel);

/** Non-blocking: >0 bytes; 0 = would block; -1 = clean end of stream; -2 = error. */
int stream_read_nb(Stream& s, void* buf, size_t len);
/** Non-blocking: >0 bytes written; 0 = would block; <0 = error. */
int stream_write_nb(Stream& s, const void* data, size_t len);

enum : int { IO_EOF = 0, IO_ERROR = -1, IO_TIMEOUT = -2, IO_CANCELED = -3 };
/** Blocking with a deadline: >0 bytes read, or IO_EOF (clean end of stream) / IO_ERROR / IO_TIMEOUT / IO_CANCELED. */
int stream_read(Stream& s, void* buf, size_t len, uint64_t deadlineMs, const std::atomic<bool>* cancel);
/** Blocking with a deadline: 0 on success, or IO_ERROR / IO_TIMEOUT / IO_CANCELED. */
int stream_write_all(Stream& s, const void* data, size_t len, uint64_t deadlineMs, const std::atomic<bool>* cancel);

/** Sleeps up to `ms`, waking early when `cancel` becomes true. Returns false if canceled. */
bool sleep_cancelable(uint32_t ms, const std::atomic<bool>* cancel);

// ---- shared worker pool (implemented in http.cpp) ----
/** Runs `job` on the HTTP worker pool (started lazily). Returns false if the pool is shutting down. */
bool pool_submit(std::function<void()> job);
std::string default_user_agent();

} // namespace moss_net
