/**
 @file  net_stream.cpp
 @brief Internal: URL parsing, header validation and deadline-aware TCP/TLS streams for the HTTP and WebSocket layers.
*/
#define ENET_BUILDING_LIB 1
#include <string.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <Moss/Moss_Network.h>
#include "net_stream.h"

// ------------------------------------------------------------------
// TLS backend registry (public API)
// ------------------------------------------------------------------
static std::atomic<const ENetTlsBackend*> g_tlsBackend{nullptr};

void enet_tls_set_backend(const ENetTlsBackend* backend) { g_tlsBackend.store(backend); }
const ENetTlsBackend* enet_tls_get_backend(void) { return g_tlsBackend.load(); }

namespace moss_net {

uint64_t now_ms() {
    return (uint64_t) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ------------------------------------------------------------------
// text helpers
// ------------------------------------------------------------------
static char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char) (c - 'A' + 'a') : c; }

bool ci_equals(const char* a, const char* b) {
    while (*a && *b) { if (lower(*a) != lower(*b)) return false; ++a; ++b; }
    return *a == *b;
}
bool ci_equals(const std::string& a, const char* b) { return ci_equals(a.c_str(), b); }

std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && (s[b] == ' ' || s[b] == '\t')) ++b;
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t')) --e;
    return s.substr(b, e - b);
}

bool has_token(const std::string& value, const char* token) {
    size_t i = 0;
    while (i <= value.size()) {
        size_t comma = value.find(',', i);
        if (comma == std::string::npos) comma = value.size();
        if (ci_equals(trim(value.substr(i, comma - i)), token)) return true;
        i = comma + 1;
    }
    return false;
}

bool valid_header_name(const char* n) {
    if (!n || !*n) return false;
    for (; *n; ++n) {
        unsigned char c = (unsigned char) *n;
        bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || strchr("!#$%&'*+-.^_`|~", c);
        if (!ok) return false;
    }
    return true;
}

bool valid_header_value(const char* v) {
    if (!v) return false;
    for (; *v; ++v) {
        unsigned char c = (unsigned char) *v;
        if (c == '\t') continue;
        if (c < 0x20 || c == 0x7f) return false;   // CR, LF, NUL and other control characters: header injection / smuggling
    }
    return true;
}

// ------------------------------------------------------------------
// HTTP head parsing
// ------------------------------------------------------------------
bool parse_status_line(const std::string& status, int& out) {
    if (status.size() < 12 || status.compare(0, 7, "HTTP/1.") != 0 || (status[7] != '0' && status[7] != '1') || status[8] != ' ') return false;
    for (int i = 9; i < 12; ++i) if (status[i] < '0' || status[i] > '9') return false;
    if (status.size() > 12 && status[12] != ' ') return false;
    out = (status[9] - '0') * 100 + (status[10] - '0') * 10 + (status[11] - '0');
    return true;
}

bool parse_header_lines(const std::string& block, size_t p, std::vector<Header>& out) {
    out.clear();
    while (p < block.size()) {
        size_t e = block.find("\r\n", p);
        if (e == std::string::npos) e = block.size();
        std::string line = block.substr(p, e - p);
        p = e + 2;
        if (line.empty() || line[0] == ' ' || line[0] == '\t') return false;          // obs-fold / blank line inside the block
        size_t colon = line.find(':');
        if (colon == std::string::npos || colon == 0) return false;
        Header kv;
        kv.name = line.substr(0, colon);
        kv.value = trim(line.substr(colon + 1));
        if (!valid_header_name(kv.name.c_str()) || !valid_header_value(kv.value.c_str())) return false;
        out.push_back(std::move(kv));
        if (out.size() > 200) return false;
    }
    return true;
}

const char* find_header_value(const std::vector<Header>& headers, const char* name) {
    for (const Header& h : headers) if (ci_equals(h.name, name)) return h.value.c_str();
    return nullptr;
}

// ------------------------------------------------------------------
// URLs
// ------------------------------------------------------------------
bool is_loopback_host(const std::string& h) {
    if (ci_equals(h, "localhost")) return true;
    if (h == "::1") return true;
    if (h.compare(0, 4, "127.") == 0) {                        // 127.0.0.0/8, digits and dots only
        for (char c : h) if (!((c >= '0' && c <= '9') || c == '.')) return false;
        return true;
    }
    return false;
}

static bool valid_host_char(char c) {
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-' || c == '.' || c == '_';
}

bool parse_url(const std::string& text, Url& out) {
    if (text.empty() || text.size() > 8192) return false;
    for (unsigned char c : text) if (c <= 0x20 || c >= 0x7f) return false;   // controls, space, non-ASCII: must be percent-encoded

    size_t schemeEnd = text.find("://");
    if (schemeEnd == std::string::npos) return false;
    Url u;
    u.scheme = text.substr(0, schemeEnd);
    for (char& c : u.scheme) c = lower(c);
    if (u.scheme == "http" || u.scheme == "ws") { u.tls = false; u.port = 80; }
    else if (u.scheme == "https" || u.scheme == "wss") { u.tls = true; u.port = 443; }
    else return false;

    size_t authStart = schemeEnd + 3;
    size_t authEnd = text.find_first_of("/?#", authStart);
    if (authEnd == std::string::npos) authEnd = text.size();
    std::string auth = text.substr(authStart, authEnd - authStart);
    if (auth.empty() || auth.find('@') != std::string::npos) return false;   // no user:password@host (credentials in URLs leak into logs)

    std::string portText;
    if (auth[0] == '[') {                                                   // IPv6 literal
        size_t close = auth.find(']');
        if (close == std::string::npos || close < 3) return false;
        u.host = auth.substr(1, close - 1);
        for (char c : u.host) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F') || c == ':' || c == '.')) return false;
        if (close + 1 < auth.size()) { if (auth[close + 1] != ':') return false; portText = auth.substr(close + 2); if (portText.empty()) return false; }
    } else {
        size_t colon = auth.find(':');
        u.host = auth.substr(0, colon);
        if (colon != std::string::npos) { portText = auth.substr(colon + 1); if (portText.empty()) return false; }
        if (u.host.empty()) return false;
        for (char c : u.host) if (!valid_host_char(c)) return false;
    }
    if (!portText.empty()) {
        unsigned long port = 0;
        for (char c : portText) { if (c < '0' || c > '9') return false; port = port * 10 + (unsigned long) (c - '0'); if (port > 65535) return false; }
        if (port == 0) return false;
        u.port = (uint16_t) port;
    }

    std::string rest = text.substr(authEnd);
    size_t frag = rest.find('#');
    if (frag != std::string::npos) rest.resize(frag);
    if (rest.empty() || rest[0] == '?') rest = "/" + rest;
    u.path = rest;
    out = u;
    return true;
}

std::string host_header_of(const Url& u) {
    std::string h = (u.host.find(':') != std::string::npos) ? "[" + u.host + "]" : u.host;
    uint16_t def = u.tls ? 443 : 80;
    if (u.port != def) h += ":" + std::to_string(u.port);
    return h;
}

std::string origin_of(const Url& u) { return u.scheme + "://" + host_header_of(u); }

// ------------------------------------------------------------------
// Streams
// ------------------------------------------------------------------
void Stream::close() {
    if (tls && backend) backend->close(tls);
    tls = nullptr;
    if (sock != ENET_SOCKET_NULL) enet_socket_destroy(sock);
    sock = ENET_SOCKET_NULL;
}

bool sleep_cancelable(uint32_t ms, const std::atomic<bool>* cancel) {
    uint64_t end = now_ms() + ms;
    for (;;) {
        if (cancel && cancel->load()) return false;
        uint64_t now = now_ms();
        if (now >= end) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds((unsigned) std::min<uint64_t>(end - now, 20)));
    }
}

// waits up to sliceMs for the socket; 0 = ready or timed out (caller re-checks), <0 = error
static int wait_socket(ENetSocket sock, bool forWrite, uint32_t sliceMs) {
    uint32_t cond = forWrite ? static_cast<uint32_t>(ENetSocketWait::SEND) : static_cast<uint32_t>(ENetSocketWait::RECEIVE);
    return enet_socket_wait(sock, &cond, sliceMs) < 0 ? -1 : 0;
}

static uint32_t slice(uint64_t deadline, uint64_t now) {
    uint64_t left = deadline > now ? deadline - now : 0;
    return (uint32_t) std::min<uint64_t>(left, 50);       // short slices keep cancellation responsive
}

static ENetHttpError connect_address(Stream& s, const ENetAddress& addr, uint64_t deadline, const std::atomic<bool>* cancel) {
    ENetSocket sock = enet_socket_create(addr.type, ENetSocketType::STREAM);
    if (sock == ENET_SOCKET_NULL) return ENetHttpError::ENET_HTTP_ERR_CONNECT_FAILED;
    enet_socket_set_option(sock, ENetSocketOption::NET_OPT_NONBLOCK, 1);
    enet_socket_set_option(sock, ENetSocketOption::NET_OPT_NODELAY, 1);
    // A WebSocket / HTTP connection should be IPv6-only when it is an IPv6 socket (no dual-stack surprises).
    if (addr.type == ENetAddressType::IPV6) enet_socket_set_option(sock, ENetSocketOption::NET_OPT_IPV6ONLY, 1);

    if (enet_socket_connect(sock, &addr) != 0) { enet_socket_destroy(sock); return ENetHttpError::ENET_HTTP_ERR_CONNECT_FAILED; }
    for (;;) {                                  // (on Windows a refused connect shows up as a timeout here; the deadline bounds it)
        if (cancel && cancel->load()) { enet_socket_destroy(sock); return ENetHttpError::ENET_HTTP_ERR_CANCELED; }
        uint64_t now = now_ms();
        if (now >= deadline) { enet_socket_destroy(sock); return ENetHttpError::ENET_HTTP_ERR_TIMEOUT; }
        uint32_t cond = static_cast<uint32_t>(ENetSocketWait::SEND);
        if (enet_socket_wait(sock, &cond, slice(deadline, now)) < 0) { enet_socket_destroy(sock); return ENetHttpError::ENET_HTTP_ERR_CONNECT_FAILED; }
        if (cond & ENetSocketWait::SEND) break;
    }
    int err = 0;
    if (enet_socket_get_option(sock, ENetSocketOption::NET_OPT_ERROR, &err) != 0 || err != 0) { enet_socket_destroy(sock); return ENetHttpError::ENET_HTTP_ERR_CONNECT_FAILED; }
    s.sock = sock;
    return ENetHttpError::ENET_HTTP_OK;
}

ENetHttpError stream_connect(Stream& s, const Url& url, const ENetTransportSecurity* security, uint64_t deadline, const std::atomic<bool>* cancel) {
    using E = ENetHttpError;
    const ENetTlsBackend* backend = nullptr;
    if (url.tls) {
        backend = enet_tls_get_backend();
        if (!backend) return E::ENET_HTTP_ERR_TLS_FAILED;       // fail closed: never fall back to plaintext
    }

    ENetAddress addr;
    memset(&addr, 0, sizeof(addr));
    addr.port = url.port;
    if (enet_address_set_host(&addr, ENetAddressType::ANY, url.host.c_str()) != 0) return E::ENET_HTTP_ERR_DNS_FAILURE;

    E rc = connect_address(s, addr, deadline, cancel);
    if (rc != E::ENET_HTTP_OK && rc != E::ENET_HTTP_ERR_CANCELED && addr.type == ENetAddressType::IPV6) {
        ENetAddress v4;                                          // simple fallback when the IPv6 route is dead
        memset(&v4, 0, sizeof(v4));
        v4.port = url.port;
        if (enet_address_set_host(&v4, ENetAddressType::IPV4, url.host.c_str()) == 0 && v4.type == ENetAddressType::IPV4)
            rc = connect_address(s, v4, deadline, cancel);
    }
    if (rc != E::ENET_HTTP_OK) return rc;

    if (backend) {
        uint64_t now = now_ms();
        uint32_t remaining = deadline > now ? (uint32_t) std::min<uint64_t>(deadline - now, 0xFFFFFFFFu) : 1;
        s.backend = backend;
        s.tls = backend->connect(s.sock, url.host.c_str(), security, remaining);
        if (!s.tls) { s.close(); return E::ENET_HTTP_ERR_TLS_FAILED; }
    }
    return E::ENET_HTTP_OK;
}

int stream_read_nb(Stream& s, void* buf, size_t len) {
    if (s.tls) return s.backend->read(s.tls, buf, len);
    return enet_socket_receive_stream(s.sock, buf, len);
}

int stream_write_nb(Stream& s, const void* data, size_t len) {
    if (s.tls) return s.backend->write(s.tls, data, len);
    return enet_socket_send_stream(s.sock, data, len);
}

int stream_read(Stream& s, void* buf, size_t len, uint64_t deadline, const std::atomic<bool>* cancel) {
    for (;;) {
        if (cancel && cancel->load()) return IO_CANCELED;
        int r = stream_read_nb(s, buf, len);
        if (r > 0) return r;
        if (r == -1) return IO_EOF;
        if (r < -1) return IO_ERROR;
        uint64_t now = now_ms();
        if (now >= deadline) return IO_TIMEOUT;
        if (wait_socket(s.sock, false, slice(deadline, now)) < 0) return IO_ERROR;
    }
}

int stream_write_all(Stream& s, const void* data, size_t len, uint64_t deadline, const std::atomic<bool>* cancel) {
    const uint8_t* p = (const uint8_t*) data;
    while (len > 0) {
        if (cancel && cancel->load()) return IO_CANCELED;
        int w = stream_write_nb(s, p, len);
        if (w > 0) { p += w; len -= (size_t) w; continue; }
        if (w < 0) return IO_ERROR;
        uint64_t now = now_ms();
        if (now >= deadline) return IO_TIMEOUT;
        if (wait_socket(s.sock, true, slice(deadline, now)) < 0) return IO_ERROR;
    }
    return 0;
}

} // namespace moss_net
