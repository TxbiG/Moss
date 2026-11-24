/**
 @file  websocket_conn.cpp
 @brief Moss_Network WebSocket connections: RFC 6455 client and server upgrade handshakes over TCP/TLS streams, and a polled
        connection object that bridges to the frame layer in websocket.cpp.
*/
#define ENET_BUILDING_LIB 1
#include <string.h>
#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <Moss/Moss_Network.h>
#include "net_stream.h"

using namespace moss_net;
using E = ENetHttpError;

static const char kWsGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// ------------------------------------------------------------------
// handshake primitives (public)
// ------------------------------------------------------------------
int enet_ws_generate_client_key(char out[25]) {
    uint8_t r[16];
    if (enet_random_bytes(r, sizeof(r)) != 0) return -1;
    return enet_base64_encode(r, sizeof(r), out, 25) == 24 ? 0 : -1;
}

int enet_ws_compute_accept_key(const char* key, char out[29]) {
    if (!key) return -1;
    size_t n = strlen(key);
    if (n == 0 || n > 64) return -1;
    char buf[64 + sizeof(kWsGuid)];
    memcpy(buf, key, n);
    memcpy(buf + n, kWsGuid, sizeof(kWsGuid) - 1);
    uint8_t digest[20];
    enet_sha1(buf, n + sizeof(kWsGuid) - 1, digest);
    return enet_base64_encode(digest, 20, out, 29) == 28 ? 0 : -1;
}

// ------------------------------------------------------------------
// the connection object
// ------------------------------------------------------------------
namespace {

struct Ev {
    ENetWebSocketEventType type = ENetWebSocketEventType::FAILURE;
    std::vector<uint8_t> data;
    uint16_t code = 0;
    E error = E::ENET_HTTP_OK;
    int status = 0;
    std::string subprotocol;
};

struct ConnectJob {                         // shared between the worker and the connection object
    std::mutex m;
    std::atomic<bool> cancel{false};
    bool done = false;
    E error = E::ENET_HTTP_OK;
    int httpStatus = 0;
    Stream stream;
    std::string leftover, subprotocol;
};

struct ClientParams {
    std::string url, subprotocols, origin;
    std::vector<Header> headers;
    bool hasTls = false, skipVerify = false;
    std::string certFile, keyFile, caFile;
    uint32_t timeoutMs = 10000, flags = 0;
};

bool is_token(const std::string& s) { return !s.empty() && valid_header_name(s.c_str()); }

bool split_tokens(const std::string& list, std::vector<std::string>& out) {
    out.clear();
    size_t i = 0;
    while (i <= list.size()) {
        size_t comma = list.find(',', i);
        if (comma == std::string::npos) comma = list.size();
        std::string t = trim(list.substr(i, comma - i));
        if (!is_token(t)) return false;
        out.push_back(t);
        i = comma + 1;
    }
    return !out.empty();
}

bool forbidden_ws_header(const std::string& n) {
    static const char* bad[] = {"host", "upgrade", "connection", "origin", "content-length", "transfer-encoding", "te", "expect", "keep-alive", "proxy-connection", nullptr};
    for (int i = 0; bad[i]; ++i) if (ci_equals(n, bad[i])) return true;
    return n.size() >= 14 && ci_equals(n.substr(0, 14), "sec-websocket-");
}

// ---- client handshake (runs on a worker thread) ----
void client_connect(std::shared_ptr<ConnectJob> job, ClientParams p) {
    E result = E::ENET_HTTP_OK;
    int httpStatus = 0;
    Stream st;
    std::string leftover, chosen;

    do {
        Url url;
        if (!parse_url(p.url, url) || (url.scheme != "ws" && url.scheme != "wss")) { result = E::ENET_HTTP_ERR_INVALID_ARGUMENT; break; }
        if (!url.tls && !is_loopback_host(url.host) && !(p.flags & ENetHttpFlag::ALLOW_INSECURE)) { result = E::ENET_HTTP_ERR_INSECURE; break; }

        uint64_t deadline = now_ms() + p.timeoutMs;
        ENetTransportSecurity sec;
        sec.cert_file = p.certFile.empty() ? nullptr : p.certFile.c_str();
        sec.key_file = p.keyFile.empty() ? nullptr : p.keyFile.c_str();
        sec.ca_file = p.caFile.empty() ? nullptr : p.caFile.c_str();
        sec.skip_verification = p.skipVerify ? 1 : 0;
        result = stream_connect(st, url, p.hasTls ? &sec : nullptr, deadline, &job->cancel);
        if (result != E::ENET_HTTP_OK) break;

        char key[25];
        if (enet_ws_generate_client_key(key) != 0) { result = E::ENET_HTTP_ERR_INTERNAL; break; }
        char expect[29];
        enet_ws_compute_accept_key(key, expect);

        std::string req = "GET " + url.path + " HTTP/1.1\r\nHost: " + host_header_of(url) + "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n";
        req += std::string("Sec-WebSocket-Key: ") + key + "\r\nSec-WebSocket-Version: 13\r\n";
        if (!p.origin.empty()) req += "Origin: " + p.origin + "\r\n";
        if (!p.subprotocols.empty()) req += "Sec-WebSocket-Protocol: " + p.subprotocols + "\r\n";
        for (const Header& h : p.headers) req += h.name + ": " + h.value + "\r\n";
        req += "\r\n";
        int w = stream_write_all(st, req.data(), req.size(), deadline, &job->cancel);
        if (w != 0) { result = (w == IO_TIMEOUT) ? E::ENET_HTTP_ERR_TIMEOUT : (w == IO_CANCELED ? E::ENET_HTTP_ERR_CANCELED : E::ENET_HTTP_ERR_IO); break; }

        std::string buf;
        size_t end = std::string::npos;
        while ((end = buf.find("\r\n\r\n")) == std::string::npos) {
            if (buf.size() > 16384) { result = E::ENET_HTTP_ERR_INVALID_RESPONSE; break; }
            uint8_t tmp[2048];
            int r = stream_read(st, tmp, sizeof(tmp), deadline, &job->cancel);
            if (r > 0) { buf.append((const char*) tmp, (size_t) r); continue; }
            result = (r == IO_TIMEOUT) ? E::ENET_HTTP_ERR_TIMEOUT : (r == IO_CANCELED ? E::ENET_HTTP_ERR_CANCELED : (r == IO_EOF ? E::ENET_HTTP_ERR_INVALID_RESPONSE : E::ENET_HTTP_ERR_IO));
            break;
        }
        if (result != E::ENET_HTTP_OK) break;

        std::string block = buf.substr(0, end);
        leftover = buf.substr(end + 4);               // frames the server sent right behind its 101
        size_t eol = block.find("\r\n");
        std::vector<Header> hs;
        if (!parse_status_line(block.substr(0, eol), httpStatus) ||
            !parse_header_lines(block, eol == std::string::npos ? block.size() : eol + 2, hs)) { result = E::ENET_HTTP_ERR_INVALID_RESPONSE; break; }
        if (httpStatus != 101) { result = E::ENET_HTTP_ERR_HANDSHAKE_REJECTED; break; }       // e.g. 401 / 403: surfaced with http_status

        const char* up = find_header_value(hs, "upgrade");
        const char* cn = find_header_value(hs, "connection");
        const char* ac = find_header_value(hs, "sec-websocket-accept");
        if (!up || !ci_equals(trim(up), "websocket") || !cn || !has_token(cn, "upgrade") || !ac ||
            strlen(ac) != 28 || !enet_secure_equals(ac, expect, 28)) { result = E::ENET_HTTP_ERR_INVALID_RESPONSE; break; }
        if (find_header_value(hs, "sec-websocket-extensions")) { result = E::ENET_HTTP_ERR_INVALID_RESPONSE; break; }   // we offered none
        if (const char* sp = find_header_value(hs, "sec-websocket-protocol")) {
            std::vector<std::string> offered;
            if (p.subprotocols.empty() || !split_tokens(p.subprotocols, offered) || std::find(offered.begin(), offered.end(), trim(sp)) == offered.end()) {
                result = E::ENET_HTTP_ERR_INVALID_RESPONSE; break;                                                        // server picked something we never offered
            }
            chosen = trim(sp);
        } else if (!p.subprotocols.empty()) {
            chosen.clear();                                                                                              // allowed: server chose none
        }
    } while (false);

    std::lock_guard<std::mutex> g(job->m);
    job->done = true;
    job->error = result;
    job->httpStatus = httpStatus;
    if (result == E::ENET_HTTP_OK && !job->cancel.load()) {
        job->stream = st; job->leftover = leftover; job->subprotocol = chosen;
        st = Stream();                                      // ownership moved
    }
    st.close();
}

} // namespace

struct ENetWebSocket {
    bool server = false;
    ENetWebSocketState state = ENetWebSocketState::CONNECTING;
    Stream stream;
    enet_ws_event_context_ptr ctx = nullptr;

    std::deque<Ev> events;
    Ev current;

    std::string pre; size_t preOff = 0;     // bytes that arrived together with the handshake
    bool eof = false, ioError = false;

    size_t maxMsg = 1u << 20, maxQueued = 4u << 20;
    uint32_t pingMs = 0, idleMs = 0;
    uint64_t lastRecv = 0, lastPing = 0, closeDeadline = 0;
    bool closeEmitted = false;

    std::shared_ptr<ConnectJob> job;        // client handshake in flight

    // server handshake
    std::vector<std::string> allowedOrigins;
    bool requireOrigin = false;
    std::string supported;
    ENetWebSocketAuthorizeCallback authorize = nullptr;
    void* authUser = nullptr;
    uint64_t hsDeadline = 0;
    std::string hsIn, hsOut;
    size_t hsOutOff = 0;
    bool hsWriting = false, hsReject = false;
    int hsStatus = 0;
    std::string hsSubprotocol;
};

namespace {

void push(ENetWebSocket* ws, Ev ev) { ws->events.push_back(std::move(ev)); }

void fail(ENetWebSocket* ws, E error, int status = 0, uint16_t code = 0) {
    Ev e; e.type = ENetWebSocketEventType::FAILURE; e.error = error; e.status = status; e.code = code;
    push(ws, std::move(e));
    ws->state = ENetWebSocketState::CLOSED;
    ws->stream.close();
}

// ---- callbacks bridging to the frame layer ----
ENetSSize cb_recv(enet_ws_event_context_ptr ctx, uint8_t* buf, size_t len, int, void* user) {
    ENetWebSocket* ws = (ENetWebSocket*) user;
    if (ws->preOff < ws->pre.size()) {
        size_t n = std::min(len, ws->pre.size() - ws->preOff);
        memcpy(buf, ws->pre.data() + ws->preOff, n);
        ws->preOff += n;
        return (ENetSSize) n;
    }
    int r = stream_read_nb(ws->stream, buf, len);
    if (r > 0) { ws->lastRecv = now_ms(); return r; }
    if (r == 0) { enet_ws_event_set_error(ctx, ENET_WS_ERR_WOULDBLOCK); return -1; }
    if (r == -1) ws->eof = true; else ws->ioError = true;
    enet_ws_event_set_error(ctx, ENET_WS_ERR_CALLBACK_FAILURE);
    return -1;
}

ENetSSize cb_send(enet_ws_event_context_ptr ctx, const uint8_t* data, size_t len, int, void* user) {
    ENetWebSocket* ws = (ENetWebSocket*) user;
    int w = stream_write_nb(ws->stream, data, len);
    if (w > 0) return w;
    if (w == 0) { enet_ws_event_set_error(ctx, ENET_WS_ERR_WOULDBLOCK); return -1; }
    ws->ioError = true;
    enet_ws_event_set_error(ctx, ENET_WS_ERR_CALLBACK_FAILURE);
    return -1;
}

int cb_genmask(enet_ws_event_context_ptr, uint8_t* buf, size_t len, void*) { return enet_random_bytes(buf, len) == 0 ? 0 : -1; }

void cb_msg(enet_ws_event_context_ptr, const enet_ws_event_on_msg_recv_arg* a, void* user) {
    ENetWebSocket* ws = (ENetWebSocket*) user;
    Ev e;
    switch (a->opcode) {
        case ENET_WS_TEXT_FRAME: e.type = ENetWebSocketEventType::TEXT; break;
        case ENET_WS_BINARY_FRAME: e.type = ENetWebSocketEventType::BINARY; break;
        case ENET_WS_PONG: e.type = ENetWebSocketEventType::PONG; break;
        default: return;                       // pings are answered by the frame layer; close is reported once the handshake completes
    }
    if (a->msg && a->msg_length) e.data.assign(a->msg, a->msg + a->msg_length);
    push(ws, std::move(e));
}

bool start_framing(ENetWebSocket* ws) {
    enet_ws_event_callbacks cbs;
    memset(&cbs, 0, sizeof(cbs));
    cbs.recv_callback = cb_recv;
    cbs.send_callback = cb_send;
    cbs.genmask_callback = ws->server ? nullptr : cb_genmask;
    cbs.on_msg_recv_callback = cb_msg;
    int rc = ws->server ? enet_ws_event_context_server_init(&ws->ctx, &cbs, ws) : enet_ws_event_context_client_init(&ws->ctx, &cbs, ws);
    if (rc != 0) { ws->ctx = nullptr; return false; }
    enet_ws_event_config_set_max_recv_msg_length(ws->ctx, ws->maxMsg);
    ws->lastRecv = ws->lastPing = now_ms();
    ws->state = ENetWebSocketState::OPEN;
    return true;
}

void finalize_close(ENetWebSocket* ws) {
    if (ws->closeEmitted) return;
    ws->closeEmitted = true;
    Ev e; e.type = ENetWebSocketEventType::CLOSE;
    e.code = ws->ctx ? enet_ws_event_get_status_code_received(ws->ctx) : 1006;
    push(ws, std::move(e));
    ws->state = ENetWebSocketState::CLOSED;
    ws->stream.close();
}

// ---- open-connection pump ----
void pump_open(ENetWebSocket* ws) {
    uint64_t now = now_ms();

    if (ws->idleMs && now - ws->lastRecv >= ws->idleMs) { fail(ws, E::ENET_HTTP_ERR_TIMEOUT); return; }
    if (ws->state == ENetWebSocketState::CLOSING && ws->closeDeadline && now >= ws->closeDeadline) {
        ws->closeEmitted = true;
        Ev e; e.type = ENetWebSocketEventType::CLOSE; e.code = 1006;           // peer never completed the closing handshake
        push(ws, std::move(e)); ws->state = ENetWebSocketState::CLOSED; ws->stream.close(); return;
    }
    if (ws->pingMs && ws->state == ENetWebSocketState::OPEN && now - std::max(ws->lastPing, ws->lastRecv) >= ws->pingMs) {
        enet_ws_event_msg ping{ENET_WS_PING, nullptr, 0};
        if (enet_ws_event_queue_msg(ws->ctx, &ping) == 0) ws->lastPing = now;
    }

    int rr = 0;
    if (ws->events.size() < 256 && enet_ws_event_get_read_enabled(ws->ctx)) rr = enet_ws_event_recv(ws->ctx);   // backpressure: stop reading while the app lags
    int sr = enet_ws_event_send(ws->ctx);

    bool closeDone = enet_ws_event_get_close_received(ws->ctx) && enet_ws_event_get_close_sent(ws->ctx) && enet_ws_event_get_queued_msg_count(ws->ctx) == 0;
    if (closeDone) { finalize_close(ws); return; }

    // Lost connection comes first: when a read callback fails the frame layer queues a close frame of its own (and may even "send" it
    // into the dead socket), which must not be mistaken for a protocol violation by the peer.
    if (ws->eof || ws->ioError || rr < 0 || sr < 0) {
        if (enet_ws_event_get_close_received(ws->ctx)) { finalize_close(ws); return; }       // peer closed properly, our reply just could not be delivered
        if (ws->state == ENetWebSocketState::CLOSING) {                                       // we asked to close and the peer dropped the TCP connection
            ws->closeEmitted = true;
            Ev e; e.type = ENetWebSocketEventType::CLOSE; e.code = 1006;
            push(ws, std::move(e)); ws->state = ENetWebSocketState::CLOSED; ws->stream.close();
            return;
        }
        fail(ws, E::ENET_HTTP_ERR_IO, 0, 1006);                                               // abnormal closure: no closing handshake happened
        return;
    }

    // we sent a close because the PEER violated the protocol (reading is shut off by the frame layer): report why
    if (!enet_ws_event_get_read_enabled(ws->ctx) && enet_ws_event_get_close_sent(ws->ctx) && !enet_ws_event_get_close_received(ws->ctx) &&
        enet_ws_event_get_queued_msg_count(ws->ctx) == 0 && ws->state == ENetWebSocketState::OPEN) {
        uint16_t sent = enet_ws_event_get_status_code_sent(ws->ctx);
        fail(ws, sent == ENET_WS_CODE_MESSAGE_TOO_BIG ? E::ENET_HTTP_ERR_TOO_LARGE : E::ENET_HTTP_ERR_INVALID_RESPONSE, 0, sent);
        return;
    }
}

// ---- server handshake pump ----
const char* reason_phrase(int s) {
    switch (s) { case 400: return "Bad Request"; case 401: return "Unauthorized"; case 403: return "Forbidden"; case 405: return "Method Not Allowed";
                 case 426: return "Upgrade Required"; case 429: return "Too Many Requests"; case 431: return "Request Header Fields Too Large"; default: return "Error"; }
}

void hs_reject(ENetWebSocket* ws, int status) {
    ws->hsReject = true; ws->hsStatus = status;
    ws->hsOut = std::string("HTTP/1.1 ") + std::to_string(status) + " " + reason_phrase(status) + "\r\nContent-Length: 0\r\nConnection: close\r\n";
    if (status == 426) ws->hsOut += "Sec-WebSocket-Version: 13\r\n";
    ws->hsOut += "\r\n";
    ws->hsWriting = true;
}

void hs_process(ENetWebSocket* ws, const std::string& block) {
    size_t eol = block.find("\r\n");
    std::string line = block.substr(0, eol);
    std::vector<Header> hs;
    // request line: GET <target> HTTP/1.1
    size_t s1 = line.find(' '), s2 = (s1 == std::string::npos) ? std::string::npos : line.find(' ', s1 + 1);
    if (s1 == std::string::npos || s2 == std::string::npos) return hs_reject(ws, 400);
    std::string method = line.substr(0, s1), target = line.substr(s1 + 1, s2 - s1 - 1), version = line.substr(s2 + 1);
    if (!parse_header_lines(block, eol == std::string::npos ? block.size() : eol + 2, hs)) return hs_reject(ws, 400);
    if (method != "GET") return hs_reject(ws, 405);
    if (version != "HTTP/1.1" || target.empty() || target[0] != '/') return hs_reject(ws, 400);
    for (unsigned char c : target) if (c <= 0x20 || c >= 0x7f) return hs_reject(ws, 400);

    int keys = 0, hosts = 0;
    for (const Header& h : hs) { if (ci_equals(h.name, "sec-websocket-key")) ++keys; if (ci_equals(h.name, "host")) ++hosts; }
    const char* host = find_header_value(hs, "host");
    const char* up = find_header_value(hs, "upgrade");
    const char* cn = find_header_value(hs, "connection");
    const char* ver = find_header_value(hs, "sec-websocket-version");
    const char* key = find_header_value(hs, "sec-websocket-key");
    if (hosts != 1 || keys != 1 || !up || !has_token(up, "websocket") || !cn || !has_token(cn, "upgrade") || !key) return hs_reject(ws, 400);
    if (!ver || trim(ver) != "13") return hs_reject(ws, 426);
    uint8_t raw[16]; size_t rawLen = 0;
    if (strlen(key) != 24 || enet_base64_decode(key, 24, raw, sizeof(raw), &rawLen) != 0 || rawLen != 16) return hs_reject(ws, 400);

    const char* origin = find_header_value(hs, "origin");
    if (!ws->allowedOrigins.empty()) {                                  // blocks cross-site WebSocket hijacking from browsers
        if (origin) { if (std::find(ws->allowedOrigins.begin(), ws->allowedOrigins.end(), std::string(origin)) == ws->allowedOrigins.end()) return hs_reject(ws, 403); }
        else if (ws->requireOrigin) return hs_reject(ws, 403);
    }
    const char* offer = find_header_value(hs, "sec-websocket-protocol");

    if (ws->authorize) {
        std::vector<ENetHttpHeader> hv;
        for (const Header& h : hs) hv.push_back({h.name.c_str(), h.value.c_str()});
        ENetWebSocketRequest rq{target.c_str(), host ? host : "", origin ? origin : "", offer ? offer : "", hv.data(), hv.size()};
        int st = ws->authorize(&rq, ws->authUser);
        if (st != 0) return hs_reject(ws, (st >= 400 && st <= 599) ? st : 403);
    }

    std::string chosen;
    if (offer && !ws->supported.empty()) {
        std::vector<std::string> offered, mine;
        if (split_tokens(offer, offered) && split_tokens(ws->supported, mine))
            for (const std::string& o : offered) if (std::find(mine.begin(), mine.end(), o) != mine.end()) { chosen = o; break; }
    }
    char accept[29];
    if (enet_ws_compute_accept_key(key, accept) != 0) return hs_reject(ws, 400);
    ws->hsOut = std::string("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ") + accept + "\r\n";
    if (!chosen.empty()) ws->hsOut += "Sec-WebSocket-Protocol: " + chosen + "\r\n";
    ws->hsOut += "\r\n";
    ws->hsSubprotocol = chosen;
    ws->hsWriting = true;
}

void pump_server_handshake(ENetWebSocket* ws) {
    if (now_ms() >= ws->hsDeadline) { fail(ws, E::ENET_HTTP_ERR_TIMEOUT); return; }

    if (!ws->hsWriting) {
        for (;;) {
            uint8_t tmp[2048];
            int r = stream_read_nb(ws->stream, tmp, sizeof(tmp));
            if (r > 0) {
                ws->hsIn.append((const char*) tmp, (size_t) r);
                size_t end = ws->hsIn.find("\r\n\r\n");
                if (end != std::string::npos && end > 8192) { hs_reject(ws, 431); break; }   // the limit applies to the header block itself, however it arrived
                if (end != std::string::npos) {
                    ws->pre = ws->hsIn.substr(end + 4);                  // pipelined frames, if any
                    hs_process(ws, ws->hsIn.substr(0, end));
                    break;
                }
                if (ws->hsIn.size() > 8192) { hs_reject(ws, 431); break; }
                continue;
            }
            if (r == 0) return;                                          // wait for more
            fail(ws, E::ENET_HTTP_ERR_IO);                               // closed or errored before sending a request
            return;
        }
    }
    if (ws->hsWriting) {
        while (ws->hsOutOff < ws->hsOut.size()) {
            int w = stream_write_nb(ws->stream, ws->hsOut.data() + ws->hsOutOff, ws->hsOut.size() - ws->hsOutOff);
            if (w > 0) { ws->hsOutOff += (size_t) w; continue; }
            if (w == 0) return;                                          // resume next poll
            fail(ws, E::ENET_HTTP_ERR_IO);
            return;
        }
        if (ws->hsReject) { fail(ws, E::ENET_HTTP_ERR_HANDSHAKE_REJECTED, ws->hsStatus); return; }
        if (!start_framing(ws)) { fail(ws, E::ENET_HTTP_ERR_INTERNAL); return; }
        Ev e; e.type = ENetWebSocketEventType::OPEN; e.subprotocol = ws->hsSubprotocol;
        push(ws, std::move(e));
    }
}

void advance(ENetWebSocket* ws) {
    switch (ws->state) {
    case ENetWebSocketState::CLOSED: return;
    case ENetWebSocketState::CONNECTING:
        if (ws->server) { pump_server_handshake(ws); if (ws->state == ENetWebSocketState::OPEN) pump_open(ws); return; }
        {
            std::lock_guard<std::mutex> g(ws->job->m);
            if (!ws->job->done) return;
            if (ws->job->error != E::ENET_HTTP_OK) {
                E err = ws->job->error; int st = ws->job->httpStatus;
                Ev e; e.type = ENetWebSocketEventType::FAILURE; e.error = err; e.status = st;
                push(ws, std::move(e)); ws->state = ENetWebSocketState::CLOSED;
                return;
            }
            ws->stream = ws->job->stream;
            ws->job->stream = Stream();
            ws->pre = ws->job->leftover;
            Ev e; e.type = ENetWebSocketEventType::OPEN; e.subprotocol = ws->job->subprotocol;
            if (!start_framing(ws)) { ws->stream.close(); Ev f; f.type = ENetWebSocketEventType::FAILURE; f.error = E::ENET_HTTP_ERR_INTERNAL; push(ws, std::move(f)); ws->state = ENetWebSocketState::CLOSED; return; }
            push(ws, std::move(e));
        }
        pump_open(ws);
        return;
    case ENetWebSocketState::OPEN:
    case ENetWebSocketState::CLOSING:
        pump_open(ws);
        return;
    }
}

} // namespace

// ------------------------------------------------------------------
// public API
// ------------------------------------------------------------------
ENetWebSocket* enet_websocket_connect(const ENetWebSocketDesc* d) {
    if (!d || !d->url) return nullptr;
    Url u;
    if (!parse_url(d->url, u) || (u.scheme != "ws" && u.scheme != "wss")) return nullptr;
    if (d->header_count > 0 && !d->headers) return nullptr;

    ClientParams p;
    p.url = d->url;
    for (size_t i = 0; i < d->header_count; ++i) {
        const ENetHttpHeader& h = d->headers[i];
        if (!h.name || !h.value || !valid_header_name(h.name) || !valid_header_value(h.value) || forbidden_ws_header(h.name)) return nullptr;
        p.headers.push_back({h.name, h.value});
    }
    if (d->subprotocols && d->subprotocols[0]) {
        std::vector<std::string> toks;
        if (!split_tokens(d->subprotocols, toks)) return nullptr;
        p.subprotocols = d->subprotocols;
    }
    if (d->origin && d->origin[0]) { if (!valid_header_value(d->origin)) return nullptr; p.origin = d->origin; }
    if (d->tls && d->tls->security) {
        const ENetTransportSecurity* s = d->tls->security;
        p.hasTls = true; p.skipVerify = s->skip_verification != 0;
        if (s->cert_file) p.certFile = s->cert_file;
        if (s->key_file) p.keyFile = s->key_file;
        if (s->ca_file) p.caFile = s->ca_file;
    }
    p.timeoutMs = d->connect_timeout_ms ? d->connect_timeout_ms : 10000;
    p.flags = d->flags;

    ENetWebSocket* ws = new ENetWebSocket();
    ws->server = false;
    ws->maxMsg = d->max_message_bytes ? d->max_message_bytes : (1u << 20);
    ws->maxQueued = d->max_queued_bytes ? d->max_queued_bytes : (4u << 20);
    ws->pingMs = d->ping_interval_ms; ws->idleMs = d->idle_timeout_ms;
    ws->job = std::make_shared<ConnectJob>();
    std::shared_ptr<ConnectJob> job = ws->job;
    if (!pool_submit([job, p] { client_connect(job, p); })) { delete ws; return nullptr; }
    return ws;
}

ENetWebSocket* enet_websocket_accept(ENetSocket socket, const ENetWebSocketServerDesc* d) {
    if (socket == ENET_SOCKET_NULL) return nullptr;
    ENetWebSocket* ws = new ENetWebSocket();
    ws->server = true;
    ws->stream.sock = socket;                                       // ownership taken
    enet_socket_set_option(socket, ENetSocketOption::NET_OPT_NONBLOCK, 1);
    enet_socket_set_option(socket, ENetSocketOption::NET_OPT_NODELAY, 1);
    uint32_t hs = 5000;
    if (d) {
        for (size_t i = 0; i < d->allowed_origin_count; ++i) if (d->allowed_origins && d->allowed_origins[i]) ws->allowedOrigins.push_back(d->allowed_origins[i]);
        ws->requireOrigin = d->require_origin != 0;
        if (d->subprotocols) ws->supported = d->subprotocols;
        ws->authorize = d->authorize; ws->authUser = d->user;
        if (d->handshake_timeout_ms) hs = d->handshake_timeout_ms;
        if (d->max_message_bytes) ws->maxMsg = d->max_message_bytes;
        if (d->max_queued_bytes) ws->maxQueued = d->max_queued_bytes;
        ws->pingMs = d->ping_interval_ms; ws->idleMs = d->idle_timeout_ms;
    }
    ws->hsDeadline = now_ms() + hs;
    return ws;
}

int enet_websocket_poll(ENetWebSocket* ws, ENetWebSocketEvent* out) {
    if (!ws || !out) return 0;
    if (ws->events.empty()) advance(ws);
    if (ws->events.empty()) return 0;
    ws->current = std::move(ws->events.front());
    ws->events.pop_front();
    out->type = ws->current.type;
    out->data = ws->current.data.empty() ? nullptr : ws->current.data.data();
    out->length = ws->current.data.size();
    out->close_code = ws->current.code;
    out->error = ws->current.error;
    out->http_status = ws->current.status;
    out->subprotocol = ws->current.subprotocol.c_str();
    return 1;
}

static int queue_message(ENetWebSocket* ws, uint8_t opcode, const void* data, size_t len) {
    if (!ws || ws->state != ENetWebSocketState::OPEN || !ws->ctx) return -1;
    if (len && !data) return -3;
    bool control = opcode == ENET_WS_PING;
    if (control && len > 125) return -3;
    if (!control && enet_ws_event_get_queued_msg_length(ws->ctx) + len > ws->maxQueued) return -2;
    enet_ws_event_msg m{opcode, (const uint8_t*) data, len};
    int rc = enet_ws_event_queue_msg(ws->ctx, &m);
    if (rc == ENET_WS_ERR_NO_MORE_MSG) return -1;
    if (rc == ENET_WS_ERR_NOMEM) return -2;
    if (rc != 0) return -3;
    enet_ws_event_send(ws->ctx);                                    // opportunistic flush; failures surface from poll
    return 0;
}

int enet_websocket_send_text(ENetWebSocket* ws, const char* text, size_t length) { return queue_message(ws, ENET_WS_TEXT_FRAME, text, length); }
int enet_websocket_send_binary(ENetWebSocket* ws, const void* data, size_t length) { return queue_message(ws, ENET_WS_BINARY_FRAME, data, length); }
int enet_websocket_send_ping(ENetWebSocket* ws, const void* data, size_t length) { return queue_message(ws, ENET_WS_PING, data, length); }

int enet_websocket_close(ENetWebSocket* ws, uint16_t code, const char* reason) {
    if (!ws) return -1;
    if (ws->state == ENetWebSocketState::CONNECTING) {
        if (ws->job) ws->job->cancel = true;
        ws->stream.close();
        Ev e; e.type = ENetWebSocketEventType::CLOSE; e.code = 1006;
        push(ws, std::move(e)); ws->state = ENetWebSocketState::CLOSED;
        return 0;
    }
    if (ws->state != ENetWebSocketState::OPEN || !ws->ctx) return -1;
    bool codeOk = code == 0 || (code >= 1000 && code <= 1003) || (code >= 1007 && code <= 1011) || (code >= 3000 && code <= 4999);
    size_t rl = reason ? strlen(reason) : 0;
    if (!codeOk || rl > 123 || (code == 0 && rl > 0)) return -3;
    int rc = enet_ws_event_queue_close(ws->ctx, code, (const uint8_t*) reason, rl);
    if (rc != 0) return -1;
    ws->state = ENetWebSocketState::CLOSING;
    ws->closeDeadline = now_ms() + 5000;
    enet_ws_event_send(ws->ctx);
    return 0;
}

ENetWebSocketState enet_websocket_state(const ENetWebSocket* ws) { return ws ? ws->state : ENetWebSocketState::CLOSED; }

void enet_websocket_destroy(ENetWebSocket* ws) {
    if (!ws) return;
    if (ws->job) {
        ws->job->cancel = true;
        std::lock_guard<std::mutex> g(ws->job->m);                  // a finished-but-unclaimed connection must not leak its socket
        ws->job->stream.close();
    }
    ws->stream.close();
    if (ws->ctx) enet_ws_event_context_free(ws->ctx);
    delete ws;
}
