/**
 @file  http.cpp
 @brief Moss_Network HTTP/1.1 client: strict request/response handling, redirects, retries, async worker pool,
        sessions (bearer auth) and resumable, hash-verified downloads.
*/
#define ENET_BUILDING_LIB 1
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>
#include <Moss/Moss_Network.h>
#include "net_stream.h"

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
#else
  #include <unistd.h>
  #include <sys/stat.h>
#endif

using namespace moss_net;
using E = ENetHttpError;

namespace {

// ------------------------------------------------------------------
// request / response model
// ------------------------------------------------------------------
using HeaderKV = moss_net::Header;

struct Job {
    ENetHttpMethod method = ENetHttpMethod::ENET_HTTP_GET;
    std::string url;
    std::vector<HeaderKV> headers;
    std::vector<uint8_t> body;
    bool hasTls = false, skipVerify = false;
    std::string certFile, keyFile, caFile;
    uint32_t timeoutMs = 15000, flags = 0, maxRedirects = 0, maxRetries = 0, backoffMs = 250;
    uint64_t maxResponse = 8u << 20;
    bool hasSink = false;
    ENetHttpSink sink{};
    std::string userAgent;

    ENetTransportSecurity security() const {
        ENetTransportSecurity s;
        s.cert_file = certFile.empty() ? nullptr : certFile.c_str();
        s.key_file = keyFile.empty() ? nullptr : keyFile.c_str();
        s.ca_file = caFile.empty() ? nullptr : caFile.c_str();
        s.skip_verification = skipVerify ? 1 : 0;
        return s;
    }
};

struct Result {
    E error = E::ENET_HTTP_OK;
    int status = 0;
    std::vector<HeaderKV> headers;
    std::vector<uint8_t> body;
    uint64_t delivered = 0;
    std::string finalUrl;
    uint32_t attempts = 0;
    uint32_t retryAfterMs = 0;
};

struct Head { int status = 0; std::vector<HeaderKV> headers; };

const char* find_header(const std::vector<HeaderKV>& h, const char* name) {
    for (const HeaderKV& kv : h) if (ci_equals(kv.name, name)) return kv.value.c_str();
    return nullptr;
}

const char* method_name(ENetHttpMethod m) {
    switch (m) {
        case ENetHttpMethod::ENET_HTTP_GET: return "GET";
        case ENetHttpMethod::ENET_HTTP_POST: return "POST";
        case ENetHttpMethod::ENET_HTTP_PUT: return "PUT";
        case ENetHttpMethod::ENET_HTTP_DELETE: return "DELETE";
        case ENetHttpMethod::ENET_HTTP_PATCH: return "PATCH";
        case ENetHttpMethod::ENET_HTTP_HEAD: return "HEAD";
    }
    return "GET";
}

// Headers the library owns. Letting callers set them invites request smuggling / framing confusion.
bool forbidden_user_header(const std::string& n) {
    static const char* bad[] = {"host", "content-length", "transfer-encoding", "connection", "expect", "te", "upgrade",
                                "proxy-connection", "keep-alive", nullptr};
    for (int i = 0; bad[i]; ++i) if (ci_equals(n, bad[i])) return true;
    return false;
}
bool sensitive_header(const std::string& n) {
    return ci_equals(n, "authorization") || ci_equals(n, "cookie") || ci_equals(n, "proxy-authorization");
}

E map_io(int r) {
    switch (r) {
        case IO_TIMEOUT: return E::ENET_HTTP_ERR_TIMEOUT;
        case IO_CANCELED: return E::ENET_HTTP_ERR_CANCELED;
        default: return E::ENET_HTTP_ERR_IO;
    }
}

// ------------------------------------------------------------------
// buffered reader on top of a stream
// ------------------------------------------------------------------
struct Reader {
    Stream* s = nullptr;
    std::string buf;
    size_t off = 0;
    uint64_t deadline = 0;
    uint32_t idleMs = 0;                       // >0: the deadline slides forward after every successful read
    const std::atomic<bool>* cancel = nullptr;
    E err = E::ENET_HTTP_OK;
    bool eof = false;

    size_t buffered() const { return buf.size() - off; }

    bool fill() {                              // false on EOF (eof = true) or error (err set)
        uint8_t tmp[16384];
        int r = stream_read(*s, tmp, sizeof(tmp), deadline, cancel);
        if (r > 0) {
            if (off > 0 && off == buf.size()) { buf.clear(); off = 0; }
            buf.append((const char*) tmp, (size_t) r);
            if (idleMs) deadline = now_ms() + idleMs;
            return true;
        }
        if (r == IO_EOF) { eof = true; return false; }
        err = map_io(r);
        return false;
    }

    // returns bytes (>0), 0 on clean EOF, -1 on error
    int read(uint8_t* out, size_t n) {
        if (buffered() == 0) {
            if (n >= 4096) {                   // large read: go straight into the caller's buffer
                int r = stream_read(*s, out, n, deadline, cancel);
                if (r > 0) { if (idleMs) deadline = now_ms() + idleMs; return r; }
                if (r == IO_EOF) { eof = true; return 0; }
                err = map_io(r); return -1;
            }
            if (!fill()) return eof ? 0 : -1;
        }
        size_t t = std::min(n, buffered());
        memcpy(out, buf.data() + off, t);
        off += t;
        return (int) t;
    }

    // CRLF-terminated line (without the CRLF); false on error / EOF / overlong
    bool read_line(std::string& line, size_t maxLen) {
        for (;;) {
            size_t pos = buf.find("\r\n", off);
            if (pos != std::string::npos) { line.assign(buf, off, pos - off); off = pos + 2; return true; }
            if (buffered() > maxLen) { err = E::ENET_HTTP_ERR_INVALID_RESPONSE; return false; }
            if (!fill()) { if (eof) err = E::ENET_HTTP_ERR_INVALID_RESPONSE; return false; }
        }
    }
};

E read_head(Reader& rd, Head& head) {
    for (int interim = 0; interim < 8; ++interim) {
        size_t pos;
        for (;;) {
            pos = rd.buf.find("\r\n\r\n", rd.off);
            if (pos != std::string::npos) break;
            if (rd.buffered() > 32768) return E::ENET_HTTP_ERR_INVALID_RESPONSE;       // header block too large
            if (!rd.fill()) return rd.eof ? E::ENET_HTTP_ERR_INVALID_RESPONSE : rd.err;
        }
        std::string block = rd.buf.substr(rd.off, pos - rd.off);
        rd.off = pos + 4;

        size_t eol = block.find("\r\n");
        if (!parse_status_line(block.substr(0, eol), head.status)) return E::ENET_HTTP_ERR_INVALID_RESPONSE;
        if (!parse_header_lines(block, eol == std::string::npos ? block.size() : eol + 2, head.headers)) return E::ENET_HTTP_ERR_INVALID_RESPONSE;
        if (head.status >= 100 && head.status < 200 && head.status != 101) continue;   // interim response (e.g. 100 Continue): skip
        return E::ENET_HTTP_OK;
    }
    return E::ENET_HTTP_ERR_INVALID_RESPONSE;
}

// ------------------------------------------------------------------
// body framing
// ------------------------------------------------------------------
struct Framing { enum Mode { NONE, LENGTH, CHUNKED, UNTIL_EOF } mode = NONE; uint64_t length = 0; };

bool parse_u64(const std::string& s, uint64_t& out, int base) {
    if (s.empty() || s.size() > 20) return false;
    uint64_t v = 0;
    for (char c : s) {
        int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (base == 16 && c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (base == 16 && c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else return false;
        if (v > (UINT64_MAX - (uint64_t) d) / (uint64_t) base) return false;
        v = v * (uint64_t) base + (uint64_t) d;
    }
    out = v;
    return true;
}

E determine_framing(const Head& h, ENetHttpMethod method, Framing& f) {
    f = Framing();
    if (method == ENetHttpMethod::ENET_HTTP_HEAD || h.status == 204 || h.status == 304 || (h.status >= 100 && h.status < 200)) return E::ENET_HTTP_OK;

    const char* te = find_header(h.headers, "transfer-encoding");
    bool haveCl = false;
    uint64_t cl = 0;
    for (const HeaderKV& kv : h.headers) {
        if (!ci_equals(kv.name, "content-length")) continue;
        uint64_t v;
        if (!parse_u64(kv.value, v, 10)) return E::ENET_HTTP_ERR_INVALID_RESPONSE;
        if (haveCl && v != cl) return E::ENET_HTTP_ERR_INVALID_RESPONSE;          // conflicting lengths
        haveCl = true; cl = v;
    }
    if (te) {
        if (haveCl) return E::ENET_HTTP_ERR_INVALID_RESPONSE;                      // both framings: classic smuggling shape
        if (!ci_equals(trim(te), "chunked")) return E::ENET_HTTP_ERR_INVALID_RESPONSE;
        f.mode = Framing::CHUNKED;
    } else if (haveCl) { f.mode = Framing::LENGTH; f.length = cl; }
    else f.mode = Framing::UNTIL_EOF;
    return E::ENET_HTTP_OK;
}

struct BodyOut {
    std::vector<uint8_t>* buf = nullptr;
    const ENetHttpSink* sink = nullptr;
    uint64_t limit = 0, delivered = 0;
    E err = E::ENET_HTTP_OK;
    bool put(const uint8_t* d, size_t n) {
        if (sink) {
            if (sink->on_data && sink->on_data(sink->user, d, n) != 0) { err = E::ENET_HTTP_ERR_CANCELED; return false; }
        } else {
            if (delivered + n > limit) { err = E::ENET_HTTP_ERR_TOO_LARGE; return false; }
            buf->insert(buf->end(), d, d + n);
        }
        delivered += n;
        return true;
    }
};

E read_body(Reader& rd, const Framing& f, BodyOut& out) {
    uint8_t tmp[16384];
    switch (f.mode) {
    case Framing::NONE: return E::ENET_HTTP_OK;

    case Framing::LENGTH: {
        uint64_t left = f.length;
        while (left > 0) {
            int r = rd.read(tmp, (size_t) std::min<uint64_t>(left, sizeof(tmp)));
            if (r == 0) return E::ENET_HTTP_ERR_INVALID_RESPONSE;                  // truncated body
            if (r < 0) return rd.err;
            if (!out.put(tmp, (size_t) r)) return out.err;
            left -= (uint64_t) r;
        }
        return E::ENET_HTTP_OK;
    }

    case Framing::CHUNKED: {
        for (;;) {
            std::string line;
            if (!rd.read_line(line, 1024)) return rd.err == E::ENET_HTTP_OK ? E::ENET_HTTP_ERR_INVALID_RESPONSE : rd.err;
            size_t semi = line.find(';');                                          // chunk extensions are ignored
            uint64_t size;
            if (!parse_u64(trim(line.substr(0, semi)), size, 16)) return E::ENET_HTTP_ERR_INVALID_RESPONSE;
            if (size == 0) break;
            while (size > 0) {
                int r = rd.read(tmp, (size_t) std::min<uint64_t>(size, sizeof(tmp)));
                if (r == 0) return E::ENET_HTTP_ERR_INVALID_RESPONSE;
                if (r < 0) return rd.err;
                if (!out.put(tmp, (size_t) r)) return out.err;
                size -= (uint64_t) r;
            }
            std::string crlf;
            if (!rd.read_line(crlf, 8) || !crlf.empty()) return rd.err == E::ENET_HTTP_OK ? E::ENET_HTTP_ERR_INVALID_RESPONSE : rd.err;
        }
        for (int i = 0; i < 64; ++i) {                                              // trailers: consumed, ignored
            std::string t;
            if (!rd.read_line(t, 8192)) return rd.err == E::ENET_HTTP_OK ? E::ENET_HTTP_ERR_INVALID_RESPONSE : rd.err;
            if (t.empty()) return E::ENET_HTTP_OK;
        }
        return E::ENET_HTTP_ERR_INVALID_RESPONSE;
    }

    case Framing::UNTIL_EOF:
        for (;;) {
            int r = rd.read(tmp, sizeof(tmp));
            if (r == 0) return E::ENET_HTTP_OK;                                     // only a CLEAN end of stream completes the body
            if (r < 0) return rd.err;
            if (!out.put(tmp, (size_t) r)) return out.err;
        }
    }
    return E::ENET_HTTP_ERR_INTERNAL;
}

// ------------------------------------------------------------------
// request execution
// ------------------------------------------------------------------
std::string build_request(const Job& job, const Url& url, ENetHttpMethod method, const std::vector<HeaderKV>& headers, const std::vector<uint8_t>& body) {
    std::string r;
    r.reserve(512 + body.size());
    r += method_name(method); r += ' '; r += url.path; r += " HTTP/1.1\r\n";
    r += "Host: " + host_header_of(url) + "\r\n";
    bool hasUa = false;
    for (const HeaderKV& h : headers) if (ci_equals(h.name, "user-agent")) hasUa = true;
    if (!hasUa) r += "User-Agent: " + job.userAgent + "\r\n";
    bool hasAe = false;
    for (const HeaderKV& h : headers) if (ci_equals(h.name, "accept-encoding")) hasAe = true;
    if (!hasAe) r += "Accept-Encoding: identity\r\n";             // we do not decompress: ask for plain bodies
    r += "Connection: close\r\n";
    for (const HeaderKV& h : headers) r += h.name + ": " + h.value + "\r\n";
    bool hasBody = !body.empty() || method == ENetHttpMethod::ENET_HTTP_POST || method == ENetHttpMethod::ENET_HTTP_PUT || method == ENetHttpMethod::ENET_HTTP_PATCH;
    if (hasBody) r += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    r += "\r\n";
    r.append((const char*) body.data(), body.size());
    return r;
}

bool resolve_location(const Url& base, const std::string& loc, std::string& out) {
    if (loc.empty()) return false;
    for (unsigned char c : loc) if (c <= 0x20 || c >= 0x7f) return false;
    if (loc.find("://") != std::string::npos && loc.find("://") < 8) { out = loc; return true; }
    if (loc.compare(0, 2, "//") == 0) { out = base.scheme + ":" + loc; return true; }
    if (loc[0] == '/') { out = origin_of(base) + loc; return true; }
    std::string path = base.path;
    size_t q = path.find('?');
    std::string pathOnly = (q == std::string::npos) ? path : path.substr(0, q);
    if (loc[0] == '?') { out = origin_of(base) + pathOnly + loc; return true; }
    size_t slash = pathOnly.rfind('/');
    out = origin_of(base) + pathOnly.substr(0, slash + 1) + loc;
    return true;
}

bool is_redirect_status(int s) { return s == 301 || s == 302 || s == 303 || s == 307 || s == 308; }

void run_once(const Job& job, const std::atomic<bool>* cancel, Result& res) {
    uint64_t deadline = now_ms() + job.timeoutMs;
    std::string curUrl = job.url;
    ENetHttpMethod method = job.method;
    std::vector<HeaderKV> headers = job.headers;
    std::vector<uint8_t> body = job.body;
    uint32_t redirects = 0;

    for (;;) {
        Url url;
        if (!parse_url(curUrl, url)) { res.error = E::ENET_HTTP_ERR_INVALID_ARGUMENT; return; }
        if (!url.tls && !is_loopback_host(url.host) && !(job.flags & ENetHttpFlag::ALLOW_INSECURE)) { res.error = E::ENET_HTTP_ERR_INSECURE; return; }

        ENetTransportSecurity sec = job.security();
        Stream st;
        E e = stream_connect(st, url, job.hasTls ? &sec : nullptr, deadline, cancel);
        if (e != E::ENET_HTTP_OK) { res.error = e; return; }
        struct Guard { Stream& s; ~Guard() { s.close(); } } guard{st};

        std::string req = build_request(job, url, method, headers, body);
        int w = stream_write_all(st, req.data(), req.size(), deadline, cancel);
        if (w != 0) { res.error = map_io(w); return; }

        Reader rd;
        rd.s = &st; rd.deadline = deadline; rd.cancel = cancel;
        Head head;
        e = read_head(rd, head);
        if (e != E::ENET_HTTP_OK) { res.error = e; return; }
        res.status = head.status;

        if (is_redirect_status(head.status) && job.maxRedirects > 0) {
            const char* loc = find_header(head.headers, "location");
            if (loc) {
                std::string next;
                Url nu;
                if (!resolve_location(url, loc, next) || !parse_url(next, nu)) { res.error = E::ENET_HTTP_ERR_INVALID_RESPONSE; return; }
                if (url.tls && !nu.tls) { res.error = E::ENET_HTTP_ERR_INSECURE; return; }          // never downgrade https
                if (++redirects > job.maxRedirects) { res.error = E::ENET_HTTP_ERR_TOO_MANY_REDIRECTS; return; }
                if (origin_of(nu) != origin_of(url)) {                                              // leaving the origin: drop credentials
                    headers.erase(std::remove_if(headers.begin(), headers.end(), [](const HeaderKV& h) { return sensitive_header(h.name); }), headers.end());
                }
                if (head.status == 301 || head.status == 302 || head.status == 303) {
                    if (method != ENetHttpMethod::ENET_HTTP_GET && method != ENetHttpMethod::ENET_HTTP_HEAD) {
                        method = ENetHttpMethod::ENET_HTTP_GET;
                        body.clear();
                        headers.erase(std::remove_if(headers.begin(), headers.end(), [](const HeaderKV& h) { return ci_equals(h.name, "content-type"); }), headers.end());
                    }
                }
                curUrl = next;
                continue;
            }
        }

        res.finalUrl = curUrl;
        res.headers = head.headers;
        if (const char* ra = find_header(head.headers, "retry-after")) {
            uint64_t secs;
            if (parse_u64(ra, secs, 10)) res.retryAfterMs = (uint32_t) std::min<uint64_t>(secs, 30) * 1000u;
        }

        Framing f;
        e = determine_framing(head, method, f);
        if (e != E::ENET_HTTP_OK) { res.error = e; return; }

        BodyOut out;
        out.buf = &res.body;
        out.limit = job.maxResponse;
        if (job.hasSink) {
            out.sink = &job.sink;
            std::vector<ENetHttpHeader> hv;
            for (const HeaderKV& kv : head.headers) hv.push_back({kv.name.c_str(), kv.value.c_str()});
            if (job.sink.on_headers && job.sink.on_headers(job.sink.user, head.status, hv.data(), hv.size(), f.mode == Framing::LENGTH ? f.length : UINT64_MAX) != 0) {
                res.error = E::ENET_HTTP_ERR_CANCELED;
                return;
            }
            rd.idleMs = job.timeoutMs;                    // streamed bodies: the timeout is max silence, not total time
        } else if (f.mode == Framing::LENGTH && f.length > job.maxResponse) {
            res.error = E::ENET_HTTP_ERR_TOO_LARGE;
            return;
        }
        e = read_body(rd, f, out);
        res.delivered = out.delivered;
        res.error = e;
        return;
    }
}

bool method_idempotent(ENetHttpMethod m) {
    return m == ENetHttpMethod::ENET_HTTP_GET || m == ENetHttpMethod::ENET_HTTP_HEAD || m == ENetHttpMethod::ENET_HTTP_PUT || m == ENetHttpMethod::ENET_HTTP_DELETE;
}

bool should_retry(const Job& job, const Result& res, uint32_t attempt) {
    if (job.hasSink || attempt > job.maxRetries) return false;
    bool safe = method_idempotent(job.method);
    if (!safe) for (const HeaderKV& h : job.headers) if (ci_equals(h.name, "idempotency-key")) safe = true;   // server de-duplicates it
    if (!safe) return false;
    switch (res.error) {
        case E::ENET_HTTP_ERR_CONNECT_FAILED: case E::ENET_HTTP_ERR_TIMEOUT: case E::ENET_HTTP_ERR_IO: case E::ENET_HTTP_ERR_DNS_FAILURE: return true;
        case E::ENET_HTTP_OK: return res.status == 408 || res.status == 429 || res.status == 502 || res.status == 503 || res.status == 504;
        default: return false;     // TLS verification failures, bad arguments, cancellation, oversize...: retrying cannot help
    }
}

void perform(const Job& job, const std::atomic<bool>* cancel, Result& res) {
    for (uint32_t attempt = 1;; ++attempt) {
        res = Result();
        res.attempts = attempt;
        if (cancel && cancel->load()) { res.error = E::ENET_HTTP_ERR_CANCELED; return; }
        run_once(job, cancel, res);
        if (!should_retry(job, res, attempt)) return;
        uint64_t delay = (uint64_t) job.backoffMs << std::min<uint32_t>(attempt - 1, 7);
        uint8_t jr = 0;
        if (enet_random_bytes(&jr, 1) == 0) delay += delay * jr / 1024;              // up to +25% jitter
        delay = std::max<uint64_t>(delay, res.retryAfterMs);
        delay = std::min<uint64_t>(delay, 30000);
        uint32_t attempts = res.attempts;
        if (!sleep_cancelable((uint32_t) delay, cancel)) { res = Result(); res.attempts = attempts; res.error = E::ENET_HTTP_ERR_CANCELED; return; }
    }
}

// ------------------------------------------------------------------
// descriptor validation / copying
// ------------------------------------------------------------------
E make_job(const ENetHttpRequestDesc* d, Job& job) {
    if (!d || !d->url) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    Url u;
    if (!parse_url(d->url, u)) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    if (u.scheme == "ws" || u.scheme == "wss") return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    if (d->body_size > 0 && !d->body) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    if (d->body_size > 0 && (d->method == ENetHttpMethod::ENET_HTTP_GET || d->method == ENetHttpMethod::ENET_HTTP_HEAD)) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    if (d->header_count > 0 && !d->headers) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    job.method = d->method;
    job.url = d->url;
    for (size_t i = 0; i < d->header_count; ++i) {
        const ENetHttpHeader& h = d->headers[i];
        if (!h.name || !h.value || !valid_header_name(h.name) || !valid_header_value(h.value) || forbidden_user_header(h.name)) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
        job.headers.push_back({h.name, h.value});
    }
    if (d->body_size) job.body.assign((const uint8_t*) d->body, (const uint8_t*) d->body + d->body_size);
    if (d->tls && d->tls->security) {
        const ENetTransportSecurity* s = d->tls->security;
        job.hasTls = true;
        job.skipVerify = s->skip_verification != 0;
        if (s->cert_file) job.certFile = s->cert_file;
        if (s->key_file) job.keyFile = s->key_file;
        if (s->ca_file) job.caFile = s->ca_file;
    }
    job.timeoutMs = d->timeout_ms ? d->timeout_ms : 15000;
    job.flags = d->flags;
    job.maxRedirects = std::min<uint32_t>(d->max_redirects, 16);
    job.maxResponse = d->max_response_bytes ? d->max_response_bytes : (8u << 20);
    job.maxRetries = std::min<uint32_t>(d->max_retries, 10);
    job.backoffMs = d->retry_backoff_ms ? d->retry_backoff_ms : 250;
    if (d->sink) { job.hasSink = true; job.sink = *d->sink; }
    job.userAgent = default_user_agent();
    return E::ENET_HTTP_OK;
}

} // namespace

// ------------------------------------------------------------------
// request object, worker pool, main-thread delivery
// ------------------------------------------------------------------
struct ENetHttpRequest {
    std::atomic<int> refs{2};                       // one for the caller's handle, one for the pipeline
    std::atomic<bool> cancel{false}, released{false};
    Job job;
    Result res;
    ENetHttpResponseCallback cb = nullptr;
    void* user = nullptr;

    // downloads
    bool isDownload = false;
    std::string dlUrl, dlDest, dlSha;
    uint64_t dlExpectedSize = 0;
    ENetHttpDownloadResult dlRes{};
    ENetHttpProgressCallback progress = nullptr;
    ENetHttpDownloadCallback doneCb = nullptr;
    std::atomic<uint64_t> bytesDone{0}, bytesTotal{0};
    std::atomic<bool> progressPending{false};
    uint64_t lastProgressPost = 0;
};

namespace {

struct Pool {
    std::mutex jm;
    std::condition_variable jcv;
    std::deque<std::function<void()>> jobs;
    bool running = true;
    std::vector<std::thread> threads;
    std::string userAgent = "Moss/1.0";

    std::mutex em;
    std::deque<std::function<void(bool)>> events;  // run by enet_http_poll() with true; with false at shutdown (release only)

    std::mutex lm;
    std::set<ENetHttpRequest*> live;               // for shutdown-time cancellation
};

Pool* g_pool = nullptr;
std::mutex g_poolMutex;

void unref(ENetHttpRequest* r) { if (r->refs.fetch_sub(1) == 1) delete r; }

void worker_main(Pool* p) {
    for (;;) {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lk(p->jm);
            p->jcv.wait(lk, [&] { return !p->running || !p->jobs.empty(); });
            if (p->jobs.empty()) return;            // only reached when !running
            job = std::move(p->jobs.front());
            p->jobs.pop_front();
        }
        job();
    }
}

Pool* start_pool(const ENetHttpConfig* cfg) {            // g_poolMutex held
    Pool* p = new Pool();
    if (cfg && cfg->user_agent && valid_header_value(cfg->user_agent)) p->userAgent = cfg->user_agent;
    uint32_t n = (cfg && cfg->worker_threads) ? std::min<uint32_t>(cfg->worker_threads, 16) : 2;
    for (uint32_t i = 0; i < n; ++i) p->threads.emplace_back(worker_main, p);
    return p;
}

void post_main(std::function<void(bool)> fn) {
    std::lock_guard<std::mutex> g(g_poolMutex);
    if (!g_pool) return;
    std::lock_guard<std::mutex> e(g_pool->em);
    g_pool->events.push_back(std::move(fn));
}

void live_add(ENetHttpRequest* r) { std::lock_guard<std::mutex> g(g_pool->lm); g_pool->live.insert(r); }
void live_remove(ENetHttpRequest* r) {
    std::lock_guard<std::mutex> g(g_poolMutex);
    if (!g_pool) return;
    std::lock_guard<std::mutex> l(g_pool->lm);
    g_pool->live.erase(r);
}

void fill_response(ENetHttpResponse* out, const Result& res, std::vector<ENetHttpHeader>& hv, bool sinkUsed) {
    hv.clear();
    for (const HeaderKV& kv : res.headers) hv.push_back({kv.name.c_str(), kv.value.c_str()});
    out->http_status = res.status;
    out->error = res.error;
    out->headers = hv.data();
    out->header_count = hv.size();
    out->body = sinkUsed ? nullptr : (res.body.empty() ? nullptr : res.body.data());
    out->body_size = sinkUsed ? (size_t) res.delivered : res.body.size();
    out->final_url = res.finalUrl.c_str();
    out->attempts = res.attempts;
    out->internal_ = nullptr;
}

} // namespace

namespace moss_net {

bool pool_submit(std::function<void()> job) {
    std::lock_guard<std::mutex> g(g_poolMutex);
    if (!g_pool) g_pool = start_pool(nullptr);
    {
        std::lock_guard<std::mutex> l(g_pool->jm);
        if (!g_pool->running) return false;
        g_pool->jobs.push_back(std::move(job));
    }
    g_pool->jcv.notify_one();
    return true;
}

std::string default_user_agent() {
    std::lock_guard<std::mutex> g(g_poolMutex);
    return g_pool ? g_pool->userAgent : std::string("Moss/1.0");
}

} // namespace moss_net

// ------------------------------------------------------------------
// public API: lifecycle
// ------------------------------------------------------------------
int enet_http_init(const ENetHttpConfig* config) {
    std::lock_guard<std::mutex> g(g_poolMutex);
    if (g_pool) return -1;
    g_pool = start_pool(config);
    return 0;
}

void enet_http_shutdown(void) {
    Pool* p;
    {
        std::lock_guard<std::mutex> g(g_poolMutex);
        p = g_pool;
        if (!p) return;
        {
            std::lock_guard<std::mutex> l(p->jm);
            p->running = false;
        }
        {
            std::lock_guard<std::mutex> l(p->lm);
            for (ENetHttpRequest* r : p->live) r->cancel = true;
        }
    }
    p->jcv.notify_all();
    for (std::thread& t : p->threads) t.join();
    {
        std::lock_guard<std::mutex> g(g_poolMutex);
        g_pool = nullptr;
    }
    // undelivered completions are dropped (callbacks do not run) but their pipeline references are released
    for (auto& fn : p->events) fn(false);
    p->events.clear();
    delete p;
}

int enet_http_poll(void) {
    std::deque<std::function<void(bool)>> batch;
    {
        std::lock_guard<std::mutex> g(g_poolMutex);
        if (!g_pool) return 0;
        std::lock_guard<std::mutex> e(g_pool->em);
        batch.swap(g_pool->events);
    }
    int n = 0;
    for (auto& fn : batch) { fn(true); ++n; }
    return n;
}

void enet_http_request_desc_init(ENetHttpRequestDesc* d, ENetHttpMethod method, const char* url) {
    memset(d, 0, sizeof(*d));
    d->method = method;
    d->url = url;
    d->timeout_ms = 15000;
    d->max_redirects = 3;
    d->max_response_bytes = 8u << 20;
}

// ------------------------------------------------------------------
// public API: requests
// ------------------------------------------------------------------
static void run_request(ENetHttpRequest* req) {
    if (req->cancel.load()) { req->res = Result(); req->res.error = E::ENET_HTTP_ERR_CANCELED; req->res.attempts = 0; }
    else perform(req->job, &req->cancel, req->res);
    live_remove(req);
    post_main([req](bool deliver) {
        if (deliver && !req->released.load() && req->cb) {
            ENetHttpResponse resp;
            std::vector<ENetHttpHeader> hv;
            fill_response(&resp, req->res, hv, req->job.hasSink);
            req->cb(&resp, req->user);
        }
        unref(req);
    });
}

ENetHttpRequest* enet_http_request_async(const ENetHttpRequestDesc* desc, ENetHttpResponseCallback cb, void* userData) {
    Job job;
    if (make_job(desc, job) != E::ENET_HTTP_OK) return nullptr;
    ENetHttpRequest* req = new ENetHttpRequest();
    req->job = std::move(job);
    req->cb = cb;
    req->user = userData;
    {
        std::lock_guard<std::mutex> g(g_poolMutex);
        if (!g_pool) g_pool = start_pool(nullptr);
        req->job.userAgent = g_pool->userAgent;
        live_add(req);
    }
    if (!pool_submit([req] { run_request(req); })) { live_remove(req); delete req; return nullptr; }
    return req;
}

void enet_http_request_cancel(ENetHttpRequest* r) { if (r) r->cancel = true; }

void enet_http_request_release(ENetHttpRequest* r) {
    if (!r) return;
    r->released = true;
    r->cancel = true;
    unref(r);
}

ENetHttpError enet_http_perform(const ENetHttpRequestDesc* desc, ENetHttpResponse* out) {
    if (!out) return E::ENET_HTTP_ERR_INVALID_ARGUMENT;
    memset(out, 0, sizeof(*out));
    Job job;
    E e = make_job(desc, job);
    if (e != E::ENET_HTTP_OK) { out->error = e; return e; }
    struct Owner { Result res; std::vector<ENetHttpHeader> hv; };
    Owner* o = new Owner();
    std::atomic<bool> never{false};
    perform(job, &never, o->res);
    fill_response(out, o->res, o->hv, job.hasSink);
    out->internal_ = o;
    return o->res.error;
}

void enet_http_response_free(ENetHttpResponse* r) {
    if (!r) return;
    struct Owner { Result res; std::vector<ENetHttpHeader> hv; };
    delete (Owner*) r->internal_;
    memset(r, 0, sizeof(*r));
}

const char* enet_http_response_header(const ENetHttpResponse* r, const char* name) {
    if (!r || !name) return nullptr;
    for (size_t i = 0; i < r->header_count; ++i) if (ci_equals(r->headers[i].name, name)) return r->headers[i].value;
    return nullptr;
}

// ------------------------------------------------------------------
// sessions
// ------------------------------------------------------------------
struct ENetHttpSession {
    Url base;
    std::string origin, prefix;
    std::vector<HeaderKV> headers;
    std::string bearer;
    bool hasTls = false, skipVerify = false;
    std::string certFile, keyFile, caFile;
    uint32_t flags = 0;
};

ENetHttpSession* enet_http_session_create(const char* baseUrl) {
    if (!baseUrl) return nullptr;
    Url u;
    if (!parse_url(baseUrl, u) || u.scheme == "ws" || u.scheme == "wss") return nullptr;
    if (u.path.find('?') != std::string::npos) return nullptr;
    ENetHttpSession* s = new ENetHttpSession();
    s->base = u;
    s->origin = origin_of(u);
    s->prefix = (u.path == "/") ? "" : u.path;
    while (!s->prefix.empty() && s->prefix.back() == '/') s->prefix.pop_back();
    return s;
}

void enet_http_session_destroy(ENetHttpSession* s) { delete s; }

int enet_http_session_set_header(ENetHttpSession* s, const char* name, const char* value) {
    if (!s || !name || !value || !valid_header_name(name) || !valid_header_value(value) || forbidden_user_header(name)) return -1;
    for (HeaderKV& h : s->headers) if (ci_equals(h.name, name)) { h.value = value; return 0; }
    s->headers.push_back({name, value});
    return 0;
}

void enet_http_session_set_bearer_token(ENetHttpSession* s, const char* token) {
    if (!s) return;
    s->bearer = (token && valid_header_value(token)) ? token : "";
}

void enet_http_session_set_tls(ENetHttpSession* s, const ENetTransportSecurity* sec) {
    if (!s) return;
    s->hasTls = sec != nullptr;
    s->skipVerify = sec && sec->skip_verification;
    s->certFile = (sec && sec->cert_file) ? sec->cert_file : "";
    s->keyFile = (sec && sec->key_file) ? sec->key_file : "";
    s->caFile = (sec && sec->ca_file) ? sec->ca_file : "";
}

void enet_http_session_set_flags(ENetHttpSession* s, uint32_t flags) { if (s) s->flags = flags; }

ENetHttpRequest* enet_http_session_request_async(ENetHttpSession* s, const ENetHttpRequestDesc* desc, ENetHttpResponseCallback cb, void* userData) {
    if (!s || !desc || !desc->url || desc->url[0] != '/' || desc->url[1] == '/') return nullptr;   // a path on OUR origin only
    std::string url = s->origin + s->prefix + desc->url;

    std::vector<ENetHttpHeader> hv;
    std::string auth = s->bearer.empty() ? std::string() : "Bearer " + s->bearer;
    auto callerHas = [&](const char* name) {
        for (size_t i = 0; i < desc->header_count; ++i) if (desc->headers[i].name && ci_equals(desc->headers[i].name, name)) return true;
        return false;
    };
    for (const HeaderKV& h : s->headers) if (!callerHas(h.name.c_str())) hv.push_back({h.name.c_str(), h.value.c_str()});
    if (!auth.empty() && !callerHas("authorization")) hv.push_back({"Authorization", auth.c_str()});
    for (size_t i = 0; i < desc->header_count; ++i) hv.push_back(desc->headers[i]);

    ENetTransportSecurity sec;
    sec.cert_file = s->certFile.empty() ? nullptr : s->certFile.c_str();
    sec.key_file = s->keyFile.empty() ? nullptr : s->keyFile.c_str();
    sec.ca_file = s->caFile.empty() ? nullptr : s->caFile.c_str();
    sec.skip_verification = s->skipVerify ? 1 : 0;
    ENetHttpTLSConfig tlsCfg{&sec};

    ENetHttpRequestDesc d = *desc;
    d.url = url.c_str();
    d.headers = hv.data();
    d.header_count = hv.size();
    d.flags |= s->flags;
    if (!d.tls && s->hasTls) d.tls = &tlsCfg;
    return enet_http_request_async(&d, cb, userData);
}

// ------------------------------------------------------------------
// downloads (patching / updates)
// ------------------------------------------------------------------
namespace {

#ifdef _WIN32
std::wstring widen(const std::string& s) {
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w((size_t) (n > 0 ? n : 1), L'\0');
    if (n > 0) MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &w[0], n);
    w.resize(n > 0 ? (size_t) n - 1 : 0);
    return w;
}
FILE* open_file(const std::string& p, const char* mode) { return _wfopen(widen(p).c_str(), widen(mode).c_str()); }
void remove_file(const std::string& p) { _wremove(widen(p).c_str()); }
bool replace_file(const std::string& from, const std::string& to) { return MoveFileExW(widen(from).c_str(), widen(to).c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0; }
#else
FILE* open_file(const std::string& p, const char* mode) { return fopen(p.c_str(), mode); }
void remove_file(const std::string& p) { remove(p.c_str()); }
bool replace_file(const std::string& from, const std::string& to) { return rename(from.c_str(), to.c_str()) == 0; }   // atomic on POSIX
#endif

bool parse_hex_sha256(const std::string& hex, uint8_t out[32]) {
    if (hex.size() != 64) return false;
    for (size_t i = 0; i < 32; ++i) {
        int v = 0;
        for (int k = 0; k < 2; ++k) {
            char c = hex[i * 2 + (size_t) k];
            int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
            if (d < 0) return false;
            v = v * 16 + d;
        }
        out[i] = (uint8_t) v;
    }
    return true;
}

// hashes an existing partial file; returns its size, or 0 (with a fresh hash state) when it cannot be read
uint64_t hash_existing(const std::string& path, ENetSha256* h) {
    enet_sha256_init(h);
    FILE* f = open_file(path, "rb");
    if (!f) return 0;
    uint8_t buf[65536];
    uint64_t total = 0;
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) { enet_sha256_update(h, buf, n); total += n; }
    bool bad = ferror(f) != 0;
    fclose(f);
    if (bad) { enet_sha256_init(h); return 0; }
    return total;
}

struct DlCtx {
    ENetHttpRequest* req = nullptr;
    std::string part;
    FILE* file = nullptr;
    ENetSha256 hash;
    uint64_t offset = 0, total = 0, expectedSize = 0;
    bool resumed = false, rangeBad = false, ioFail = false, tooLarge = false;
    int badStatus = 0;
};

void post_progress(DlCtx* c, bool force) {
    ENetHttpRequest* r = c->req;
    r->bytesDone = c->total;
    uint64_t now = now_ms();
    if (!r->progress) return;
    if (!force && now - r->lastProgressPost < 100) return;
    r->lastProgressPost = now;
    if (r->progressPending.exchange(true)) return;
    r->refs.fetch_add(1);
    post_main([r](bool deliver) {
        r->progressPending = false;
        if (deliver && !r->released.load() && r->progress) r->progress(r->bytesDone.load(), r->bytesTotal.load(), r->user);
        unref(r);
    });
}

int dl_on_headers(void* user, int status, const ENetHttpHeader* headers, size_t count, uint64_t contentLength) {
    DlCtx* c = (DlCtx*) user;
    if (status == 416) { c->rangeBad = true; return 1; }
    if (status != 200 && status != 206) { c->badStatus = status; return 1; }
    if (status == 206) {
        if (c->offset == 0) { c->badStatus = status; return 1; }                           // we did not ask for a range
        uint64_t start = UINT64_MAX;
        for (size_t i = 0; i < count; ++i) {
            if (!ci_equals(headers[i].name, "content-range")) continue;
            std::string v = headers[i].value;                                              // "bytes START-END/TOTAL"
            if (v.compare(0, 6, "bytes ") != 0) break;
            size_t dash = v.find('-');
            if (dash == std::string::npos) break;
            if (!parse_u64(v.substr(6, dash - 6), start, 10)) start = UINT64_MAX;
            break;
        }
        if (start != c->offset) { c->badStatus = status; return 1; }                       // wrong slice: refuse to append it
        c->file = open_file(c->part, "ab");
        c->resumed = true;
    } else {
        if (c->offset > 0) { c->offset = 0; c->resumed = false; }                           // server ignored Range: start over
        enet_sha256_init(&c->hash);
        c->file = open_file(c->part, "wb");
        c->total = 0;
    }
    if (!c->file) { c->ioFail = true; return 1; }
    c->total = c->offset;
    c->req->bytesTotal = c->expectedSize ? c->expectedSize : (contentLength == UINT64_MAX ? 0 : c->offset + contentLength);
    return 0;
}

int dl_on_data(void* user, const uint8_t* data, size_t len) {
    DlCtx* c = (DlCtx*) user;
    if (c->expectedSize && c->total + len > c->expectedSize) { c->tooLarge = true; return 1; }
    if (fwrite(data, 1, len, c->file) != len) { c->ioFail = true; return 1; }
    enet_sha256_update(&c->hash, data, len);
    c->total += len;
    post_progress(c, false);
    return 0;
}

void run_download(ENetHttpRequest* req) {
    ENetHttpDownloadResult& out = req->dlRes;
    memset(&out, 0, sizeof(out));
    uint8_t expect[32];
    bool haveHash = !req->dlSha.empty() && parse_hex_sha256(req->dlSha, expect);
    std::string part = req->dlDest + ".part";

    for (int pass = 0; pass < 2; ++pass) {
        if (req->cancel.load()) { out.error = E::ENET_HTTP_ERR_CANCELED; return; }
        DlCtx c;
        c.req = req;
        c.part = part;
        c.expectedSize = req->dlExpectedSize;
        enet_sha256_init(&c.hash);
        if (pass == 0 && haveHash) c.offset = hash_existing(part, &c.hash);     // resume only when the result can be verified
        else remove_file(part);
        c.total = c.offset;

        Job job = req->job;
        ENetHttpSink sink;
        sink.on_headers = dl_on_headers; sink.on_data = dl_on_data; sink.user = &c;
        job.hasSink = true; job.sink = sink;
        std::string range;
        if (c.offset > 0) { range = "bytes=" + std::to_string(c.offset) + "-"; job.headers.push_back({"Range", range}); }

        Result res;
        perform(job, &req->cancel, res);
        if (c.file) { if (fclose(c.file) != 0) c.ioFail = true; c.file = nullptr; }
        out.http_status = res.status;

        if (c.rangeBad) { remove_file(part); if (pass == 0) continue; out.error = E::ENET_HTTP_ERR_INVALID_RESPONSE; return; }
        if (c.ioFail)   { out.error = E::ENET_HTTP_ERR_IO; return; }
        if (c.tooLarge) { remove_file(part); out.error = E::ENET_HTTP_ERR_INTEGRITY; return; }
        if (c.badStatus) { out.error = E::ENET_HTTP_ERR_INVALID_RESPONSE; out.http_status = c.badStatus; return; }
        if (res.error != E::ENET_HTTP_OK) { out.error = res.error; return; }       // network trouble: keep the .part for a later resume

        uint8_t digest[32];
        enet_sha256_final(&c.hash, digest);
        bool sizeOk = !req->dlExpectedSize || c.total == req->dlExpectedSize;
        bool hashOk = !haveHash || enet_secure_equals(digest, expect, 32);
        if (!sizeOk || !hashOk) {
            remove_file(part);                                                      // never leave unverified data behind
            if (c.resumed && pass == 0) continue;                                   // a resumed file may have mixed versions: retry clean once
            out.error = E::ENET_HTTP_ERR_INTEGRITY;
            return;
        }
        if (!replace_file(part, req->dlDest)) { out.error = E::ENET_HTTP_ERR_IO; return; }
        out.error = E::ENET_HTTP_OK;
        out.bytes = c.total;
        out.verified = (haveHash || req->dlExpectedSize) ? 1 : 0;
        out.resumed = c.resumed ? 1 : 0;
        req->bytesDone = c.total;
        post_progress(&c, true);
        return;
    }
}

} // namespace

ENetHttpRequest* enet_http_download_async(const ENetHttpDownloadDesc* d, ENetHttpProgressCallback progress, ENetHttpDownloadCallback done, void* userData) {
    if (!d || !d->url || !d->dest_path || !d->dest_path[0]) return nullptr;
    uint8_t tmp[32];
    if (d->expected_sha256_hex && d->expected_sha256_hex[0] && !parse_hex_sha256(d->expected_sha256_hex, tmp)) return nullptr;

    ENetHttpRequestDesc rd;
    enet_http_request_desc_init(&rd, ENetHttpMethod::ENET_HTTP_GET, d->url);
    rd.headers = d->headers; rd.header_count = d->header_count; rd.tls = d->tls;
    rd.timeout_ms = d->timeout_ms ? d->timeout_ms : 60000;
    rd.max_redirects = d->max_redirects; rd.flags = d->flags;
    ENetHttpSink dummy{nullptr, nullptr, nullptr};
    rd.sink = &dummy;                                    // validated and replaced per attempt in run_download
    Job job;
    if (make_job(&rd, job) != E::ENET_HTTP_OK) return nullptr;

    ENetHttpRequest* req = new ENetHttpRequest();
    req->isDownload = true;
    req->job = std::move(job);
    req->dlUrl = d->url;
    req->dlDest = d->dest_path;
    req->dlSha = d->expected_sha256_hex ? d->expected_sha256_hex : "";
    req->dlExpectedSize = d->expected_size;
    req->bytesTotal = d->expected_size;
    req->progress = progress;
    req->doneCb = done;
    req->user = userData;
    {
        std::lock_guard<std::mutex> g(g_poolMutex);
        if (!g_pool) g_pool = start_pool(nullptr);
        req->job.userAgent = g_pool->userAgent;
        live_add(req);
    }
    bool ok = pool_submit([req] {
        run_download(req);
        live_remove(req);
        post_main([req](bool deliver) {
            if (deliver && !req->released.load() && req->doneCb) req->doneCb(&req->dlRes, req->user);
            unref(req);
        });
    });
    if (!ok) { live_remove(req); delete req; return nullptr; }
    return req;
}
