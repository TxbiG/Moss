/**
 @file  tls_openssl.cpp
 @brief Reference TLS backend for Moss_Network built on OpenSSL (1.1.1+ / 3.x). Optional: add it to the build and link
        libssl/libcrypto, then call enet_tls_set_backend(enet_tls_openssl_backend()).

 Verifies the certificate chain and the host name by default (SNI is sent for DNS names; IP literals are matched against
 IP SANs), requires TLS 1.2+, and reports a connection that ends WITHOUT close_notify as an error (-2) so truncation attacks
 are not mistaken for a complete response.

 Notes: on POSIX it sets SIGPIPE to ignored once (OpenSSL writes to the socket without MSG_NOSIGNAL). A fresh SSL_CTX is built per
 connection for simplicity; cache one per ENetTransportSecurity if you open very many connections. On Android / iOS / consoles
 prefer the platform's TLS (system trust store, certificate pinning policy, etc.) through your own ENetTlsBackend.
*/
#define ENET_BUILDING_LIB 1
#include <string.h>
#include <limits.h>
#include <Moss/Moss_Network.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/x509v3.h>
#ifndef _WIN32
  #include <signal.h>
  #include <arpa/inet.h>
#else
  #include <ws2tcpip.h>
#endif
#include <chrono>

namespace {

struct Conn { SSL_CTX* ctx; SSL* ssl; bool shutdownSent; };

bool is_ip_literal(const char* host) {
    unsigned char buf[sizeof(struct in6_addr)];
    return inet_pton(AF_INET, host, buf) == 1 || inet_pton(AF_INET6, host, buf) == 1;
}

uint64_t now_ms() {
    return (uint64_t) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void conn_free(Conn* c) {
    if (c->ssl) SSL_free(c->ssl);
    if (c->ctx) SSL_CTX_free(c->ctx);
    delete c;
}

void* tls_connect(ENetSocket sock, const char* host, const ENetTransportSecurity* sec, uint32_t timeoutMs) {
#if !defined(_WIN32) && defined(SIGPIPE)
    static bool once = (signal(SIGPIPE, SIG_IGN), true);
    (void) once;
#endif
    Conn* c = new Conn{nullptr, nullptr, false};
    c->ctx = SSL_CTX_new(TLS_client_method());
    if (!c->ctx) { conn_free(c); return nullptr; }
    SSL_CTX_set_min_proto_version(c->ctx, TLS1_2_VERSION);

    bool verify = !(sec && sec->skip_verification);
    if (verify) {
        SSL_CTX_set_verify(c->ctx, SSL_VERIFY_PEER, nullptr);
        if (sec && sec->ca_file) { if (SSL_CTX_load_verify_locations(c->ctx, sec->ca_file, nullptr) != 1) { conn_free(c); return nullptr; } }
        else if (SSL_CTX_set_default_verify_paths(c->ctx) != 1) { conn_free(c); return nullptr; }
    } else {
        SSL_CTX_set_verify(c->ctx, SSL_VERIFY_NONE, nullptr);
    }
    if (sec && sec->cert_file && sec->key_file) {
        if (SSL_CTX_use_certificate_chain_file(c->ctx, sec->cert_file) != 1 ||
            SSL_CTX_use_PrivateKey_file(c->ctx, sec->key_file, SSL_FILETYPE_PEM) != 1 ||
            SSL_CTX_check_private_key(c->ctx) != 1) { conn_free(c); return nullptr; }
    }

    c->ssl = SSL_new(c->ctx);
    if (!c->ssl || SSL_set_fd(c->ssl, (int) sock) != 1) { conn_free(c); return nullptr; }
    bool ip = is_ip_literal(host);
    if (verify) {
        bool ok = ip ? X509_VERIFY_PARAM_set1_ip_asc(SSL_get0_param(c->ssl), host) == 1 : SSL_set1_host(c->ssl, host) == 1;
        if (!ok) { conn_free(c); return nullptr; }
    }
    if (!ip) SSL_set_tlsext_host_name(c->ssl, host);

    uint64_t deadline = now_ms() + timeoutMs;
    for (;;) {
        ERR_clear_error();
        int r = SSL_connect(c->ssl);
        if (r == 1) break;
        int err = SSL_get_error(c->ssl, r);
        if (err != SSL_ERROR_WANT_READ && err != SSL_ERROR_WANT_WRITE) { conn_free(c); return nullptr; }
        uint64_t now = now_ms();
        if (now >= deadline) { conn_free(c); return nullptr; }
        uint32_t cond = (err == SSL_ERROR_WANT_WRITE) ? static_cast<uint32_t>(ENetSocketWait::SEND) : static_cast<uint32_t>(ENetSocketWait::RECEIVE);
        uint32_t slice = (uint32_t) (deadline - now < 100 ? deadline - now : 100);
        if (enet_socket_wait(sock, &cond, slice) < 0) { conn_free(c); return nullptr; }
    }
    if (verify && SSL_get_verify_result(c->ssl) != X509_V_OK) { conn_free(c); return nullptr; }
    return c;
}

int tls_read(void* connection, void* buf, size_t len) {
    Conn* c = (Conn*) connection;
    ERR_clear_error();
    int n = SSL_read(c->ssl, buf, len > (size_t) INT_MAX ? INT_MAX : (int) len);
    if (n > 0) return n;
    switch (SSL_get_error(c->ssl, n)) {
        case SSL_ERROR_WANT_READ: case SSL_ERROR_WANT_WRITE: return 0;
        case SSL_ERROR_ZERO_RETURN: return -1;             // authenticated close_notify: a clean end of stream
        default: return -2;                                // includes EOF without close_notify (possible truncation)
    }
}

int tls_write(void* connection, const void* data, size_t len) {
    Conn* c = (Conn*) connection;
    ERR_clear_error();
    int n = SSL_write(c->ssl, data, len > (size_t) INT_MAX ? INT_MAX : (int) len);
    if (n > 0) return n;
    switch (SSL_get_error(c->ssl, n)) {
        case SSL_ERROR_WANT_READ: case SSL_ERROR_WANT_WRITE: return 0;
        default: return -1;
    }
}

void tls_close(void* connection) {
    Conn* c = (Conn*) connection;
    if (c->ssl && !c->shutdownSent) { ERR_clear_error(); SSL_shutdown(c->ssl); c->shutdownSent = true; }   // best-effort close_notify
    conn_free(c);
}

const ENetTlsBackend kBackend = { tls_connect, tls_read, tls_write, tls_close };

} // namespace

const ENetTlsBackend* enet_tls_openssl_backend(void) { return &kBackend; }
