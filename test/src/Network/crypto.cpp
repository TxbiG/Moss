/**
 @file  crypto.cpp
 @brief Moss_Network crypto & encoding helpers: base64, SHA-1, SHA-256, HMAC-SHA256, hex, secure random.
*/
#define ENET_BUILDING_LIB 1
#include <string.h>
#include <stdint.h>
#include <Moss/Moss_Network.h>

#if defined(_WIN32)
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
  #include <bcrypt.h>
  #ifdef _MSC_VER
    #pragma comment(lib, "bcrypt.lib")
  #endif
#elif defined(__APPLE__) || defined(__ANDROID__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__)
  #include <stdlib.h>   // arc4random_buf
#else
  #include <stdio.h>
  #include <errno.h>
  #if defined(__linux__)
    #include <sys/random.h>
  #endif
#endif

// ------------------------------------------------------------------
// base64 (RFC 4648, padded)
// ------------------------------------------------------------------
static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t enet_base64_encoded_size(size_t n) { return ((n + 2) / 3) * 4; }

size_t enet_base64_encode(const void* data, size_t length, char* out, size_t outCapacity) {
    const uint8_t* in = (const uint8_t*) data;
    size_t need = enet_base64_encoded_size(length);
    if (out == NULL || outCapacity < need + 1) return 0;

    size_t o = 0, i = 0;
    for (; i + 2 < length; i += 3) {
        uint32_t v = ((uint32_t) in[i] << 16) | ((uint32_t) in[i + 1] << 8) | in[i + 2];
        out[o++] = kB64[(v >> 18) & 63]; out[o++] = kB64[(v >> 12) & 63];
        out[o++] = kB64[(v >> 6) & 63];  out[o++] = kB64[v & 63];
    }
    if (i < length) {
        uint32_t v = (uint32_t) in[i] << 16;
        if (i + 1 < length) v |= (uint32_t) in[i + 1] << 8;
        out[o++] = kB64[(v >> 18) & 63];
        out[o++] = kB64[(v >> 12) & 63];
        out[o++] = (i + 1 < length) ? kB64[(v >> 6) & 63] : '=';
        out[o++] = '=';
    }
    out[o] = '\0';
    return o;
}

static int b64val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

int enet_base64_decode(const char* text, size_t length, uint8_t* out, size_t outCapacity, size_t* outLength) {
    if (length % 4 != 0) return -1;
    size_t pad = 0;
    if (length >= 1 && text[length - 1] == '=') pad++;
    if (length >= 2 && text[length - 2] == '=') pad++;
    size_t outLen = (length / 4) * 3 - pad;
    if (outLen > outCapacity) return -1;

    size_t o = 0;
    for (size_t i = 0; i < length; i += 4) {
        bool last = (i + 4 == length);
        int a = b64val(text[i]), b = b64val(text[i + 1]);
        int c = (last && pad >= 2) ? 0 : b64val(text[i + 2]);
        int d = (last && pad >= 1) ? 0 : b64val(text[i + 3]);
        if (a < 0 || b < 0 || c < 0 || d < 0) return -1;
        if (last && pad == 2 && text[i + 2] != '=') return -1;
        if (last && pad >= 1 && text[i + 3] != '=') return -1;
        uint32_t v = ((uint32_t) a << 18) | ((uint32_t) b << 12) | ((uint32_t) c << 6) | (uint32_t) d;
        // canonical encodings only: unused low bits must be zero
        if (last && pad == 2 && (v & 0xFFFF) != 0) return -1;
        if (last && pad == 1 && (v & 0xFF) != 0) return -1;
        if (o < outLen) out[o++] = (uint8_t) (v >> 16);
        if (o < outLen) out[o++] = (uint8_t) (v >> 8);
        if (o < outLen) out[o++] = (uint8_t) v;
    }
    if (outLength) *outLength = outLen;
    return 0;
}

// ------------------------------------------------------------------
// SHA-1 (RFC 3174) - WebSocket handshake only
// ------------------------------------------------------------------
static inline uint32_t rol32(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

static void sha1_block(uint32_t h[5], const uint8_t* p) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i)
        w[i] = ((uint32_t) p[i * 4] << 24) | ((uint32_t) p[i * 4 + 1] << 16) | ((uint32_t) p[i * 4 + 2] << 8) | p[i * 4 + 3];
    for (int i = 16; i < 80; ++i) w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20)      { f = (b & c) | (~b & d);          k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d;                   k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else             { f = b ^ c ^ d;                   k = 0xCA62C1D6; }
        uint32_t t = rol32(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

void enet_sha1(const void* data, size_t length, uint8_t out[20]) {
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
    const uint8_t* p = (const uint8_t*) data;
    size_t remaining = length;
    while (remaining >= 64) { sha1_block(h, p); p += 64; remaining -= 64; }

    uint8_t tail[128] = {0};
    if (remaining) memcpy(tail, p, remaining);
    tail[remaining] = 0x80;
    size_t tailLen = (remaining < 56) ? 64 : 128;
    uint64_t bits = (uint64_t) length * 8;
    for (int i = 0; i < 8; ++i) tail[tailLen - 1 - i] = (uint8_t) (bits >> (8 * i));
    sha1_block(h, tail);
    if (tailLen == 128) sha1_block(h, tail + 64);

    for (int i = 0; i < 5; ++i) {
        out[i * 4] = (uint8_t) (h[i] >> 24); out[i * 4 + 1] = (uint8_t) (h[i] >> 16);
        out[i * 4 + 2] = (uint8_t) (h[i] >> 8); out[i * 4 + 3] = (uint8_t) h[i];
    }
}

// ------------------------------------------------------------------
// SHA-256 (FIPS 180-4)
// ------------------------------------------------------------------
static const uint32_t K256[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};
static inline uint32_t ror32(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void sha256_block(uint32_t st[8], const uint8_t* p) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
        w[i] = ((uint32_t) p[i * 4] << 24) | ((uint32_t) p[i * 4 + 1] << 16) | ((uint32_t) p[i * 4 + 2] << 8) | p[i * 4 + 3];
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = st[0], b = st[1], c = st[2], d = st[3], e = st[4], f = st[5], g = st[6], h = st[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t S1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t t1 = h + S1 + ch + K256[i] + w[i];
        uint32_t S0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
        uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2 = S0 + mj;
        h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    st[0] += a; st[1] += b; st[2] += c; st[3] += d; st[4] += e; st[5] += f; st[6] += g; st[7] += h;
}

void enet_sha256_init(ENetSha256* c) {
    static const uint32_t iv[8] = {0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    memcpy(c->state, iv, sizeof(iv));
    c->length = 0;
    c->bufferLength = 0;
}

void enet_sha256_update(ENetSha256* c, const void* data, size_t length) {
    const uint8_t* p = (const uint8_t*) data;
    c->length += length;
    if (c->bufferLength) {
        size_t take = 64 - c->bufferLength;
        if (take > length) take = length;
        memcpy(c->buffer + c->bufferLength, p, take);
        c->bufferLength += take; p += take; length -= take;
        if (c->bufferLength == 64) { sha256_block(c->state, c->buffer); c->bufferLength = 0; }
    }
    while (length >= 64) { sha256_block(c->state, p); p += 64; length -= 64; }
    if (length) { memcpy(c->buffer, p, length); c->bufferLength = length; }
}

void enet_sha256_final(ENetSha256* c, uint8_t out[32]) {
    uint64_t bits = c->length * 8;
    uint8_t pad[72] = {0x80};
    size_t padLen = (c->bufferLength < 56) ? (56 - c->bufferLength) : (120 - c->bufferLength);
    uint8_t lenBytes[8];
    for (int i = 0; i < 8; ++i) lenBytes[i] = (uint8_t) (bits >> (56 - 8 * i));
    uint64_t savedLength = c->length;
    enet_sha256_update(c, pad, padLen);
    enet_sha256_update(c, lenBytes, 8);
    c->length = savedLength;
    for (int i = 0; i < 8; ++i) {
        out[i * 4] = (uint8_t) (c->state[i] >> 24); out[i * 4 + 1] = (uint8_t) (c->state[i] >> 16);
        out[i * 4 + 2] = (uint8_t) (c->state[i] >> 8); out[i * 4 + 3] = (uint8_t) c->state[i];
    }
}

void enet_sha256(const void* data, size_t length, uint8_t out[32]) {
    ENetSha256 c; enet_sha256_init(&c); enet_sha256_update(&c, data, length); enet_sha256_final(&c, out);
}

void enet_hmac_sha256(const void* key, size_t keyLength, const void* data, size_t length, uint8_t out[32]) {
    uint8_t k[64] = {0};
    if (keyLength > 64) enet_sha256(key, keyLength, k); else if (keyLength) memcpy(k, key, keyLength);
    uint8_t ipad[64], opad[64];
    for (int i = 0; i < 64; ++i) { ipad[i] = k[i] ^ 0x36; opad[i] = k[i] ^ 0x5c; }
    uint8_t inner[32];
    ENetSha256 c;
    enet_sha256_init(&c); enet_sha256_update(&c, ipad, 64); enet_sha256_update(&c, data, length); enet_sha256_final(&c, inner);
    enet_sha256_init(&c); enet_sha256_update(&c, opad, 64); enet_sha256_update(&c, inner, 32); enet_sha256_final(&c, out);
}

// ------------------------------------------------------------------
// hex, constant-time compare, random, UUID
// ------------------------------------------------------------------
void enet_hex_encode(const uint8_t* data, size_t length, char* out) {
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < length; ++i) { out[i * 2] = hex[data[i] >> 4]; out[i * 2 + 1] = hex[data[i] & 15]; }
    out[length * 2] = '\0';
}

int enet_secure_equals(const void* a, const void* b, size_t length) {
    const volatile uint8_t* x = (const volatile uint8_t*) a;
    const volatile uint8_t* y = (const volatile uint8_t*) b;
    uint8_t diff = 0;
    for (size_t i = 0; i < length; ++i) diff |= (uint8_t) (x[i] ^ y[i]);
    return diff == 0;
}

int enet_random_bytes(void* out, size_t length) {
    if (length == 0) return 0;
    if (out == NULL) return -1;
#if defined(_WIN32)
    while (length > 0) {
        ULONG chunk = length > 0x10000000u ? 0x10000000u : (ULONG) length;
        if (BCryptGenRandom(NULL, (PUCHAR) out, chunk, BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0) return -1;
        out = (uint8_t*) out + chunk; length -= chunk;
    }
    return 0;
#elif defined(__APPLE__) || defined(__ANDROID__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__)
    arc4random_buf(out, length);
    return 0;
#else
    uint8_t* p = (uint8_t*) out;
    size_t left = length;
  #if defined(__linux__)
    while (left > 0) {
        ssize_t r = getrandom(p, left > 256 ? 256 : left, 0);
        if (r < 0) { if (errno == EINTR) continue; break; }
        p += r; left -= (size_t) r;
    }
    if (left == 0) return 0;
  #endif
    // Fallback for other POSIX systems (and very old kernels): /dev/urandom. Fails rather than using a weak generator.
    FILE* f = fopen("/dev/urandom", "rb");
    if (!f) return -1;
    size_t got = fread(p, 1, left, f);
    fclose(f);
    return got == left ? 0 : -1;
#endif
}

int enet_make_uuid_v4(char out[37]) {
    uint8_t b[16];
    if (enet_random_bytes(b, sizeof(b)) != 0) { out[0] = '\0'; return -1; }
    b[6] = (uint8_t) ((b[6] & 0x0F) | 0x40);   // version 4
    b[8] = (uint8_t) ((b[8] & 0x3F) | 0x80);   // RFC 4122 variant
    static const char hex[] = "0123456789abcdef";
    int o = 0;
    for (int i = 0; i < 16; ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10) out[o++] = '-';
        out[o++] = hex[b[i] >> 4]; out[o++] = hex[b[i] & 15];
    }
    out[o] = '\0';
    return 0;
}
