#ifndef PCKFILE_H
#define PCKFILE_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <stdexcept>
#include <random>
#include <limits>
#include <utility>

// ---------------------------
// PCK Format
// ---------------------------
// Header:
//   magic[4] = "PCK0"
//   version_u32 = 1
//   flags_u32   = 0 (reserved)
//   file_count_u32
//   header_salt[16]  (random)   -- reserved for future KDF use
//
// Per-file entry table:
//   name_len_u16
//   name_bytes (utf-8)
//   offset_u64        (absolute offset to file payload)
//   size_u64          (plaintext size)
//   enc_size_u64      (ciphertext size, equals size if not encrypted)
//   flags_u32         (bit0: encrypted)
//   iv[12]            (valid if encrypted)
//   tag[16]           (valid if encrypted)
//
// Payloads follow immediately after table; each file at its offset.
//
// Notes:
// - Compression is not included here (add later if needed).
// - If OpenSSL is unavailable, encryption is disabled but format stays valid.
//
// Encryption: AES-256-GCM per-file
// - key: 32 bytes provided by caller
// - iv:  12 random bytes per file
// - tag: 16 bytes GCM tag
//

class PCKFile {
public:
    struct Entry {
        std::string name;       // logical name inside pack (e.g., "shaders/sky.fxb")
        uint64_t    offset = 0; // payload offset in file
        uint64_t    size   = 0; // plaintext size
        uint64_t    enc_sz = 0; // stored bytes (may equal size if not encrypted)
        bool        encrypted = false;
        std::array<uint8_t,12> iv{};
        std::array<uint8_t,16> tag{};
    };

    using Key256 = std::array<uint8_t,32>;

    // --------- Create / Pack ---------
    // inputs: vector of (diskPath, packName)
    static void create(const std::string& outPck, const std::vector<std::pair<std::string,std::string>>& inputs, const Key256* key /* null -> no encryption */) {
        // Gather file data
        struct Pending {
            std::string name;
            std::vector<uint8_t> data;
            bool encrypt;
            std::array<uint8_t,12> iv{};
            std::array<uint8_t,16> tag{};
            std::vector<uint8_t> stored; // ciphertext or plaintext
        };
        std::vector<Pending> pendings;
        pendings.reserve(inputs.size());

        // Prepare RNG for IV/salt
        std::random_device rd;
        std::mt19937_64 gen(rd());
        auto rndfill = [&](uint8_t* dst, size_t n){
            for (size_t i=0;i<n;i++) dst[i] = static_cast<uint8_t>(gen() & 0xFF);
        };

        // Header salt (reserved for future PBKDF2/Argon2)
        std::array<uint8_t,16> header_salt{};
        rndfill(header_salt.data(), header_salt.size());

        for (auto& in : inputs) {
            Pending p;
            p.name = in.second;
            p.data = read_file(in.first);
            p.encrypt = (key != nullptr);

            if (p.encrypt) {
                // Encrypt with AES-256-GCM if OpenSSL available
            #if defined(MOSS_USE_OPENSSL)
                rndfill(p.iv.data(), p.iv.size());
                p.stored.resize(p.data.size());
                int outlen = 0;

                EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
                if (!ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");
                if (EVP_EncryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) != 1)
                    throw std::runtime_error("EncryptInit failed");
                if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, (int)p.iv.size(), nullptr) != 1)
                    throw std::runtime_error("SET_IVLEN failed");
                if (EVP_EncryptInit_ex(ctx, nullptr, nullptr, key->data(), p.iv.data()) != 1)
                    throw std::runtime_error("Encrypt key/iv failed");

                const uint8_t* inptr  = p.data.data();
                uint8_t*       outptr = p.stored.data();
                int tmplen = 0;
                if (p.data.size()) {
                    if (EVP_EncryptUpdate(ctx, outptr, &outlen, inptr, (int)p.data.size()) != 1)
                        throw std::runtime_error("EncryptUpdate failed");
                }
                if (EVP_EncryptFinal_ex(ctx, outptr + outlen, &tmplen) != 1)
                    throw std::runtime_error("EncryptFinal failed");
                outlen += tmplen;

                if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, 16, p.tag.data()) != 1)
                    throw std::runtime_error("GET_TAG failed");

                p.stored.resize((size_t)outlen);
                EVP_CIPHER_CTX_free(ctx);
            #else
                // No OpenSSL: store plaintext; mark not encrypted
                p.encrypt = false;
                p.stored = p.data;
            #endif
            } else {
                p.stored = p.data;
            }

            pendings.push_back(std::move(p));
        }

        // Layout: write header, then table, then payloads.
        std::ofstream out(outPck, std::ios::binary);
        if (!out) throw std::runtime_error("Cannot open output: " + outPck);

        // Compute offsets: We need to know table size first.
        // Table size = sum of entry descriptors.
        // Each entry: u16 nameLen + name + u64 off + u64 size + u64 enc + u32 flags + iv[12] + tag[16]
        // iv/tag present in file regardless (zeroed if not encrypted) for simplicity.
        uint32_t file_count = u32(pendings.size());
        size_t table_bytes = 0;
        for (auto& p : pendings) {
            table_bytes += 2;                          // name len
            table_bytes += p.name.size();              // name
            table_bytes += 8 + 8 + 8 + 4;              // offset,size,enc,flags
            table_bytes += 12 + 16;                    // iv, tag
        }

        // Header
        char magic[4] = {'P','C','K','0'};
        write_bytes(out, magic, sizeof(magic));
        uint32_t version = 1;
        uint32_t flags   = 0;
        write_pod(out, version);
        write_pod(out, flags);
        write_pod(out, file_count);
        write_bytes(out, header_salt.data(), header_salt.size());

        // Reserve space to later compute payload offsets
        std::streampos table_start = out.tellp();

        // We’ll build entries with offsets after we know table_start + table_bytes
        uint64_t payload_start = (uint64_t)table_start + (uint64_t)table_bytes;

        // First pass: compute offsets
        uint64_t cursor = payload_start;
        std::vector<Entry> entries;
        entries.reserve(pendings.size());
        for (auto& p : pendings) {
            Entry e;
            e.name = p.name;
            e.offset = cursor;
            e.size   = (uint64_t)p.data.size();
            e.enc_sz = (uint64_t)p.stored.size();
            e.encrypted = p.encrypt;
            e.iv = p.iv;
            e.tag = p.tag;
            entries.push_back(e);
            cursor += e.enc_sz;
        }

        // Write table
        for (size_t i=0;i<entries.size();++i) {
            const auto& e = entries[i];
            uint16_t nlen = (uint16_t)e.name.size();
            write_pod(out, nlen);
            write_bytes(out, e.name.data(), nlen);
            write_pod(out, e.offset);
            write_pod(out, e.size);
            write_pod(out, e.enc_sz);
            uint32_t eflags = e.encrypted ? 1u : 0u;
            write_pod(out, eflags);
            write_bytes(out, e.iv.data(), e.iv.size());
            write_bytes(out, e.tag.data(), e.tag.size());
        }

        // Write payloads
        for (size_t i=0;i<pendings.size();++i) {
            out.seekp(entries[i].offset, std::ios::beg);
            write_bytes(out, pendings[i].stored.data(), pendings[i].stored.size());
        }
    }

    // --------- Open / Read ---------
    void open(const std::string& pckPath, const Key256* key /*optional*/) {
        key_ = key ? *key : Key256{};
        key_present_ = (key != nullptr);

        f_.close();
        f_.clear();
        f_.open(pckPath, std::ios::binary);
        if (!f_) throw std::runtime_error("PckArchive::open: cannot open " + pckPath);

        // Read header
        char magic[4];
        read_bytes(f_, magic, 4);
        if (std::memcmp(magic,"PCK0",4) != 0)
            throw std::runtime_error("Not a PCK0 file");

        uint32_t version=0, flags=0, count=0;
        read_pod(f_, version);
        read_pod(f_, flags);
        read_pod(f_, count);
        read_bytes(f_, header_salt_.data(), header_salt_.size());

        if (version != 1) throw std::runtime_error("Unsupported PCK version");

        entries_.clear();
        entries_.reserve(count);

        // Read table
        for (uint32_t i=0;i<count;i++) {
            Entry e;
            uint16_t nlen=0;
            read_pod(f_, nlen);
            e.name.resize(nlen);
            read_bytes(f_, e.name.data(), nlen);
            read_pod(f_, e.offset);
            read_pod(f_, e.size);
            read_pod(f_, e.enc_sz);
            uint32_t eflags=0; read_pod(f_, eflags);
            e.encrypted = (eflags & 1u) != 0u;
            read_bytes(f_, e.iv.data(), e.iv.size());
            read_bytes(f_, e.tag.data(), e.tag.size());
            entries_.push_back(std::move(e));
        }
    }

    const std::vector<Entry>& list() const { return entries_; }

    // Extract one file to memory
    std::vector<uint8_t> extract(const std::string& name) const {
        const Entry* e = find(name);
        if (!e) throw std::runtime_error("extract: not found: " + name);

        f_.clear();
        f_.seekg((std::streamoff)e->offset, std::ios::beg);
        std::vector<uint8_t> enc(e->enc_sz);
        if (e->enc_sz && !f_.read(reinterpret_cast<char*>(enc.data()), enc.size()))
            throw std::runtime_error("extract: short read");

        if (!e->encrypted) {
            // Plain
            return enc;
        }

        // Encrypted
        if (!key_present_) throw std::runtime_error("extract: key required");
    #if defined(MOSS_USE_OPENSSL)
        std::vector<uint8_t> plain(e->size);
        int outlen = 0, tmplen=0;
        EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
        if (!ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");
        if (EVP_DecryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) != 1)
            throw std::runtime_error("DecryptInit failed");
        if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, (int)e->iv.size(), nullptr) != 1)
            throw std::runtime_error("SET_IVLEN failed");
        if (EVP_DecryptInit_ex(ctx, nullptr, nullptr, key_.data(), e->iv.data()) != 1)
            throw std::runtime_error("Decrypt key/iv failed");

        const uint8_t* inptr = enc.data();
        uint8_t*       outptr = plain.data();
        if (!enc.empty()) {
            if (EVP_DecryptUpdate(ctx, outptr, &outlen, inptr, (int)enc.size()) != 1)
                throw std::runtime_error("DecryptUpdate failed");
        }

        // Set expected tag before final
        if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_TAG, 16, const_cast<uint8_t*>(e->tag.data())) != 1)
            throw std::runtime_error("SET_TAG failed");

        if (EVP_DecryptFinal_ex(ctx, outptr + outlen, &tmplen) != 1) {
            EVP_CIPHER_CTX_free(ctx);
            throw std::runtime_error("DecryptFinal: authentication failed (wrong key or file tampered)");
        }
        outlen += tmplen;
        EVP_CIPHER_CTX_free(ctx);
        plain.resize((size_t)outlen);
        if (plain.size() != e->size) plain.resize((size_t)e->size); // clamp (GCM guarantees auth, size is trusted)
        return plain;
    #else
        (void)name;
        throw std::runtime_error("PckArchive: built without OpenSSL; cannot decrypt");
    #endif
    }

    // Extract one file to disk
    void extract_to(const std::string& name, const std::string& outPath) const {
        auto data = extract(name);
        write_file(outPath, data);
    }

private:
    const Entry* find(const std::string& n) const { for (const auto& e : entries_) if (e.name == n) return &e; return nullptr; }

    inline std::vector<uint8_t> read_file(const std::string& path) {
        std::ifstream in(path, std::ios::binary);
        if (!in) throw std::runtime_error("read_file: cannot open " + path);
        in.seekg(0, std::ios::end);
        auto sz = static_cast<size_t>(in.tellg());
        in.seekg(0, std::ios::beg);
        std::vector<uint8_t> buf(sz);
        if (sz && !in.read(reinterpret_cast<char*>(buf.data()), sz))
            throw std::runtime_error("read_file: short read");
        return buf;
    }

    void write_file(const std::string& path, const std::vector<uint8_t>& data) {
        std::ofstream out(path, std::ios::binary);
        if (!out) throw std::runtime_error("write_file: cannot open " + path);
        if (!data.empty() && !out.write(reinterpret_cast<const char*>(data.data()), data.size()))
            throw std::runtime_error("write_file: short write");
    }

    template<class T>
    inline void write_pod(std::ostream& out, const T& v) { out.write(reinterpret_cast<const char*>(&v), sizeof(T)); if (!out) throw std::runtime_error("write_pod: failed"); }

    template<class T>
    inline void read_pod(std::istream& in, T& v) { in.read(reinterpret_cast<char*>(&v), sizeof(T)); if (!in) throw std::runtime_error("read_pod: failed"); }
    inline void write_bytes(std::ostream& out, const void* p, size_t n) { out.write(reinterpret_cast<const char*>(p), n); if (!out) throw std::runtime_error("write_bytes: failed"); }
    inline void read_bytes(std::istream& in, void* p, size_t n) { in.read(reinterpret_cast<char*>(p), n); if (!in) throw std::runtime_error("read_bytes: failed"); }

    inline uint32_t u32(size_t x) {
        if (x > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("value too large for u32");
        return static_cast<uint32_t>(x);
    }

    // Members
    mutable std::ifstream f_;
    std::array<uint8_t,16> header_salt_{};
    std::vector<Entry> entries_;
    Key256 key_{};
    bool key_present_ = false;
};

#endif // PCKFILE_H