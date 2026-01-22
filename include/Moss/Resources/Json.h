





#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <memory>
#include <vector>
#include <memory>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <limits>
#include <cstring>

// Add Encryption

enum class Type { Null, Bool, Number, String, Array, Object };


class Json {
public:
    using Array  = std::vector<std::shared_ptr<Json>>;
    using Object = std::unordered_map<std::string, std::shared_ptr<Json>>;

    // Constructors
    Json() : type(Type::Null) {}
    explicit Json(Type t) : type(t) {}
    Json(double num) : type(Type::Number), number(num) {}
    Json(bool b) : type(Type::Bool), boolean(b) {}
    Json(const std::string &s) : type(Type::String), str(s) {}

    // Factory
    static std::shared_ptr<Json> make_array()  { return std::make_shared<Json>(Type::Array); }
    static std::shared_ptr<Json> make_object() { return std::make_shared<Json>(Type::Object); }

    // Parsing
    static std::shared_ptr<Json> parse(const std::string &input);

    // Serialization
    std::string stringify() const;

    // Accessors
    Type get_type() const { return type; }
    double get_number() const { return number; }
    bool get_bool() const { return boolean; }
    const std::string& get_string() const { return str; }
    const Array& get_array() const { return array; }
    const Object& get_object() const { return object; }

    // Mutators
    void push_back(std::shared_ptr<Json> value) { array.push_back(value); }
    void set(const std::string& key, std::shared_ptr<Json> value) { object[key] = value; }

private:
    Type type;
    double number{};
    bool boolean{};
    std::string str;
    Array array;
    Object object;

    // Helpers
    static void skip_ws(const char **s) { while (std::isspace(**s)) (*s)++; }

    static std::string parse_string(const char **s);
    static std::shared_ptr<Json> parse_value(const char **s);
    static std::shared_ptr<Json> parse_array(const char **s);
    static std::shared_ptr<Json> parse_object(const char **s);
};

#if defined(__AVX2__) || defined(__SSE2__)
  #include <immintrin.h>
#endif
class SIMDJson {
public:
    // ---- Value storage
    using Array  = std::vector<std::unique_ptr<SIMDJson>>;
    struct Pair { std::string key; std::unique_ptr<SIMDJson> value; };
    using Object = std::vector<Pair>;

    // ---- Constructors
    SIMDJson() : type_(Type::Null) {}
    static std::unique_ptr<SIMDJson> make_null()   { return std::unique_ptr<SIMDJson>(new SIMDJson()); }
    static std::unique_ptr<SIMDJson> make_bool(bool b) { auto p = std::unique_ptr<SIMDJson>(new SIMDJson()); p->type_ = Type::Bool;   p->b_=b; return p; }
    static std::unique_ptr<SIMDJson> make_number(double x){ auto p=std::unique_ptr<SIMDJson>(new SIMDJson()); p->type_=Type::Number; p->num_=x; return p; }
    static std::unique_ptr<SIMDJson> make_string(std::string s){ auto p=std::unique_ptr<SIMDJson>(new SIMDJson()); p->type_=Type::String; p->str_=std::move(s); return p; }
    static std::unique_ptr<SIMDJson> make_array()  { auto p = std::unique_ptr<SIMDJson>(new SIMDJson()); p->type_=Type::Array;  return p; }
    static std::unique_ptr<SIMDJson> make_object() { auto p = std::unique_ptr<SIMDJson>(new SIMDJson()); p->type_=Type::Object; return p; }

    // ---- Introspection
    Type   type()   const { return type_; }
    bool   as_bool()const { if(type_!=Type::Bool)   throw std::runtime_error("not bool");   return b_; }
    double as_num() const { if(type_!=Type::Number) throw std::runtime_error("not number"); return num_; }
    const std::string& as_str() const { if(type_!=Type::String) throw std::runtime_error("not string"); return str_; }
    Array& as_arr()       { if(type_!=Type::Array)  throw std::runtime_error("not array");  return arr_; }
    const Array& as_arr() const { if(type_!=Type::Array)  throw std::runtime_error("not array");  return arr_; }
    Object& as_obj()      { if(type_!=Type::Object) throw std::runtime_error("not object"); return obj_; }
    const Object& as_obj()const{ if(type_!=Type::Object) throw std::runtime_error("not object"); return obj_; }

    // ---- Object helpers
    SIMDJson* find(const std::string& key) {
        if (type_ != Type::Object) return nullptr;
        for (auto& kv : obj_) if (kv.key == key) return kv.value.get();
        return nullptr;
    }
    const SIMDJson* find(const std::string& key) const {
        if (type_ != Type::Object) return nullptr;
        for (auto& kv : obj_) if (kv.key == key) return kv.value.get();
        return nullptr;
    }
    void set(const std::string& key, std::unique_ptr<SIMDJson> v) {
        if (type_ != Type::Object) type_ = Type::Object;
        for (auto& kv : obj_) if (kv.key == key) { kv.value = std::move(v); return; }
        obj_.push_back({key, std::move(v)});
    }

    // ---- Parsing / Serialization API
    static std::unique_ptr<SIMDJson> parse(const std::string& text) {
        const char* s = text.c_str();
        const char* end = s + text.size();
        skip_ws_simd(s, end);
        auto v = parse_value(s, end);
        if (!v) throw std::runtime_error("JSON parse error");
        skip_ws_simd(s, end);
        if (s != end) throw std::runtime_error("Trailing garbage after JSON");
        return v;
    }

    static std::unique_ptr<SIMDJson> from_file(const std::string& filename) {
        std::ifstream in(filename, std::ios::binary);
        if (!in) throw std::runtime_error("Cannot open file: " + filename);
        std::ostringstream ss; ss << in.rdbuf();
        return parse(ss.str());
    }

    bool to_file(const std::string& filename, bool pretty=false, int indent=0) const {
        std::ofstream out(filename, std::ios::binary);
        if (!out) return false;
        stringify(out, pretty, indent);
        return true;
    }

    std::string dump(bool pretty=false, int indent=0) const {
        std::ostringstream out;
        stringify(out, pretty, indent);
        return out.str();
    }

private:
    // ---- Data
    Type type_{Type::Null};
    bool b_{false};
    double num_{0.0};
    std::string str_;
    Array  arr_;
    Object obj_;

    // =========================
    // Parsing (recursive descent)
    // =========================
    static void skip_ws_scalar(const char*& s, const char* end) {
        while (s < end && (*s==' ' || *s=='\t' || *s=='\n' || *s=='\r')) ++s;
    }

    // SIMD-accelerated whitespace skip (AVX2/SSE2), falls back to scalar.
    static void skip_ws_simd(const char*& s, const char* end) {
    #if defined(__AVX2__)
        const __m256i sp   = _mm256_set1_epi8(' ');
        const __m256i tab  = _mm256_set1_epi8('\t');
        const __m256i cr   = _mm256_set1_epi8('\r');
        const __m256i lf   = _mm256_set1_epi8('\n');
        while (s + 32 <= end) {
            __m256i v = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(s));
            __m256i m = _mm256_or_si256(
                        _mm256_or_si256(_mm256_cmpeq_epi8(v, sp), _mm256_cmpeq_epi8(v, tab)),
                        _mm256_or_si256(_mm256_cmpeq_epi8(v, cr), _mm256_cmpeq_epi8(v, lf)));
            // mask bit=1 for whitespace
            uint32_t mask = _mm256_movemask_epi8(m);
            if (mask != 0xFFFFFFFFu) { // some non-ws encountered
                int first_non = __builtin_ctz(~mask);
                s += first_non;
                return;
            }
            s += 32;
        }
        skip_ws_scalar(s, end);
    #elif defined(__SSE2__)
        const __m128i sp   = _mm_set1_epi8(' ');
        const __m128i tab  = _mm_set1_epi8('\t');
        const __m128i cr   = _mm_set1_epi8('\r');
        const __m128i lf   = _mm_set1_epi8('\n');
        while (s + 16 <= end) {
            __m128i v = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s));
            __m128i m = _mm_or_si128(
                        _mm_or_si128(_mm_cmpeq_epi8(v, sp), _mm_cmpeq_epi8(v, tab)),
                        _mm_or_si128(_mm_cmpeq_epi8(v, cr), _mm_cmpeq_epi8(v, lf)));
            uint32_t mask = static_cast<uint32_t>(_mm_movemask_epi8(m));
            if (mask != 0xFFFFu) {
                int first_non = __builtin_ctz(~mask);
                s += first_non;
                return;
            }
            s += 16;
        }
        skip_ws_scalar(s, end);
    #else
        skip_ws_scalar(s, end);
    #endif
    }

    static std::unique_ptr<SIMDJson> parse_value(const char*& s, const char* end) {
        skip_ws_simd(s, end);
        if (s >= end) return nullptr;

        switch (*s) {
            case 'n': return parse_lit(s, end, "null",  Type::Null);
            case 't': return parse_lit(s, end, "true",  Type::Bool, true);
            case 'f': return parse_lit(s, end, "false", Type::Bool, false);
            case '"': return parse_string(s, end);
            case '{': return parse_object(s, end);
            case '[': return parse_array(s, end);
            default:  return parse_number(s, end);
        }
    }

    static std::unique_ptr<SIMDJson> parse_lit(const char*& s, const char* end,
                                           const char* word, Type t, bool b=false) {
        size_t L = std::strlen(word);
        if (static_cast<size_t>(end - s) < L || std::strncmp(s, word, L) != 0)
            throw std::runtime_error("Invalid literal");
        s += L;
        if (t == Type::Null)  return make_null();
        if (t == Type::Bool)  return make_bool(b);
        return nullptr;
    }

    static std::unique_ptr<SIMDJson> parse_number(const char*& s, const char* end) {
        const char* p = s;
        if (p < end && (*p=='-' || *p=='+')) ++p;
        bool has_digit=false;
        while (p<end && std::isdigit(static_cast<unsigned char>(*p))) { has_digit=true; ++p; }
        if (p<end && *p=='.') { ++p; while (p<end && std::isdigit(static_cast<unsigned char>(*p))) { has_digit=true; ++p; } }
        if (p<end && (*p=='e' || *p=='E')) { ++p; if (p<end && (*p=='+'||*p=='-')) ++p; while (p<end && std::isdigit(static_cast<unsigned char>(*p))) { has_digit=true; ++p; } }
        if (!has_digit) throw std::runtime_error("Invalid number");
        double val = std::strtod(s, nullptr);
        s = p;
        return make_number(val);
    }

    static std::unique_ptr<SIMDJson> parse_string(const char*& s, const char* end) {
        if (s>=end || *s!='"') throw std::runtime_error("Expected string");
        ++s; // skip opening quote
        std::string out;
        out.reserve(16);
        while (s<end) {
            char c = *s++;
            if (c == '"') break;
            if (c == '\\') {
                if (s>=end) throw std::runtime_error("Bad escape");
                char e = *s++;
                switch (e) {
                    case '"': out.push_back('"'); break;
                    case '\\':out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    // NOTE: \uXXXX not implemented here; add if you need unicode escapes.
                    default: throw std::runtime_error("Unsupported escape");
                }
            } else {
                out.push_back(c);
            }
        }
        return make_string(std::move(out));
    }

    static std::unique_ptr<SIMDJson> parse_array(const char*& s, const char* end) {
        if (*s != '[') throw std::runtime_error("Expected '['");
        ++s;
        skip_ws_simd(s, end);
        auto arr = make_array();
        if (s<end && *s==']') { ++s; return arr; }
        while (s<end) {
            auto item = parse_value(s, end);
            if (!item) throw std::runtime_error("Bad array item");
            arr->arr_.push_back(std::move(item));
            skip_ws_simd(s, end);
            if (s<end && *s==',') { ++s; skip_ws_simd(s,end); continue; }
            if (s<end && *s==']') { ++s; break; }
            throw std::runtime_error("Expected ',' or ']'");
        }
        return arr;
    }

    static std::unique_ptr<SIMDJson> parse_object(const char*& s, const char* end) {
        if (*s != '{') throw std::runtime_error("Expected '{'");
        ++s;
        skip_ws_simd(s, end);
        auto obj = make_object();
        if (s<end && *s=='}') { ++s; return obj; }
        while (s<end) {
            skip_ws_simd(s, end);
            auto key = parse_string(s, end);
            skip_ws_simd(s, end);
            if (s>=end || *s!=':') throw std::runtime_error("Expected ':'");
            ++s;
            auto val = parse_value(s, end);
            obj->obj_.push_back({ key->as_str(), std::move(val) });
            skip_ws_simd(s, end);
            if (s<end && *s==',') { ++s; continue; }
            if (s<end && *s=='}') { ++s; break; }
            throw std::runtime_error("Expected ',' or '}'");
        }
        return obj;
    }

    // =========================
    // Serialization
    // =========================
    void stringify(std::ostream& out, bool pretty, int indent) const {
        switch (type_) {
            case Type::Null:   out << "null"; break;
            case Type::Bool:   out << (b_ ? "true" : "false"); break;
            case Type::Number: out << num_; break;
            case Type::String: write_string(out, str_); break;
            case Type::Array:  {
                out << '[';
                for (size_t i=0;i<arr_.size();++i) {
                    if (i) out << ',';
                    if (pretty) out << '\n' << std::string(indent+2,' ');
                    arr_[i]->stringify(out, pretty, indent+2);
                }
                if (pretty && !arr_.empty()) out << '\n' << std::string(indent,' ');
                out << ']';
            } break;
            case Type::Object: {
                out << '{';
                for (size_t i=0;i<obj_.size();++i) {
                    if (i) out << ',';
                    if (pretty) out << '\n' << std::string(indent+2,' ');
                    write_string(out, obj_[i].key);
                    out << (pretty ? ": " : ":");
                    obj_[i].value->stringify(out, pretty, indent+2);
                }
                if (pretty && !obj_.empty()) out << '\n' << std::string(indent,' ');
                out << '}';
            } break;
        }
    }

    static void write_string(std::ostream& out, const std::string& s) {
        out << '"';
        for (char c : s) {
            switch (c) {
                case '"':  out << "\\\""; break;
                case '\\': out << "\\\\"; break;
                case '\b': out << "\\b";  break;
                case '\f': out << "\\f";  break;
                case '\n': out << "\\n";  break;
                case '\r': out << "\\r";  break;
                case '\t': out << "\\t";  break;
                default:   out << c;      break; // (no unicode-escape here)
            }
        }
        out << '"';
    }
};