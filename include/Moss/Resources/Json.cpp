#include <Moss/Core/Resources/Json.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>

// --- STRING PARSING ---
std::string Json::parse_string(const char **s) {
    if (**s != '"') throw std::runtime_error("Expected string");
    (*s)++;
    std::string result;
    while (**s && **s != '"') {
        if (**s == '\\') {
            (*s)++;
            switch (**s) {
                case '"': result.push_back('"'); break;
                case '\\': result.push_back('\\'); break;
                case '/': result.push_back('/'); break;
                case 'b': result.push_back('\b'); break;
                case 'f': result.push_back('\f'); break;
                case 'n': result.push_back('\n'); break;
                case 'r': result.push_back('\r'); break;
                case 't': result.push_back('\t'); break;
                default: throw std::runtime_error("Invalid escape sequence");
            }
        } else {
            result.push_back(**s);
        }
        (*s)++;
    }
    if (**s == '"') (*s)++;
    else throw std::runtime_error("Unterminated string");
    return result;
}

// --- VALUE PARSING ---
std::shared_ptr<Json> Json::parse_value(const char **s) {
    skip_ws(s);
    if (**s == '"') {
        return std::make_shared<Json>(parse_string(s));
    } else if (**s == '-' || std::isdigit(**s)) {
        char* end;
        double num = std::strtod(*s, &end);
        *s = end;
        return std::make_shared<Json>(num);
    } else if (**s == '{') {
        return parse_object(s);
    } else if (**s == '[') {
        return parse_array(s);
    } else if (std::strncmp(*s, "true", 4) == 0) {
        *s += 4;
        return std::make_shared<Json>(true);
    } else if (std::strncmp(*s, "false", 5) == 0) {
        *s += 5;
        return std::make_shared<Json>(false);
    } else if (std::strncmp(*s, "null", 4) == 0) {
        *s += 4;
        return std::make_shared<Json>(Type::Null);
    }
    throw std::runtime_error("Invalid JSON value");
}

// --- ARRAY PARSING ---
std::shared_ptr<Json> Json::parse_array(const char **s) {
    (*s)++;
    skip_ws(s);
    auto arr = make_array();
    if (**s == ']') { (*s)++; return arr; }
    while (**s) {
        arr->push_back(parse_value(s));
        skip_ws(s);
        if (**s == ',') { (*s)++; skip_ws(s); }
        else if (**s == ']') { (*s)++; break; }
        else throw std::runtime_error("Expected , or ] in array");
    }
    return arr;
}

// --- OBJECT PARSING ---
std::shared_ptr<Json> Json::parse_object(const char **s) {
    (*s)++;
    skip_ws(s);
    auto obj = make_object();
    if (**s == '}') { (*s)++; return obj; }
    while (**s) {
        std::string key = parse_string(s);
        skip_ws(s);
        if (**s != ':') throw std::runtime_error("Expected : in object");
        (*s)++;
        skip_ws(s);
        obj->set(key, parse_value(s));
        skip_ws(s);
        if (**s == ',') { (*s)++; skip_ws(s); }
        else if (**s == '}') { (*s)++; break; }
        else throw std::runtime_error("Expected , or } in object");
    }
    return obj;
}

// --- ENTRY POINT ---
std::shared_ptr<Json> Json::parse(const std::string &input) {
    const char* s = input.c_str();
    return parse_value(&s);
}

// --- STRINGIFY ---
void Json::stringify_impl(std::string &out) const {
    switch (type) {
        case Type::Null: out += "null"; break;
        case Type::Bool: out += (boolean ? "true" : "false"); break;
        case Type::Number: {
            std::ostringstream ss;
            ss << number;
            out += ss.str();
            break;
        }
        case Type::String:
            out.push_back('"');
            for (char c : str) {
                switch (c) {
                    case '"': out += "\\\""; break;
                    case '\\': out += "\\\\"; break;
                    case '\n': out += "\\n"; break;
                    case '\r': out += "\\r"; break;
                    case '\t': out += "\\t"; break;
                    default: out.push_back(c); break;
                }
            }
            out.push_back('"');
            break;
        case Type::Array:
            out.push_back('[');
            for (size_t i = 0; i < array.size(); i++) {
                if (i > 0) out.push_back(',');
                array[i]->stringify_impl(out);
            }
            out.push_back(']');
            break;
        case Type::Object:
            out.push_back('{');
            {
                bool first = true;
                for (auto &kv : object) {
                    if (!first) out.push_back(',');
                    first = false;
                    out.push_back('"');
                    out += kv.first;
                    out += "\":";
                    kv.second->stringify_impl(out);
                }
            }
            out.push_back('}');
            break;
    }
}

std::string Json::stringify() const {
    std::string out;
    stringify_impl(out);
    return out;
}
