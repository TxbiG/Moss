#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Moss/Core/Variants/TMap.h>
#include <Moss/Core/Variants/Vector.h>
#include <stdexcept>

#include <filesystem>
namespace fs = std::filesystem;

// Add Encryption

/*! Config is used for Resource file such as .ini or .cfg */
class [[nodiscard]] Config {
public:
    Config(const std::string& filename,  const std::unordered_map<std::string, std::string>& defaults = {}) : filename(filename), defaults(defaults) { if (!fs::exists(filename)) { MOSS_ERROR(filename, "[Config] File not found. Creating default: %s"); createDefault(filename); } parseFile(filename); }

    std::string get(const std::string& key) const {
        auto it = data.find(key);
        if (it != data.end()) return it->second;
        auto def = defaults.find(key);
        if (def != defaults.end()) return def->second;
        throw std::runtime_error("Key not found: " + key);
    }

    void set(const std::string& key, const std::string& value) { data[key] = value; dirty = true; }

    void save() const {
        std::ofstream out(filename);
        if (!out.is_open()) throw std::runtime_error("Cannot open file for writing");

        std::string currentSection;
        for (const auto& line : lines) {
            if (line.empty() || line[0] == ';' || line[0] == '#') {
                out << line << '\n';
                continue;
            }

            if (line.front() == '[' && line.back() == ']') {
                currentSection = line.substr(1, line.size() - 2);
                out << line << '\n';
                continue;
            }

            auto eqPos = line.find('=');
            if (eqPos != std::string::npos) {
                std::string key = trim(line.substr(0, eqPos));
                std::string fullKey = currentSection.empty() ? key : currentSection + "." + key;
                auto it = data.find(fullKey);
                std::string newValue = (it != data.end()) ? it->second : line.substr(eqPos + 1);
                out << key << " = " << newValue << '\n';
            } else {
                out << line << '\n';
            }
        }
    }

private:
    std::string filename;
    std::unordered_map<std::string, std::string> data;
    std::unordered_map<std::string, std::string> defaults;
    std::vector<std::string> lines;
    bool dirty = false;

    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t\r\n");
        size_t end = s.find_last_not_of(" \t\r\n");
        return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }

    void parseFile(const std::string& filename) {
        std::ifstream in(filename);
        if (!in.is_open()) throw std::runtime_error("Cannot open file");

        std::string line, section;
        while (std::getline(in, line)) {
            lines.push_back(line);
            std::string trimmed = trim(line);

            if (trimmed.empty() || trimmed[0] == ';' || trimmed[0] == '#') continue;

            if (trimmed.front() == '[' && trimmed.back() == ']') {
                section = trimmed.substr(1, trimmed.size() - 2);
            } else {
                auto eqPos = trimmed.find('=');
                if (eqPos == std::string::npos) continue;

                std::string key = trim(trimmed.substr(0, eqPos));
                std::string value = trim(trimmed.substr(eqPos + 1));
                std::string fullKey = section.empty() ? key : section + "." + key;
                data[fullKey] = value;
            }
        }
    }

    void createDefault(const std::string& filename) {
        std::ofstream out(filename);
        if (!out.is_open()) throw std::runtime_error("Cannot create default config");

        std::string lastSection;
        for (const auto& [fullKey, value] : defaults) {
            auto dotPos = fullKey.find('.');
            std::string section, key;
            if (dotPos != std::string::npos) {
                section = fullKey.substr(0, dotPos);
                key = fullKey.substr(dotPos + 1);
            } else {
                key = fullKey;
            }

            if (section != lastSection) {
                if (!lastSection.empty()) out << "\n";
                out << "[" << section << "]\n";
                lines.push_back("[" + section + "]");
                lastSection = section;
            }

            out << key << " = " << value << "\n";
            lines.push_back(key + "=" + value);
        }
    }
};