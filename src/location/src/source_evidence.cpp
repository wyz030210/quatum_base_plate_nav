#include "location/source_evidence.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace {
constexpr double timeout_sec = 1.0;
std::vector<std::string> fields(const std::string & sentence) {
    std::vector<std::string> result;
    std::stringstream stream(sentence.substr(1, sentence.find('*') - 1));
    std::string value;
    while (std::getline(stream, value, ',')) result.push_back(value);
    return result;
}
bool number(const std::string & text, double & value) {
    try {
        size_t used = 0;
        value = std::stod(text, &used);
        return used == text.size() && std::isfinite(value);
    } catch (...) {return false;}
}
int64_t epoch(const std::string & text) {
    double value = 0;
    if (text.size() < 6 || !number(text, value) || value < 0 || value >= 240000) return -1;
    int hours = static_cast<int>(value / 10000);
    int minutes = static_cast<int>(value / 100) % 100;
    double seconds = value - hours * 10000 - minutes * 100;
    if (hours > 23 || minutes > 59 || seconds >= 60) return -1;
    return std::llround((hours * 3600 + minutes * 60 + seconds) * 1000000);
}
bool coordinates(const std::vector<std::string> & f, size_t first) {
    double lat = 0, lon = 0;
    if (f.size() <= first + 3 || !number(f[first], lat) || !number(f[first + 2], lon)) return false;
    if ((f[first + 1] != "N" && f[first + 1] != "S") ||
        (f[first + 3] != "E" && f[first + 3] != "W")) return false;
    return lat >= 0 && lon >= 0 && lat <= 9000 && lon <= 18000 &&
        std::fmod(lat, 100) < 60 && std::fmod(lon, 100) < 60;
}
bool checksum(const std::string & text) {
    auto star = text.find('*');
    if (text.empty() || text[0] != '$' || star == std::string::npos || star + 2 >= text.size()) return false;
    unsigned value = 0;
    for (size_t i = 1; i < star; ++i) value ^= static_cast<unsigned char>(text[i]);
    try {
        size_t used = 0;
        auto expected = std::stoul(text.substr(star + 1, 2), &used, 16);
        return used == 2 && value == expected;
    } catch (...) {return false;}
}
double age(const SourceMeasurement & source) {
    if (!source.sequence) return -1;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - source.received).count();
}
}

void SourceEvidence::connect() {
    const auto generation = state_.connection_generation + 1;
    const auto gaps = state_.discontinuity_sequence + 1;
    state_ = RtkSourceState{};
    state_.connected = true;
    state_.connection_generation = generation;
    state_.discontinuity_sequence = gaps;
    progressing_positions_ = 0;
    // High-water epochs intentionally survive reconnect. Cached pre-disconnect
    // position/quality cannot establish a new connection's measurement proof.
}
void SourceEvidence::disconnect() {
    state_.connected = false;
    state_.ready = false;
    ++state_.discontinuity_sequence;
    for (auto * source : {&state_.position, &state_.quality, &state_.heading}) {
        source->valid = false;
        source->reason = "disconnected";
    }
}
void SourceEvidence::update(SourceMeasurement & source, int64_t stamp, bool valid,
                            const std::string & reason, bool timeless) {
    if (!valid) {
        if (source.valid) ++state_.discontinuity_sequence;
        source.valid = false;
        source.reason = reason;
        return;
    }
    if (!timeless && stamp <= source.epoch_us) return;
    if (source.valid && age(source) > timeout_sec) ++state_.discontinuity_sequence;
    source.epoch_us = stamp;
    source.received = std::chrono::steady_clock::now();
    ++source.sequence;
    source.valid = true;
    source.reason.clear();
}
void SourceEvidence::observe(const std::string & sentence) {
    if (sentence.size() < 6) return;
    const auto kind = sentence.substr(3, 3);
    if (kind != "GGA" && kind != "RMC" && kind != "HDT") return;
    const auto f = fields(sentence);
    if (!checksum(sentence)) {
        // Corrupt serial traffic cannot advance positive evidence.
        return;
    }
    if (kind == "HDT") {
        double heading = 0;
        const bool good = f.size() >= 3 && number(f[1], heading) &&
            heading >= 0 && heading < 360 && f[2] == "T";
        update(state_.heading, -1, good, "invalid_hdt", true);
        return;
    }
    int64_t stamp = f.size() > 1 ? epoch(f[1]) : -1;
    constexpr int64_t day = 86400000000LL;
    const auto epoch_now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(epoch_now - epoch_received_).count();
    if (stamp >= 0 && last_position_ >= 0) {
        stamp += (last_position_ / day) * day;
        // Midnight is accepted only across a continuously observed narrow
        // boundary. Long outages/reset clocks stay invalid until explicit
        // process/profile reinitialization; no date is invented from traffic.
        if (last_position_ % day >= day - 60000000 && stamp % day < 60000000 &&
            elapsed <= timeout_sec) stamp += day;
        if (stamp > last_position_ && (stamp - last_position_) / 1e6 > std::max(5.0, elapsed + 2.0)) {
            update(state_.position, stamp, false, "position_epoch_jump");
            return;
        }
    }
    int64_t & high_water = kind == "GGA" ? last_gga_ : last_rmc_;
    const bool regressed = stamp < high_water;
    bool good = false;
    if (kind == "GGA") {
        double quality = 0;
        const bool quality_numeric = f.size() > 6 && number(f[6], quality) &&
            quality >= 0 && quality <= 8 && std::floor(quality) == quality;
        state_.fix_quality = quality_numeric ? static_cast<int>(quality) : 0;
        good = stamp >= 0 && f.size() >= 10 && coordinates(f, 2) && state_.fix_quality > 0;
        const bool fixed = good && state_.fix_quality == 4 && !regressed;
        if (!fixed || stamp > last_gga_) {
            update(state_.quality, stamp, fixed, regressed ? "gga_epoch_regression" : "gga_not_fixed");
        }
    } else {
        good = stamp >= 0 && f.size() >= 10 && f[2] == "A" && coordinates(f, 3);
    }
    if (regressed) {
        update(state_.position, stamp, false, "position_epoch_regression");
        return;
    }
    if (!good) {
        update(state_.position, stamp, false, "invalid_position");
        return;
    }
    if (stamp == high_water) return;
    high_water = stamp;
    if (stamp <= last_position_) return;
    last_position_ = stamp;
    epoch_received_ = epoch_now;
    const auto old_seq = state_.position.sequence;
    update(state_.position, stamp, true, "");
    if (state_.position.sequence != old_seq) ++progressing_positions_;
}
RtkSourceState SourceEvidence::snapshot() const {
    auto result = state_;
    for (auto * source : {&result.position, &result.quality, &result.heading}) {
        source->age_sec = age(*source);
        if (source->age_sec < 0 || source->age_sec > timeout_sec) {
            source->valid = false;
            source->reason = "source_expired_or_unavailable";
        }
    }
    if (result.quality.valid && std::abs(result.position.epoch_us - result.quality.epoch_us) >
        static_cast<int64_t>(timeout_sec * 1000000)) {
        result.quality.valid = false;
        result.quality.reason = "quality_epoch_not_associated_with_position";
    }
    result.ready = result.connected && result.position.valid && result.quality.valid &&
        result.heading.valid && progressing_positions_ >= 2 && result.quality.sequence >= 2;
    return result;
}
