#ifndef LOCATION_SOURCE_EVIDENCE_H
#define LOCATION_SOURCE_EVIDENCE_H

#include <chrono>
#include <cstdint>
#include <string>

// Measurement provenance only. This does not estimate or change the legacy pose.
struct SourceMeasurement {
    uint64_t sequence = 0;
    int64_t epoch_us = -1;
    bool valid = false;
    double age_sec = -1;
    std::string reason = "unavailable";
    std::chrono::steady_clock::time_point received{};
};

struct RtkSourceState {
    bool connected = false;
    bool ready = false;
    uint64_t connection_generation = 0;
    uint64_t discontinuity_sequence = 0;
    SourceMeasurement position, quality, heading;
    int fix_quality = 0;
};

class SourceEvidence {
public:
    void connect();
    void disconnect();
    void observe(const std::string & sentence);
    RtkSourceState snapshot() const;
private:
    RtkSourceState state_;
    int64_t last_gga_ = -1, last_rmc_ = -1, last_position_ = -1;
    std::chrono::steady_clock::time_point epoch_received_{};
    unsigned progressing_positions_ = 0;
    void update(SourceMeasurement & source, int64_t epoch, bool valid,
                const std::string & reason, bool timeless = false);
};
#endif
