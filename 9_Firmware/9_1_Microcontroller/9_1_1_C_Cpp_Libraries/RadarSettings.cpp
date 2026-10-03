#include "RadarSettings.h"
#include <cstring>

RadarSettings::RadarSettings() {
    resetToDefaults();
}

void RadarSettings::resetToDefaults() {
    system_frequency = 10.0e9;    // 10 GHz
    chirp_duration_1 = 30.0e-6;   // 30 �s
    chirp_duration_2 = 0.5e-6;    // 0.5 �s
    chirps_per_position = 32;
    freq_min = 10.0e6;           // 10 MHz
    freq_max = 30.0e6;           // 30 MHz
    prf1 = 1000.0;               // 1 kHz
    prf2 = 2000.0;               // 2 kHz
    max_distance = 50000.0;      // 50 km
    map_size = 50000.0;          // 50 km

    scan_config_default(&scan_config);
    has_scan_config = false;
    
    settings_valid = true;
}

bool RadarSettings::parseFromUSB(const uint8_t* data, uint32_t length) {
    // Accepted layouts (all multi-byte fields big-endian):
    //   legacy (82 B):  "SET" + 9 doubles + 1 uint32 + "END"
    //   scan   (98 B):  legacy fields + "SCN" + pattern u8 + uniform_beams u8
    //                   + uniform_max_angle_deg double + azimuth_positions u8
    //                   + step_half_period_us u16 + "END"
    // Parsing is all-or-nothing: a packet that fails any check leaves the
    // previously accepted settings untouched.
    if (data == nullptr ||
        (length != PACKET_SIZE_LEGACY && length != PACKET_SIZE_WITH_SCAN)) {
        return false;
    }
    
    // Check for start marker "SET"
    if (memcmp(data, "SET", 3) != 0) {
        return false;
    }
    
    // Check for end marker "END"
    if (memcmp(data + length - 3, "END", 3) != 0) {
        return false;
    }
    
    RadarSettings parsed;   // starts from defaults (legacy scan plan)
    uint32_t offset = 3;  // Skip "SET"
    
    // Extract all parameters in order
    parsed.system_frequency = extractDouble(data + offset);
    offset += 8;
    
    parsed.chirp_duration_1 = extractDouble(data + offset);
    offset += 8;
    
    parsed.chirp_duration_2 = extractDouble(data + offset);
    offset += 8;
    
    parsed.chirps_per_position = extractUint32(data + offset);
    offset += 4;
    
    parsed.freq_min = extractDouble(data + offset);
    offset += 8;
    
    parsed.freq_max = extractDouble(data + offset);
    offset += 8;
    
    parsed.prf1 = extractDouble(data + offset);
    offset += 8;
    
    parsed.prf2 = extractDouble(data + offset);
    offset += 8;
    
    parsed.max_distance = extractDouble(data + offset);
    offset += 8;
    
    parsed.map_size = extractDouble(data + offset);
    offset += 8;

    if (length == PACKET_SIZE_WITH_SCAN) {
        if (memcmp(data + offset, "SCN", 3) != 0) {
            return false;
        }
        offset += 3;
        parsed.scan_config.pattern = data[offset];
        offset += 1;
        parsed.scan_config.uniform_beams = data[offset];
        offset += 1;
        parsed.scan_config.uniform_max_angle_deg = (float)extractDouble(data + offset);
        offset += 8;
        parsed.scan_config.azimuth_positions = data[offset];
        offset += 1;
        parsed.scan_config.step_half_period_us = extractUint16(data + offset);
        parsed.has_scan_config = true;
    }
    
    // Validate the received settings
    if (!parsed.validateSettings()) {
        return false;
    }

    parsed.settings_valid = true;
    *this = parsed;
    return true;
}

// Written as !(lo <= v && v <= hi) so NaN is rejected (every NaN comparison is false)
static bool inRange(double v, double lo, double hi) {
    return lo <= v && v <= hi;
}

bool RadarSettings::validateSettings() {
    // Check for reasonable value ranges
    if (!inRange(system_frequency, 1e9, 100e9)) return false;
    if (!inRange(chirp_duration_1, 1e-6, 1000e-6)) return false;
    if (!inRange(chirp_duration_2, 0.1e-6, 10e-6)) return false;
    if (chirps_per_position < 1 || chirps_per_position > 256) return false;
    if (!inRange(freq_min, 1e6, 100e6)) return false;
    if (!(freq_max > freq_min) || !inRange(freq_max, 1e6, 100e6)) return false;
    if (!inRange(prf1, 100, 10000)) return false;
    if (!inRange(prf2, 100, 10000)) return false;
    if (!inRange(max_distance, 100, 100000)) return false;
    if (!inRange(map_size, 1000, 200000)) return false;
    if (scan_config_validate(&scan_config) != SCAN_OK) return false;
    
    return true;
}

double RadarSettings::extractDouble(const uint8_t* data) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; i++) {
        bits = (bits << 8) | data[i];
    }
    
    double value;
    memcpy(&value, &bits, sizeof(double));
    return value;
}

uint32_t RadarSettings::extractUint32(const uint8_t* data) {
    uint32_t value = 0;
    for (int i = 0; i < 4; i++) {
        value = (value << 8) | data[i];
    }
    return value;
}

uint16_t RadarSettings::extractUint16(const uint8_t* data) {
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}
