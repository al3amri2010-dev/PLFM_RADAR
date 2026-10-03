#ifndef RADARSETTINGS_H
#define RADARSETTINGS_H

#include <cstdint>
#include "scan_config.h"

class RadarSettings {
public:
    // Default constructor with initial values
    RadarSettings();
    
    // Parse settings from USB CDC data
    bool parseFromUSB(const uint8_t* data, uint32_t length);
    
    // Getters for all settings
    double getSystemFrequency() const { return system_frequency; }
    double getChirpDuration1() const { return chirp_duration_1; }
    double getChirpDuration2() const { return chirp_duration_2; }
    uint32_t getChirpsPerPosition() const { return chirps_per_position; }
    double getFreqMin() const { return freq_min; }
    double getFreqMax() const { return freq_max; }
    double getPRF1() const { return prf1; }
    double getPRF2() const { return prf2; }
    double getMaxDistance() const { return max_distance; }
    double getMapSize() const { return map_size; }

    // Optional scan block ("SCN"); defaults to the legacy scan plan
    const scan_config_t& getScanConfig() const { return scan_config; }
    bool hasScanConfig() const { return has_scan_config; }

    // Packet sizes: legacy, and legacy + "SCN" scan block
    static constexpr uint32_t PACKET_SIZE_LEGACY = 82;
    static constexpr uint32_t PACKET_SIZE_WITH_SCAN = 98;
    
    // Check if settings are valid
    bool isValid() const { return settings_valid; }
    
    // Reset to default values
    void resetToDefaults();

private:
    // Radar system parameters
    double system_frequency;      // Hz
    double chirp_duration_1;      // seconds (long chirp)
    double chirp_duration_2;      // seconds (short chirp)
    uint32_t chirps_per_position;
    double freq_min;              // Hz
    double freq_max;              // Hz
    double prf1;                  // Hz
    double prf2;                  // Hz
    double max_distance;          // meters
    double map_size;              // meters

    scan_config_t scan_config;
    bool has_scan_config;
    
    bool settings_valid;
    
    // Helper methods
    bool validateSettings();
    double extractDouble(const uint8_t* data);
    uint32_t extractUint32(const uint8_t* data);
    uint16_t extractUint16(const uint8_t* data);
};

#endif // RADARSETTINGS_H