/*******************************************************************************
 * test_radar_settings_scan.cpp
 *
 * RadarSettings::parseFromUSB with the optional "SCN" scan block.
 *
 * Covers:
 *   - Legacy 82-byte packet still parses; scan plan stays at legacy default
 *   - 98-byte packet with a valid scan block is applied
 *   - Invalid scan block / NaN field / bad length / bad marker rejected
 *   - Rejection is all-or-nothing: previous settings are retained
 ******************************************************************************/
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "RadarSettings.h"

static void put_double(std::vector<uint8_t>& p, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int i = 7; i >= 0; i--) p.push_back((uint8_t)(bits >> (8 * i)));
}

static void put_u32(std::vector<uint8_t>& p, uint32_t v)
{
    for (int i = 3; i >= 0; i--) p.push_back((uint8_t)(v >> (8 * i)));
}

static std::vector<uint8_t> base_packet(double system_frequency = 10.5e9)
{
    std::vector<uint8_t> p = {'S', 'E', 'T'};
    put_double(p, system_frequency);
    put_double(p, 30.0e-6);
    put_double(p, 0.5e-6);
    put_u32(p, 32);
    put_double(p, 10.0e6);
    put_double(p, 30.0e6);
    put_double(p, 1000.0);
    put_double(p, 2000.0);
    put_double(p, 3000.0);
    put_double(p, 6000.0);
    return p;
}

static std::vector<uint8_t> legacy_packet(double system_frequency = 10.5e9)
{
    std::vector<uint8_t> p = base_packet(system_frequency);
    p.insert(p.end(), {'E', 'N', 'D'});
    return p;
}

static std::vector<uint8_t> scan_packet(uint8_t pattern, uint8_t beams, double max_deg,
                                        uint8_t az, uint16_t half_us)
{
    std::vector<uint8_t> p = base_packet();
    p.insert(p.end(), {'S', 'C', 'N', pattern, beams});
    put_double(p, max_deg);
    p.push_back(az);
    p.push_back((uint8_t)(half_us >> 8));
    p.push_back((uint8_t)(half_us & 0xFF));
    p.insert(p.end(), {'E', 'N', 'D'});
    return p;
}

static bool parse(RadarSettings& s, const std::vector<uint8_t>& p)
{
    return s.parseFromUSB(p.data(), (uint32_t)p.size());
}

int main()
{
    std::printf("=== RadarSettings scan block ===\n");

    std::printf("  Test 1: legacy packet keeps legacy scan plan... ");
    {
        RadarSettings s;
        std::vector<uint8_t> p = legacy_packet();
        assert(p.size() == RadarSettings::PACKET_SIZE_LEGACY);
        assert(parse(s, p));
        assert(s.isValid());
        assert(!s.hasScanConfig());
        assert(s.getScanConfig().pattern == SCAN_PATTERN_LEGACY);
        assert(s.getScanConfig().azimuth_positions == 50);
        assert(s.getMaxDistance() == 3000.0);
    }
    std::printf("PASS\n");

    std::printf("  Test 2: valid scan block applied... ");
    {
        RadarSettings s;
        std::vector<uint8_t> p = scan_packet(SCAN_PATTERN_UNIFORM, 15, 44.4, 40, 750);
        assert(p.size() == RadarSettings::PACKET_SIZE_WITH_SCAN);
        assert(parse(s, p));
        assert(s.hasScanConfig());
        assert(s.getScanConfig().pattern == SCAN_PATTERN_UNIFORM);
        assert(s.getScanConfig().uniform_beams == 15);
        assert(std::fabs(s.getScanConfig().uniform_max_angle_deg - 44.4f) < 1e-4f);
        assert(s.getScanConfig().azimuth_positions == 40);
        assert(s.getScanConfig().step_half_period_us == 750);
    }
    std::printf("PASS\n");

    std::printf("  Test 3: invalid scan block rejected, previous kept... ");
    {
        RadarSettings s;
        assert(parse(s, scan_packet(SCAN_PATTERN_UNIFORM, 15, 44.4, 40, 750)));
        /* 25 azimuth positions leave gaps */
        assert(!parse(s, scan_packet(SCAN_PATTERN_UNIFORM, 15, 44.4, 25, 750)));
        /* even beam count */
        assert(!parse(s, scan_packet(SCAN_PATTERN_UNIFORM, 14, 44.4, 40, 750)));
        /* stepping faster than validated */
        assert(!parse(s, scan_packet(SCAN_PATTERN_UNIFORM, 15, 44.4, 40, 100)));
        /* NaN steering limit */
        assert(!parse(s, scan_packet(SCAN_PATTERN_UNIFORM, 15, NAN, 40, 750)));
        assert(s.isValid());
        assert(s.getScanConfig().azimuth_positions == 40);
        assert(s.getScanConfig().step_half_period_us == 750);
    }
    std::printf("PASS\n");

    std::printf("  Test 4: NaN in a legacy field rejected... ");
    {
        RadarSettings s;
        assert(!parse(s, legacy_packet(NAN)));
        assert(s.isValid());                       /* defaults retained */
        assert(s.getSystemFrequency() == 10.0e9);
    }
    std::printf("PASS\n");

    std::printf("  Test 5: bad framing rejected... ");
    {
        RadarSettings s;
        std::vector<uint8_t> p = scan_packet(SCAN_PATTERN_UNIFORM, 15, 44.4, 40, 750);
        p[79] = 'X';                               /* corrupt "SCN" */
        assert(!parse(s, p));
        std::vector<uint8_t> q = legacy_packet();
        q.insert(q.end() - 3, 0x00);               /* 83 bytes: unknown layout */
        assert(!parse(s, q));
        assert(!s.parseFromUSB(nullptr, 82));
        assert(!s.hasScanConfig());
    }
    std::printf("PASS\n");

    std::printf("=== ALL RadarSettings scan TESTS PASSED ===\n");
    return 0;
}
