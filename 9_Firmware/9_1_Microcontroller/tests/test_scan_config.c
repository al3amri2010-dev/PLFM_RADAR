/*******************************************************************************
 * test_scan_config.c
 *
 * Unit tests for scan_config.c (validated, host-configurable scan plan).
 *
 * Covers:
 *   - Legacy default reproduces the original beam sequence exactly
 *   - Legacy elevation coverage gap is detected and reported
 *   - Uniform pattern: count, ordering, symmetry, coverage guarantee
 *   - Revolution time estimate (legacy vs uniform)
 *   - Every validation rejection path (fail-safe on bad host input)
 ******************************************************************************/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "scan_config.h"

static int near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static const float original_table[31] = {
    160.0f, 80.0f, 53.333f, 40.0f, 32.0f, 26.667f, 22.857f, 20.0f, 17.778f, 16.0f,
    14.545f, 13.333f, 12.308f, 11.429f, 10.667f, 0.0f,
    -10.667f, -11.429f, -12.308f, -13.333f, -14.545f, -16.0f, -17.778f, -20.0f,
    -22.857f, -26.667f, -32.0f, -40.0f, -53.333f, -80.0f, -160.0f
};

static void test_legacy_default(void)
{
    scan_config_t c;
    scan_dwell_t d[SCAN_MAX_DWELLS];
    uint8_t n, k;

    printf("  Test 1: legacy default reproduces original sequence... ");
    scan_config_default(&c);
    assert(c.pattern == SCAN_PATTERN_LEGACY);
    assert(scan_config_validate(&c) == SCAN_OK);
    n = scan_build_dwells(&c, d, SCAN_MAX_DWELLS);
    assert(n == 45);
    for (k = 0; k < 15; k++) {
        assert(d[3 * k].phase_step_deg == original_table[k]);
        assert(d[3 * k].toggle_elevation == 1);
        assert(d[3 * k + 1].phase_step_deg == 0.0f);
        assert(d[3 * k + 1].toggle_elevation == 0);
        assert(d[3 * k + 2].phase_step_deg == original_table[k + 16]);
        assert(d[3 * k + 2].toggle_elevation == 0);
    }
    assert(scan_steps_per_position(&c) == 4);
    printf("PASS\n");
}

static void test_legacy_gap(void)
{
    float gap;
    printf("  Test 2: legacy elevation gap detected... ");
    gap = scan_legacy_max_gap_deg();
    /* Beams at 26.4 deg then 62.7 deg: ~30..45 deg uncovered */
    assert(gap > 10.0f && gap < 20.0f);
    printf("PASS (gap %.1f deg)\n", (double)gap);
}

static void test_uniform(void)
{
    scan_config_t c;
    scan_dwell_t d[SCAN_MAX_DWELLS];
    uint8_t n, k;

    printf("  Test 3: uniform 15-beam pattern... ");
    scan_config_default(&c);
    c.pattern = SCAN_PATTERN_UNIFORM;
    assert(scan_config_validate(&c) == SCAN_OK);
    n = scan_build_dwells(&c, d, SCAN_MAX_DWELLS);
    assert(n == 15);
    assert(d[7].phase_step_deg == 0.0f);                       /* exact broadside */
    for (k = 0; k < n; k++) {
        float a = scan_phase_step_to_angle_deg(d[k].phase_step_deg);
        assert(d[k].toggle_elevation == 1);
        assert(fabsf(a) <= SCAN_MAX_STEER_DEG + 0.01f);
        assert(near(d[k].phase_step_deg, -d[n - 1 - k].phase_step_deg, 1e-3f));
        if (k > 0) {
            float du = (d[k - 1].phase_step_deg - d[k].phase_step_deg) / 180.0f;
            assert(du > 0.0f);                                  /* monotonic sweep */
            assert(du <= SCAN_ELEV_BEAMWIDTH_U + 1e-6f);        /* no gap */
        }
    }
    assert(near(scan_phase_step_to_angle_deg(d[0].phase_step_deg), 44.4f, 0.05f));
    printf("PASS\n");
}

static void test_revolution_time(void)
{
    scan_config_t c;
    uint32_t legacy_us, uniform_us;

    printf("  Test 4: revolution time estimate... ");
    scan_config_default(&c);
    legacy_us = scan_revolution_time_us(&c);
    /* 50 x (45 x 5647.4 + 4 x 1000) */
    assert(legacy_us > 12900000u && legacy_us < 12915000u);
    c.pattern = SCAN_PATTERN_UNIFORM;
    uniform_us = scan_revolution_time_us(&c);
    /* 50 x (15 x 5647.4 + 4 x 1000) */
    assert(uniform_us > 4430000u && uniform_us < 4440000u);
    printf("PASS (legacy %.2f s, uniform %.2f s)\n",
           legacy_us / 1e6, uniform_us / 1e6);
}

static void test_rejections(void)
{
    scan_config_t c;
    scan_dwell_t d[SCAN_MAX_DWELLS];

    printf("  Test 5: invalid configurations rejected... ");
    assert(scan_config_validate(NULL) == SCAN_ERR_NULL);

    scan_config_default(&c); c.pattern = 7;
    assert(scan_config_validate(&c) == SCAN_ERR_PATTERN);

    scan_config_default(&c); c.pattern = SCAN_PATTERN_UNIFORM; c.uniform_beams = 14;
    assert(scan_config_validate(&c) == SCAN_ERR_BEAM_COUNT);
    c.uniform_beams = 33;
    assert(scan_config_validate(&c) == SCAN_ERR_BEAM_COUNT);
    c.uniform_beams = 1;
    assert(scan_config_validate(&c) == SCAN_ERR_BEAM_COUNT);

    scan_config_default(&c); c.pattern = SCAN_PATTERN_UNIFORM;
    c.uniform_max_angle_deg = 0.0f;
    assert(scan_config_validate(&c) == SCAN_ERR_MAX_ANGLE);
    c.uniform_max_angle_deg = 45.5f;
    assert(scan_config_validate(&c) == SCAN_ERR_MAX_ANGLE);
    c.uniform_max_angle_deg = NAN;
    assert(scan_config_validate(&c) == SCAN_ERR_MAX_ANGLE);

    /* 3 beams over +/-44.4 deg: spacing 0.7 in u -> coverage gap */
    scan_config_default(&c); c.pattern = SCAN_PATTERN_UNIFORM; c.uniform_beams = 3;
    assert(scan_config_validate(&c) == SCAN_ERR_ELEV_GAP);
    /* 13 beams over +/-45: spacing 0.118 > 0.1108 -> gap */
    c.uniform_beams = 13; c.uniform_max_angle_deg = 45.0f;
    assert(scan_config_validate(&c) == SCAN_ERR_ELEV_GAP);

    scan_config_default(&c); c.azimuth_positions = 0;
    assert(scan_config_validate(&c) == SCAN_ERR_AZ_DIVISOR);
    c.azimuth_positions = 30;                       /* 200 % 30 != 0 */
    assert(scan_config_validate(&c) == SCAN_ERR_AZ_DIVISOR);
    c.azimuth_positions = 25;                       /* 14.4 deg > 12.69 deg */
    assert(scan_config_validate(&c) == SCAN_ERR_AZ_GAP);
    c.azimuth_positions = 40;
    assert(scan_config_validate(&c) == SCAN_OK);
    c.azimuth_positions = 200;
    assert(scan_config_validate(&c) == SCAN_OK);

    scan_config_default(&c); c.step_half_period_us = SCAN_MIN_STEP_HALF_US - 1;
    assert(scan_config_validate(&c) == SCAN_ERR_STEP_SPEED);
    c.step_half_period_us = SCAN_MAX_STEP_HALF_US + 1;
    assert(scan_config_validate(&c) == SCAN_ERR_STEP_SPEED);

    /* Invalid config yields no dwells, no steps, no time */
    scan_config_default(&c); c.azimuth_positions = 25;
    assert(scan_build_dwells(&c, d, SCAN_MAX_DWELLS) == 0);
    assert(scan_steps_per_position(&c) == 0);
    assert(scan_revolution_time_us(&c) == 0);

    /* Output buffer too small */
    scan_config_default(&c);
    assert(scan_build_dwells(&c, d, 44) == 0);
    assert(scan_build_dwells(&c, NULL, SCAN_MAX_DWELLS) == 0);
    printf("PASS\n");
}

int main(void)
{
    printf("=== scan_config: validated scan plan ===\n");
    test_legacy_default();
    test_legacy_gap();
    test_uniform();
    test_revolution_time();
    test_rejections();
    printf("=== ALL scan_config TESTS PASSED ===\n");
    return 0;
}
