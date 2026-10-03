/*
 * scan_config.c -- Validated, host-configurable radar scan plan.
 * See scan_config.h for the contract.
 */
#include "scan_config.h"

#include <math.h>
#include <stddef.h>

#define SCAN_PI 3.14159265358979f

/* Original per-element phase increments (degrees), positions 1..31.
 * Index 15 is broadside.  Kept verbatim from main.cpp so the LEGACY
 * pattern reproduces the original beam sequence exactly. */
static const float legacy_phase_differences[31] = {
    160.0f, 80.0f, 53.333f, 40.0f, 32.0f, 26.667f, 22.857f, 20.0f, 17.778f, 16.0f,
    14.545f, 13.333f, 12.308f, 11.429f, 10.667f, 0.0f,
    -10.667f, -11.429f, -12.308f, -13.333f, -14.545f, -16.0f, -17.778f, -20.0f,
    -22.857f, -26.667f, -32.0f, -40.0f, -53.333f, -80.0f, -160.0f
};

void scan_config_default(scan_config_t *cfg)
{
    if (cfg == NULL) return;
    cfg->pattern               = SCAN_PATTERN_LEGACY;
    cfg->uniform_beams         = 15;
    cfg->uniform_max_angle_deg = 44.4f;
    cfg->azimuth_positions     = 50;
    cfg->step_half_period_us   = SCAN_MIN_STEP_HALF_US;
}

float scan_phase_step_to_angle_deg(float phase_step_deg)
{
    /* Half-wave spacing: sin(theta) = delta_phi / 180 deg */
    float u = phase_step_deg / 180.0f;
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    return asinf(u) * 180.0f / SCAN_PI;
}

static float uniform_spacing_u(const scan_config_t *cfg)
{
    float u_max = sinf(cfg->uniform_max_angle_deg * SCAN_PI / 180.0f);
    return 2.0f * u_max / (float)(cfg->uniform_beams - 1);
}

scan_status_t scan_config_validate(const scan_config_t *cfg)
{
    if (cfg == NULL) return SCAN_ERR_NULL;

    if (cfg->pattern == SCAN_PATTERN_UNIFORM) {
        if (cfg->uniform_beams < SCAN_MIN_UNIFORM_BEAMS ||
            cfg->uniform_beams > SCAN_MAX_UNIFORM_BEAMS ||
            (cfg->uniform_beams % 2) == 0)        /* odd: includes broadside */
            return SCAN_ERR_BEAM_COUNT;
        /* !(x > 0) also rejects NaN */
        if (!(cfg->uniform_max_angle_deg > 0.0f) ||
            cfg->uniform_max_angle_deg > SCAN_MAX_STEER_DEG)
            return SCAN_ERR_MAX_ANGLE;
        if (uniform_spacing_u(cfg) > SCAN_ELEV_BEAMWIDTH_U)
            return SCAN_ERR_ELEV_GAP;
    } else if (cfg->pattern != SCAN_PATTERN_LEGACY) {
        return SCAN_ERR_PATTERN;
    }

    if (cfg->azimuth_positions == 0 ||
        (SCAN_STEPPER_STEPS_PER_REV % cfg->azimuth_positions) != 0)
        return SCAN_ERR_AZ_DIVISOR;
    if (360.0f / (float)cfg->azimuth_positions > SCAN_AZ_BEAMWIDTH_DEG)
        return SCAN_ERR_AZ_GAP;

    if (cfg->step_half_period_us < SCAN_MIN_STEP_HALF_US ||
        cfg->step_half_period_us > SCAN_MAX_STEP_HALF_US)
        return SCAN_ERR_STEP_SPEED;

    return SCAN_OK;
}

uint8_t scan_build_dwells(const scan_config_t *cfg, scan_dwell_t *out, uint8_t max)
{
    uint8_t n = 0;
    uint8_t k;

    if (out == NULL || scan_config_validate(cfg) != SCAN_OK) return 0;

    if (cfg->pattern == SCAN_PATTERN_LEGACY) {
        if (max < 45) return 0;
        /* Original order: per position, +theta (elevation toggle), broadside, -theta */
        for (k = 0; k < 15; k++) {
            out[n].phase_step_deg = legacy_phase_differences[k];
            out[n].toggle_elevation = 1;
            n++;
            out[n].phase_step_deg = 0.0f;
            out[n].toggle_elevation = 0;
            n++;
            out[n].phase_step_deg = legacy_phase_differences[k + 16];
            out[n].toggle_elevation = 0;
            n++;
        }
        return n;
    }

    /* UNIFORM: monotonic sweep from +max to -max, one elevation per dwell */
    if (max < cfg->uniform_beams) return 0;
    {
        float u_max = sinf(cfg->uniform_max_angle_deg * SCAN_PI / 180.0f);
        float du = uniform_spacing_u(cfg);
        uint8_t mid = cfg->uniform_beams / 2;
        for (k = 0; k < cfg->uniform_beams; k++) {
            float u = (k == mid) ? 0.0f : (u_max - du * (float)k);
            out[n].phase_step_deg = 180.0f * u;
            out[n].toggle_elevation = 1;
            n++;
        }
    }
    return n;
}

uint16_t scan_steps_per_position(const scan_config_t *cfg)
{
    if (scan_config_validate(cfg) != SCAN_OK) return 0;
    return (uint16_t)(SCAN_STEPPER_STEPS_PER_REV / cfg->azimuth_positions);
}

uint32_t scan_revolution_time_us(const scan_config_t *cfg)
{
    scan_dwell_t dwells[SCAN_MAX_DWELLS];
    uint8_t n = scan_build_dwells(cfg, dwells, SCAN_MAX_DWELLS);
    float dwell_us, per_position_us;

    if (n == 0) return 0;
    dwell_us = SCAN_CHIRPS_PER_HALF * SCAN_PRI1_US + SCAN_GUARD_US +
               SCAN_CHIRPS_PER_HALF * SCAN_PRI2_US;
    per_position_us = (float)n * dwell_us +
                      (float)scan_steps_per_position(cfg) * 2.0f *
                      (float)cfg->step_half_period_us;
    return (uint32_t)(per_position_us * (float)cfg->azimuth_positions + 0.5f);
}

float scan_legacy_max_gap_deg(void)
{
    /* Collect beam centres in u, sort, then walk the coverage over
     * [-sin45, +sin45] using +/- half-beamwidth around each centre. */
    float u[31];
    float half = 0.5f * SCAN_ELEV_BEAMWIDTH_U;
    float req = sinf(SCAN_MAX_STEER_DEG * SCAN_PI / 180.0f);
    float covered_to = -req;
    float worst = 0.0f;
    int i, j;

    for (i = 0; i < 31; i++) u[i] = legacy_phase_differences[i] / 180.0f;
    for (i = 1; i < 31; i++) {          /* insertion sort, ascending */
        float v = u[i];
        for (j = i - 1; j >= 0 && u[j] > v; j--) u[j + 1] = u[j];
        u[j + 1] = v;
    }

    for (i = 0; i < 31 && covered_to < req; i++) {
        float lo = u[i] - half, hi = u[i] + half;
        if (lo > covered_to) {
            float gap_end = (lo < req) ? lo : req;
            float g = (asinf(gap_end) - asinf(covered_to)) * 180.0f / SCAN_PI;
            if (g > worst) worst = g;
        }
        if (hi > covered_to) covered_to = hi;
    }
    if (covered_to < req) {
        float g = (asinf(req) - asinf(covered_to)) * 180.0f / SCAN_PI;
        if (g > worst) worst = g;
    }
    return worst;
}
