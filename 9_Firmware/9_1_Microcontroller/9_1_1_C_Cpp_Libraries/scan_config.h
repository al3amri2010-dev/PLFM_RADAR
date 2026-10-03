/*
 * scan_config.h -- Validated, host-configurable radar scan plan.
 *
 * Pure C (no HAL dependencies) so it can be unit-tested on the host.
 *
 * A scan revolution is: for each azimuth position, visit every elevation
 * "dwell" in the dwell list, transmitting one 16-long + 16-short chirp burst
 * per dwell, then step the motor to the next azimuth.
 *
 * Fixed by the FPGA contract (NOT configurable here):
 *   - 16 long + 16 short chirps per dwell (doppler_processor: 2 x 16 sub-frames)
 *   - chirp durations / PRIs (plfm_chirp_controller.v)
 *
 * Every configuration accepted by scan_config_validate() is guaranteed to
 * leave no elevation or azimuth coverage gap wider than one 3 dB beamwidth.
 * The LEGACY pattern is kept bit-for-bit as the default for backward
 * compatibility, but it does NOT meet that guarantee (see
 * scan_legacy_max_gap_deg()).
 */
#ifndef SCAN_CONFIG_H
#define SCAN_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Array geometry: 16-element elevation column, 8-element azimuth row,
 * half-wavelength spacing.  3 dB beamwidth in u = sin(theta) is
 * 0.886 * lambda / (N * d) = 0.886 * 2 / N. */
#define SCAN_ELEV_ELEMENTS          16
#define SCAN_ELEV_BEAMWIDTH_U       (0.886f * 2.0f / SCAN_ELEV_ELEMENTS)   /* 0.1108 */
#define SCAN_AZ_BEAMWIDTH_DEG       12.69f                                  /* 8 elements */
#define SCAN_MAX_STEER_DEG          45.0f   /* element pattern limit */

#define SCAN_STEPPER_STEPS_PER_REV  200
#define SCAN_MIN_STEP_HALF_US       500     /* validated default; never faster */
#define SCAN_MAX_STEP_HALF_US       5000

#define SCAN_MAX_DWELLS             45      /* legacy pattern size */
#define SCAN_MIN_UNIFORM_BEAMS      3
#define SCAN_MAX_UNIFORM_BEAMS      31

/* Chirp burst timing per dwell (must match main.cpp / FPGA) */
#define SCAN_CHIRPS_PER_HALF        16
#define SCAN_PRI1_US                167.0f
#define SCAN_PRI2_US                175.0f
#define SCAN_GUARD_US               175.4f

typedef enum {
    SCAN_PATTERN_LEGACY  = 0,   /* original 15 x {+theta, broadside, -theta} */
    SCAN_PATTERN_UNIFORM = 1    /* evenly spaced in sin(theta), no repeats   */
} scan_pattern_t;

typedef struct {
    uint8_t  pattern;               /* scan_pattern_t */
    uint8_t  uniform_beams;         /* UNIFORM only: odd, 3..31 */
    float    uniform_max_angle_deg; /* UNIFORM only: (0, 45] */
    uint8_t  azimuth_positions;     /* divides 200, step <= azimuth beamwidth */
    uint16_t step_half_period_us;   /* stepper pulse half period */
} scan_config_t;

typedef enum {
    SCAN_OK = 0,
    SCAN_ERR_NULL,
    SCAN_ERR_PATTERN,
    SCAN_ERR_BEAM_COUNT,
    SCAN_ERR_MAX_ANGLE,
    SCAN_ERR_ELEV_GAP,          /* beam spacing wider than one beamwidth */
    SCAN_ERR_AZ_DIVISOR,        /* positions do not divide steps/rev */
    SCAN_ERR_AZ_GAP,            /* azimuth step wider than one beamwidth */
    SCAN_ERR_STEP_SPEED
} scan_status_t;

/* One elevation dwell */
typedef struct {
    float   phase_step_deg;     /* per-element phase increment */
    uint8_t toggle_elevation;   /* 1 = signal FPGA elevation change first */
} scan_dwell_t;

/* Legacy defaults: identical behaviour to the original firmware. */
void scan_config_default(scan_config_t *cfg);

scan_status_t scan_config_validate(const scan_config_t *cfg);

/* Fills out[] (capacity max) and returns the dwell count, or 0 if the
 * configuration is invalid or does not fit. */
uint8_t scan_build_dwells(const scan_config_t *cfg, scan_dwell_t *out, uint8_t max);

/* Steering angle for a per-element phase increment (half-wave spacing). */
float scan_phase_step_to_angle_deg(float phase_step_deg);

/* Motor steps between azimuth positions (0 if invalid). */
uint16_t scan_steps_per_position(const scan_config_t *cfg);

/* Lower bound on one revolution, in microseconds (chirp bursts + motor
 * stepping; excludes SPI beam-update and logging overhead). 0 if invalid. */
uint32_t scan_revolution_time_us(const scan_config_t *cfg);

/* Widest uncovered elevation gap of the legacy pattern, in degrees, measured
 * between the 3 dB edges of adjacent beams over the ±45 degree requirement. */
float scan_legacy_max_gap_deg(void);

#ifdef __cplusplus
}
#endif

#endif /* SCAN_CONFIG_H */
