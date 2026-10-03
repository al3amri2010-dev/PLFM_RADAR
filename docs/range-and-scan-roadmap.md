# AERIS-10 Range & Scan Roadmap

Status: **simulation-verified only.** Nothing in this document has been
validated on hardware. Every figure marked *measured* comes from the
repository's own simulation models (Icarus Verilog / host unit tests); figures
marked *estimate* are first-order calculations.

---

## 1. Findings that block the 3 km / 20 km targets

### 1.1 Matched filter cannot keep up with the chirp rate (*measured*)

`matched_filter_multi_segment.v` + `matched_filter_processing_chain.v`
(synthesis branch, `fft_engine.v`) takes:

| Chirp | Work | First output | Last output |
|---|---|---|---|
| Long (30 µs) | 4 × 1024-pt blocks | 0.71 ms | **2.87 ms** |
| Short (0.5 µs) | 1 × 1024-pt block | ~0.7 ms | ~0.7 ms |

The long-chirp PRI is **167 µs**, so roughly **1 long chirp in 17** is
processed; chirp-start events that arrive while the filter is busy are
ignored. This is the root blocker: no amount of range-window re-wiring helps
until pulse compression is real-time.

### 1.2 Receive window covers the transmit pulse, not the echoes

The filter starts capturing on chirp start and stops after the chirp length
(3000 samples long / 50 samples short). That window is the time the
T/R switches are in **transmit**. The 137–175 µs listen period, where echoes
arrive, is never processed.

### 1.3 Long-chirp correlation is only valid near zero delay

Each 1024-sample received block is correlated against the *matching* 1024-sample
segment of the 3000-sample reference. That is only a matched filter for echoes
whose delay is small compared with a segment; it is not a partitioned
(full-reference) correlation.

### 1.4 Frame contract is inconsistent

* TX (`plfm_chirp_controller.v`), STM32 (`executeChirpSequence`) and the Doppler
  processor all use **16 long, guard, 16 short** per burst.
* RX auto-scan (`radar_mode_controller.v`, mode 01) instead runs
  **long → listen → guard → short → listen per chirp**, and only flags the
  long chirp to the matched filter.
* Each long chirp emits 4 × 1024 samples, so the 1024→64 decimator produces
  **4 Doppler rows per chirp**; Doppler "chirps" are actually segments.

### 1.5 Elevation coverage gap and redundant dwells (*measured*, `scan_config`)

The original phase table (`phase_differences[]`) steers to
0°, ±3.4°…±12.8° (13 beams), ±17.2°, ±26.4°, **±62.7°**.
With a 16-element 3 dB beamwidth of ~6.3°, **~30°–45° is not covered**
(`scan_legacy_max_gap_deg()` = 15.0°), and the broadside beam is re-visited 15
times per azimuth (45 dwells where 15 suffice).

### 1.6 Long chirp eclipse is partial, not total (*estimate*)

A target at delay *d* < 30 µs returns an echo whose last *d* µs arrive after
TX ends. Visible energy fraction = *d* / 30 µs:

| Range | Visible | SNR loss vs. full chirp |
|---|---|---|
| 750 m | 17 % | −7.8 dB |
| 1.5 km | 33 % | −4.8 dB |
| 3 km | 67 % | −1.8 dB |
| ≥ 4.5 km | 100 % | 0 dB |

The 0.5 µs short chirp is **−17.8 dB** relative to a full long chirp, so the
partially eclipsed long chirp is the better waveform beyond ~750 m.

### 1.7 Detection range is bounded by physics (*estimate*)

Radar equation, 5 dB NF, 8 dB losses, 13 dB required SNR, 30 µs × 20 MHz pulse
compression, 16-pulse coherent integration:

| Target RCS | Nexus 16 × 1 W, 25.8 dBi | Extended 16 × 10 W, ~30 dBi |
|---|---|---|
| 0.01 m² (small drone) | ~1.3 km | ~3.8 km |
| 0.1 m² (large drone) | ~2.3 km | ~6.7 km |
| 1 m² (small aircraft) | ~4.1 km | ~12 km |

20 km is only reachable for aircraft-size targets on the Extended variant.

---

## 2. Done in this change (phase 0: contract fixes)

| Area | Change | Verification |
|---|---|---|
| FPGA | Matched filter reacts to **every** toggle of `mc_new_chirp` (was rising edge only → every 2nd chirp dropped) | `tb_mf_chirp_trigger.v` (new, in regression) |
| FPGA | Short chirp now terminates (`chirp_complete`); previously the filter looped in `ST_COLLECT_DATA` forever | `tb_mf_chirp_trigger.v` |
| FPGA | STM32 mode labels chirps long/short by burst position (was always long) | `tb_radar_mode_controller.v` group 15b |
| MCU | `scan_config.{h,c}`: validated scan plan; legacy default reproduces the original sequence bit-for-bit | `test_scan_config.c` |
| MCU | New **UNIFORM** pattern: 15 beams evenly spaced in sin θ over ±44.4°, no coverage gap | `test_scan_config.c` |
| MCU | Settings packet accepts optional `SCN` block (98 B); parse is all-or-nothing; NaN fields now rejected | `test_radar_settings_scan.cpp`, cross-layer Tier 3 |
| MCU | Scan plan switched only at a revolution boundary, from an interrupt-safe snapshot | code review (needs hardware) |
| MCU | Removed `DIAG` printf from `pulseTXMode/pulseRXMode` (µs timing path) | code review |

Revolution time (lower bound, chirp bursts + stepping):
**legacy 12.91 s → uniform 4.44 s** (*measured* by `scan_revolution_time_us`).
The README's 8.3 s figure does not match the firmware's 45 dwells/azimuth.

Validation rules (rejected → previous plan stays active):

* UNIFORM beams odd, 3–31; steering limit (0°, 45°]; beam spacing ≤ one 3 dB
  beamwidth (0.1108 in sin θ).
* Azimuth positions divide 200 motor steps and step ≤ 12.69° azimuth
  beamwidth → allowed 40, 50, 100, 200.
* Stepper half-period 500–5000 µs (never faster than the original).
* Chirps per dwell fixed at 16 + 16 (Doppler contract).

**Build note:** `scan_config.c` must be added to the STM32CubeIDE project (the
IDE project is not tracked in the repository).

**Default is unchanged (LEGACY).** Switch to UNIFORM only after bench
validation of the new beam table.

---

## 3. Phase 1 — real-time 3 km receive chain (Nexus)

Goal: every chirp produces exactly **one** range profile covering 0–3 km, in
real time.

1. **Decimate before pulse compression.** The DDC outputs 100 MSPS complex for a
   20 MHz chirp; 25 MSPS complex is sufficient. ÷4 cuts every FFT by 4×.
2. **Single-block FFT correlation with a precomputed reference spectrum.**
   At 25 MSPS: 3 km echo window (500 samples) + 30 µs chirp (750 samples)
   → 2048-pt FFT, valid delays 0–1298 samples (7.8 km). Reference FFT stored in
   ROM (constant) → only forward + inverse FFT per chirp.
3. **Streaming FFT engine** (Xilinx `xfft` pipelined-streaming or an open R2²SDF
   core): 2048 points at 100 MHz ≈ 20 µs per transform → ~45 µs per chirp,
   comfortably inside the 167 µs PRI.
4. **Receive window = listen period**, blanking samples while TX is active
   (leakage) and starting the delay axis at TX start.
5. **Frame contract:** RX auto-scan mirrors TX (16 long, guard, 16 short); one
   row per chirp; rewrite `tb_radar_mode_controller` groups 3/4/14.
6. **Range bins:** 6 m native → 128 bins × 24 m for 3 km. Requires Doppler,
   MTI, CFAR, USB packet and GUI updates (`RANGE_BINS` 64 → 128), or keep
   64 × 48 m as an interim.
7. **Near range:** short sub-frame covers ~75–750 m, long sub-frame
   ~750 m–3 km; CFAR per sub-frame then merge.
8. Wire the existing but unused `host_range_mode` register (opcode 0x20).

Verification: bit-exact Python golden model of the new chain, co-simulation
with `tb/cosim` targets at 100 m / 1 km / 2.9 km (inside and beyond eclipse),
throughput assertion (no dropped chirp over a full frame), Vivado
utilisation + timing closure on XC7A50T (cannot be done in this environment).

## 4. Phase 2 — configurable 3 km / 20 km

* `host_range_mode`: 0 = 1.5 km legacy, 1 = 3 km, 2 = 20 km (Extended only).
* 20 km at 25 MSPS: 3333 + 750 samples → 8192-pt FFT (or 2 × 4096 partitioned);
  PRI 167 µs gives 25 km unambiguous range (OK).
* BRAM budget for 8192-pt + 32-chirp corner turn must be checked on 50T; may
  require the XC7A200T board.
* Medium chirp option (e.g. 5 µs, 750 m eclipse, +10 dB over short chirp) for
  better near-range sensitivity.
* MCU: per-mode PA enable / power sequencing for the 16 × QPA2962 boards.
* GUI V7: range-mode selector; range axis from `WaveformConfig`.

## 5. Scan-time roadmap

* **GUI V7 → MCU settings channel.** V7 only *receives* GPS from the STM32;
  nothing sends the `SET`/`SCN` packet today (V5/V6 did). Add a sender + UI.
* **Timer-driven stepper** (continuous rotation) instead of blocking 1 ms steps.
* **Align FPGA `ELEVATIONS_PER_AZIMUTH` (31)** with the active dwell count (status
  counters only today).
* **Settings integrity:** add CRC to the `SET` packet; `USBHandler` takes the
  first `END` byte sequence, which payload bytes can mimic.

## 6. Gaps for critical / safety-relevant use

These are outside what simulation can close and must be addressed before any
critical deployment:

1. **No hardware validation** of any change; bring-up on real boards first.
2. **No timing closure / resource report** for the FPGA changes here.
3. **No azimuth position feedback** — missed stepper steps go undetected. Add an
   encoder or a north index sensor and check it every revolution.
4. **Matched filter throughput** (§1.1) — until Phase 1 lands, range/Doppler
   output is not physically meaningful.
5. **Built-in test** does not cover the RF chain end-to-end (loop-back target).
6. **Documentation drift:** README scan time, ±45° coverage, and FPGA part
   (50T vs 200T) disagree with the code.
7. **Regulatory / certification:** spectrum licence at 10.5 GHz, EMC, and any
   functional-safety standard (e.g. IEC 61508) are out of scope of this repo.
