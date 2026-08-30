// PROJECT R.E.A.P. — ESP32 Firmware
// Version: 0.5-layer2-adaptive (Session 32, 2026-08-29 — Layer 2 implemented + relay live)
//
// v0.5 changes (over v0.4):
// - LAYER 2 ADAPTIVE THRESHOLD IMPLEMENTED. It was previously DEAD CODE: baselineWatts[]
//   was declared and never assigned, so its guard was permanently false and no adaptive
//   detection existed. The backend had no threshold endpoints either. Both now exist.
//   Training learns max wattage over a window, threshold = max x 1.20, persisted to the
//   backend AND on-device NVS. Boot order: backend -> NVS -> Layer 1 alone (Q3.5).
//   Every running-max sample is logged for convergence analysis (Q3.4).
// - ENERGY UNITS BUG FIXED: the accumulator computed W*h and stored it in a field named
//   kWh. Every energy/cost/carbon value was 1000x too large since the project began.
//   Historical data is recoverable by dividing by 1000 - see knowledge/data-export.md.
// - RELAY_POLL_INTERVAL_MS 30000 -> 2000. Measured command-to-action latency was 19 s
//   against Q4.2 target of <2 s; now ~1-2 s, verified.
// - LAYER1_POWER_MAX restored 2500 -> 1500 after calibration.
// - THRESHOLD_REFRESH_MS: thresholds re-fetched every 60 s so a changed threshold takes
//   effect without a reboot (30 detection trials would otherwise need 30 reboots).
// - CHANNEL_INSTRUMENTED[]: only ch1 has a sensor. Channels 2-3 never train or arm.
//
// --- v0.4 changelog retained below ---
// Version: 0.4-phase1-calibrated (Session 31, 2026-08-28 — first REAL calibrated data)
//
// v0.4 changes (over v0.3):
// - CALIBRATED: SCT_CAL 30.0 -> 7.75 (current, 1.30% MAPE vs watt reader)
//               ZMPT_CAL 0.6345 (voltage, vs electrician's DMM)
//   Both sensors on CHANNEL 1 ONLY. Channels 2-3 have no sensors fitted yet.
// - EmonLib ADC_BITS bug: library compiled with AVR default 10 bits (ADC_COUNTS=1024)
//   while ESP32 analogRead() returns 0-4095 => every current read 4x high.
// - EMON_SETTLE_PASSES + LAYER1_WARMUP_CYCLES: fixes every boot instantly tripping all
//   three relays on EmonLib's unconverged first readings.
// - NOISE_FLOOR_A[] quadrature noise-floor subtraction. Sets limit of detection
//   (ch1 = 0.30 A ~ 66 W). MUST be disclosed in the paper's methodology.
// - sampleVoltageRaw(): direct-ADC Vrms via the variance identity, median of 3 windows
//   to reject windows corrupted by WiFi/HTTP preempting the sampling loop.
// - ZMPT_WIRED[] / VOLTAGE_REF_CH: unwired channels inherit ch1's voltage (single-phase
//   supply). Prevents floating pins reporting hundreds of volts and tripping Layer 1.
// - [READ] / [VOLT] / [ADC] serial diagnostics — GATE THESE before unattended runs.
// - ⚠ LAYER1_POWER_MAX temporarily 1500 -> 2500 for calibration. MUST BE RESTORED.
// - ⚠ LAYER1_VOLTAGE_MAX 240 V is only ~4% above this house's 225-230 V idle line.
//   Adviser decision pending — see state/decisions.md #60.
//
// --- v0.3 changelog retained below ---
// Version: 0.3-phase1-real-sensors (Session 28 prep — first hardware arrival)
// Description: IoT-based energy monitoring and safety automation system
// Monitors 3 appliances, detects wattage anomalies, triggers relay shutdown
//
// v0.3 changes (over v0.2-phone-backend):
// - readCurrent() now uses real EmonLib calcIrms() when CURRENT_SENSOR_CONNECTED=true.
//   Returns 0.0 when sensor not yet wired (safe default).
// - readVoltage() still returns 220.0 placeholder; flips to ZMPT read when
//   VOLTAGE_SENSOR_CONNECTED=true (ZMPT physical wiring still pending).
// - PIN_RELAY_CH3 reassigned 14 -> 16 (avoids ESP32 strapping pin that can fluctuate at boot).
// - Layer 1 Hardcoded Safety Floor added (per paper/rebrand-plan.md §5.1):
//     OVER_VOLTAGE  (V > 240, only when ZMPT online)
//     OVER_POWER    (P > 1500W, always-on once current sensor online)
//   Layer 1 fires immediately, no debounce, before the existing Layer 2 (20% threshold) check.
// - emon[ch].current() initial calibration constant set to 30.0 (SCT-013-030 starting value).
//   Refine against the plug-in watt reader during Phase 1 calibration.

// ============================================================
// LIBRARIES
// ============================================================
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "EmonLib.h"
#include "esp_wifi.h" // required for esp_wifi_set_ps()
#include "esp_task_wdt.h" // Session 21: hardware watchdog timer
#include <Preferences.h>  // v0.5: NVS storage for the learned Layer 2 threshold

// ============================================================
// CONFIGURATION — update these before flashing
// ============================================================

// WiFi
const char* SSID     = "<network SSID redacted>";
const char* PASSWORD = "<password redacted>";

// Backend
const char* BACKEND_URL_READINGS      = "http://192.168.1.58:3000/api/readings";
const char* BACKEND_URL_ANOMALIES     = "http://192.168.1.58:3000/api/anomalies";
const char* BACKEND_URL_RELAY_PENDING = "http://192.168.1.58:3000/api/relay/pending";
const char* BACKEND_URL_RELAY_ACK     = "http://192.168.1.58:3000/api/relay/ack";
// v0.5 (Session 32): Layer 2 adaptive threshold persistence endpoints.
const char* BACKEND_URL_THRESHOLD       = "http://192.168.1.58:3000/api/threshold";
const char* BACKEND_URL_THRESHOLD_TRAIN = "http://192.168.1.58:3000/api/threshold/training";

// API key — must match API_KEY env var on phone backend
const char* API_KEY = "<API key redacted>";

// VECO rate
const float VECO_RATE = 11.51; // ₱/kWh — update each billing period

// Carbon emission factor — Philippine grid
// FLAG: verify against latest DOE/DOE-ERC figures before final paper
const float CARBON_FACTOR = 0.6; // kg CO2 per kWh (approximate PH grid average)

// Anomaly threshold (Layer 2 — Per-Appliance Adaptive Threshold; will be replaced by
// learned-max threshold once adaptive logic lands. For now, fixed 20% above manual baseline.)
const float ANOMALY_THRESHOLD_PCT = 0.20;

// Layer 1 — Hardcoded Safety Floor (always-on, never disabled, applied BEFORE Layer 2)
// Per paper/rebrand-plan.md §5.1 — catches catastrophic faults regardless of system state.
// 240.0 -> 250.0 on 2026-08-29 (Session 32) to match the paper, which was corrected the same
// day. The original 240 V predated any measurement of this house; the supply was then measured
// at 210-230 V idle, leaving only ~4% headroom and making nuisance over-voltage trips likely —
// which would have contaminated Q3.1 detection accuracy with false positives.
// 250 V is still inside the ZMPT101B's rated range. See state/decisions.md #60.
const float LAYER1_VOLTAGE_MAX = 250.0; // Over-voltage trip threshold (V)
// RESTORED to 1500.0 on 2026-08-29 (Session 32) ahead of the Q3/Q4 detection trials.
// It had been raised to 2500.0 during calibration on 2026-08-28 so a 1500W load would not
// sit exactly on the ceiling and spam false OVER_POWER anomalies into the DB. Any Q3.1
// detection data recorded at 2500.0 is INVALID and must not be used.
const float LAYER1_POWER_MAX   = 1500.0; // Absolute power ceiling per channel (W)

// Sensor presence flags — flip to true once the corresponding sensor is physically
// wired and calibrated against the plug-in watt reader.
// Until both are true, the system runs in "safe stub" mode: power calculation uses
// V=220 placeholder, current may read 0, and Layer 1 over-voltage is skipped.
const bool VOLTAGE_SENSOR_CONNECTED = true; // ZMPT101B ch1 wired + calibrated 2026-08-28 (Session 31)
const bool CURRENT_SENSOR_CONNECTED = true; // SCT-013-030 — flip after bias circuit wired + calibration

// SCT-013-030 calibration constant for emon.current()
// Starting value 30.0 = scale factor for 30A:1V output. Refine against watt reader
// during Phase 1 calibration: real_current_known / esp32_reported_current = adjustment factor.
// CALIBRATED 2026-08-28 (Session 31, Phase 1) — was 30.0.
// Bench calibration against the plug-in watt reader, 1000W clothes iron on max/dry:
//   watt reader 3.60 A  vs  ESP32 14.26 A (mean of 5 cycles)  ->  ratio 3.96 ≈ 4.00
// Cause: EmonLib scales by (Vsupply/1000)/ADC_COUNTS. Compiled with the AVR default
// ADC_BITS=10 (ADC_COUNTS=1024) while the ESP32's analogRead() returns 0-4095, so every
// current read 4x high. 30.0 / 4 = 7.5. Verify against the watt reader after flashing and
// refine with SCT_CAL_NEW = SCT_CAL * (watt_reader_amps / esp32_amps) if it is still off.
// Refinement pass 2 (same session): with SCT_CAL=7.5 the ESP32 read 3.580 A (mean of 5
// cycles) against the watt reader's 3.70 A — 3.2% low. 7.5 * (3.70/3.580) = 7.75.
// NOTE: the iron's element resistance rises as it heats, so its draw genuinely declines
// during a run (3.610 -> 3.558 across those 5 cycles). Some of that 3.2% is the load
// drifting between the two non-simultaneous readings, not sensor error. Verify once more
// before treating 7.75 as final.
const float SCT_CAL = 7.75;

// v0.4 NOISE-FLOOR COMPENSATION (measured 2026-08-28, Session 31).
// EmonLib returns an RMS, and the RMS of random ADC noise is always positive — it can
// never average to zero. With no load at all the channels still reported a standing
// current, which propagated into power/energy/cost/carbon as phantom watts.
// Measured floors with SCT_CAL=7.75, all loads off:
//   ch1 0.262 A (CT attached)   ch2 0.114 A   ch3 0.131 A (bare biased pins)
// Because noise sums in quadrature, the true current is recoverable:
//   I_true = sqrt(I_measured^2 - I_floor^2)
// This zeroes the idle reading while changing a 3.58 A load by only 0.25%.
// ⚠ PAPER: this is instrument noise-floor compensation, a standard technique, but it
// MUST be disclosed in the methodology alongside the raw measured floors above. Report
// the floor as a stated limit of detection — it is also the direct answer to Q5.1 on
// measurement consistency across power levels. Do not present compensated values as raw.
// Re-measure these floors after any change to wiring, divider resistors, or SCT_CAL.
// Pass 2: floors raised from the noise MEAN to the observed noise MAXIMUM. Subtracting
// the mean left ~0.089 A of residual on ch1 (only the upward excursions survive), which
// integrates into ~20 W of phantom load and ~P16/day of fabricated cost across 3 channels.
// Using the max instead makes idle read a true zero.
//   Raw observed idle ranges: ch1 0.245-0.299   ch2 0.105-0.135   ch3 0.110-0.148
// TRADE-OFF — this sets the LIMIT OF DETECTION, and it must be reported as such:
//   ch1 LOD = 0.30 A (~66 W @ 220V). Loads below this are indistinguishable from noise
//   and are reported as zero. TV (0.68 A) and rice cooker (4.55 A) sit well clear.
//   Phone charger (0.068 A) is far below and REQUIRES 10-turn compensation to be seen.
// v0.5: last RAW (uncompensated) current per channel, so the noise floor can be re-measured
// without guessing backwards from the compensated value. Printed on the [READ] line.
float lastRawCurrent[3] = {0, 0, 0};
// ⚠ PENDING RE-MEASUREMENT (2026-08-29, Session 32).
// A reading of 0.309-0.382 A on an apparently idle circuit was briefly attributed to induced
// noise from the newly-installed relay, and the floor was raised to 0.420 to compensate.
// That was WRONG: the LED was plugged into the same relay-switched circuit and drawing, so
// those figures were a genuine measurement, not a floor. Reverted to 0.300.
// LESSON: "idle" means EVERY load on the monitored circuit disconnected, not just the
// appliance of interest. Verify what is physically plugged in before attributing a reading
// to noise — otherwise real current gets compensated away and the limit of detection is
// degraded for no reason.
// The true floor with the relay installed has NOT yet been measured cleanly. Take it with
// the relay CLOSED and both the LED and iron disconnected, then set this from the observed
// maximum. ch2/ch3 values are moot — CHANNEL_INSTRUMENTED gates those channels to zero.
const float NOISE_FLOOR_A[3] = {0.300, 0.140, 0.150};

// v0.4 ZMPT101B voltage sensing (Session 31, ch1 wired 2026-08-28).
// EmonLib has no standalone Vrms call, so readVoltage() samples the ADC directly over a
// whole number of mains cycles and computes RMS via the variance identity:
//   Vrms_counts = sqrt( E[x^2] - E[x]^2 )
// which removes the DC bias without needing to know it in advance (the module biases to
// ~1.65V at 3.3V supply, but that drifts, so measuring it each pass is more robust).
// ZMPT_CAL converts RMS ADC counts to volts. Set to 1.0 until calibrated: run with
// VOLTAGE_DIAGNOSTIC on, read the [VOLT] line's rms_counts, compare against the plug-in
// watt reader's V display, then ZMPT_CAL = watt_reader_volts / rms_counts.
// CALIBRATED 2026-08-28 (Session 31), ch1 only:
//   multimeter 227 V  vs  rms_counts 367.24 (mean of 366.71 / 367.77)  ->  227/367.24 = 0.6181
// Calibrated against the ELECTRICIAN'S DIGITAL MULTIMETER, not the plug-in watt reader —
// the two disagreed by 5-7% on voltage (DMM 225-230 V vs watt reader 210-220 V idle) and a
// DMM is the better voltage instrument. Record both in the paper; the disagreement is real
// measurement uncertainty and belongs in the limitations.
// ⚠ NOT YET VERIFIED against a simultaneous reading — the diagnostic run showed rms_counts
// in two clusters (366-368 then 334) which is most likely a genuine line sag when something
// switched on elsewhere in the house, but could be a sampling artifact. Verify by reading
// the firmware's reported V and the DMM at the SAME instant, then refine:
//   ZMPT_CAL_NEW = ZMPT_CAL * (dmm_volts / firmware_volts)
// Refinement pass 2 (same session), after reseating the loose VCC/GND/OUT jumpers that
// had been making the bias wander between 617 and 4033 counts:
//   DMM 228.5 V  vs  rms_counts 360.14 (mean of 7 clean windows)  ->  228.5/360.14 = 0.6345
// FINAL (Session 31). Four calibration points taken across the session vs the electrician's DMM:
//     227.0 V / 367.24 counts = 0.6181   (single reading)
//     228.5 V / 360.14 counts = 0.6345   (single reading)
//     229.0 V / 373.04 counts = 0.6139   (single reading)
//     219.0 V / 362.83 counts = 0.6036   (8 consecutive readings, DMM held steady) <-- ADOPTED
// The first three pair one DMM glance against a few cycles and carry pairing error from
// non-simultaneity; the fourth is 8 consecutive firmware readings against a DMM the operator
// confirmed was steady, so it is the best-quality pairing and is adopted rather than a
// straight mean of all four (which would give 0.6175, only 2.3% away in any case).
// ⚠ The full spread across the four points, 0.6036-0.6345, IS the residual uncertainty.
// Report voltage accuracy as approximately +/-3%, NOT as a single clean figure. The line
// itself moved from 229 V to 219 V during the session, so no single-point calibration here
// is reproducible — this is a property of the supply, not of the sensor.
float ZMPT_CAL[3] = {0.6036, 1.0, 1.0};

// While true, every cycle prints raw ADC min/max/RMS for the voltage pins WITHOUT
// affecting the power math (readVoltage still returns the 220 placeholder until
// VOLTAGE_SENSOR_CONNECTED is flipped). This lets ZMPT_CAL be calibrated safely:
// a wrong voltage would corrupt power/energy/cost AND could trip Layer 1.
// Turn OFF once ZMPT_CAL is set and VOLTAGE_SENSOR_CONNECTED is true.
const bool VOLTAGE_DIAGNOSTIC = true;

// Which channels physically have a ZMPT101B fitted. Only ch1 as of Session 31.
// Channels without one must NOT read their own pin — it floats, and floating ADC noise
// scaled by ZMPT_CAL would report hundreds of volts and trip Layer 1 OVER_VOLTAGE.
const bool ZMPT_WIRED[3] = {true, false, false};

// Unwired channels inherit this channel's voltage. Physically correct here: the house is a
// SINGLE-PHASE supply, so every outlet shares the same line voltage (bar small per-circuit
// wiring drops). This matches the paper's single-phase framing. If per-channel ZMPTs are
// ever fitted, set ZMPT_WIRED accordingly and each channel reads its own.
const int VOLTAGE_REF_CH = 0;

// v0.4 SENSOR-VALIDITY GUARD.
// If the ZMPT loses its AC input (pigtail unplugged, terminal works loose, mains off) the
// module keeps producing a biased output with almost no AC riding on it, so the RMS collapses
// to ~26 counts instead of ~363. Scaled by ZMPT_CAL that yields a plausible-looking ~15.6 V,
// which the firmware would otherwise log as truth and feed into P = V * I -- silently
// corrupting power, energy, cost and carbon for every row until someone noticed.
// Observed 2026-08-28: pigtail unplugged after calibration, DB logged voltage 15.64896 for
// every row while the actual supply was ~219 V.
//
// A healthy signal is ~360 counts. Collapsed is 26-40. Threshold of 100 counts (~60 V at the
// calibrated constant) separates them cleanly with huge margin: even a severe brownout to
// 100 V would still read ~166 counts, well clear. Anything below 100 is a SENSOR FAULT, not
// a supply condition.
//
// On fault the reading falls back to the nominal placeholder rather than 0, because 0 would
// make power read 0 and hide the failure as "no load". The fallback is exactly 220.000, and
// genuine measurements are never exactly that -- so affected rows stay identifiable and
// filterable in analysis with `WHERE voltage = 220.0`.
const float VOLT_MIN_VALID_COUNTS   = 100.0;
const float VOLTAGE_NOMINAL_FALLBACK = 220.0;

// v0.6 (Session 32): UPPER sanity bound on voltage.
// The lower guard above catches a COLLAPSED signal, but there was no guard against an
// implausibly HIGH one — and that turned out to be the dominant fault mode.
// Measured 2026-08-29: an intermittent ZMPT connection produced readings of 655-819 V on a
// 220 V supply. Those readings exceeded LAYER1_VOLTAGE_MAX and tripped all three relays,
// accounting for 13 of 25 trips in the collection window — a 52% false alarm rate, every one
// of them an over-voltage trip on a reading that was physically impossible.
// A Philippine residential supply cannot reach 300 V; anything above it is a sensor fault,
// not a grid event. Treated identically to a collapsed signal: fall back to the flagged
// placeholder rather than act on a false reading.
const float VOLT_MAX_VALID_V = 300.0;
bool voltageSensorHealthy[3] = {true, true, true};

// NOTE: the PIN_VOLTAGE[3] lookup array is declared further down, immediately after
// PIN_VOLTAGE_CH1/2/3 are defined in the GPIO PIN ASSIGNMENTS block — it cannot live
// here because those constants do not exist yet at this point in the file.

// Sample window: 100 ms = 6 full cycles at 60 Hz. Whole cycles matter — a partial cycle
// biases the RMS. Also records min/max so clipping against the 0/4095 rails is visible.
// 50 ms = exactly 3 cycles at 60 Hz. Whole cycles matter — a partial cycle biases the RMS.
// Three of these windows are taken per reading and the MEDIAN is used, because WiFi/HTTP
// activity occasionally preempts the sampling loop and one window comes back with a
// collapsed span (observed: span 228 / rms 40 against a normal span 1030 / rms 365, which
// would have written a phantom ~25 V row). A median of 3 rejects a single bad window.
const unsigned long VOLT_WINDOW_US = 50000UL;
const int VOLT_WINDOWS = 3;

int lastVmin[3] = {0, 0, 0};
int lastVmax[3] = {0, 0, 0};
float lastVrmsCounts[3] = {0, 0, 0};

// v0.4: EmonLib settling controls.
// calcIrms() applies a high-pass filter to strip the ~1.65V DC bias. Its filter state
// starts at zero, so the first calls return hugely inflated currents until it converges.
// EMON_SETTLE_PASSES discards that many passes per channel inside setup();
// LAYER1_WARMUP_CYCLES additionally holds Layer 1 disarmed for the first N loop cycles
// so a cold filter can never trip the relays on boot.
const int EMON_SETTLE_PASSES  = 5;
const int LAYER1_WARMUP_CYCLES = 3;

// Transmission interval
const int INTERVAL_MS = 7000; // send readings every 7 seconds — increased from 5s to give phone SQLite WASM more headroom per cycle
const int HTTP_TIMEOUT_MS = 1500; // Session 21: cut from 6000ms — backend confirmed fast via [SLOW INSERT] diagnostic (zero slow events); shorter timeout catches WiFi recovery sooner without waiting through long stalls

// Session 21: Ring buffer for cycles that fail to POST — preserves data through WiFi blips
const int BUFFER_SIZE = 100;       // capacity: ~12 minutes of buffered cycles at 7s intervals
const int DRAIN_PER_CYCLE = 10;    // max buffered cycles to drain per successful POST

// Session 21: Hardware watchdog + WiFi reconnect timeout
const int WDT_TIMEOUT_MS = 90000;             // hardware watchdog: reset ESP32 if loop() stops feeding it (90s)
const int WIFI_RECONNECT_TIMEOUT_MS = 60000;  // give up + restart if WiFi can't reconnect in 60s

// ============================================================
// GPIO PIN ASSIGNMENTS
// ============================================================

// Current sensors (SCT-013-030) — ADC1 only, never ADC2
const int PIN_CURRENT_CH1 = 32;
const int PIN_CURRENT_CH2 = 33;
const int PIN_CURRENT_CH3 = 34;

// Voltage sensors (ZMPT101B) — ADC1 only
const int PIN_VOLTAGE_CH1 = 35;
const int PIN_VOLTAGE_CH2 = 36;
const int PIN_VOLTAGE_CH3 = 39;

// v0.4: lookup array for sampleVoltageRaw(). Must be declared AFTER the three constants
// above — declaring it up beside ZMPT_CAL fails to compile ('not declared in this scope').
const int PIN_VOLTAGE[3] = {PIN_VOLTAGE_CH1, PIN_VOLTAGE_CH2, PIN_VOLTAGE_CH3};

// Relay outputs — active-LOW (LOW = tripped, HIGH = normal)
const int PIN_RELAY_CH1 = 26;
const int PIN_RELAY_CH2 = 27;
const int PIN_RELAY_CH3 = 16; // v0.3: was 14 — GPIO 14 is an ESP32 strapping pin (boot mode), can fluctuate at startup causing spurious relay clicks. GPIO 16 has no boot-time function.

// ============================================================
// APPLIANCE CONFIG
// ============================================================
const char* APPLIANCE_NAMES[3] = {"rice_cooker", "tv", "phone_charger"};

// Baseline wattage per channel — fill in after initial calibration
float baselineWatts[3] = {0, 0, 0}; // legacy Layer 2 field — superseded by learnedThreshold[] below

// ============================================================
// v0.5 (Session 32) — LAYER 2 ADAPTIVE THRESHOLD
// Previously this layer was DEAD CODE: baselineWatts[] was declared and never assigned, so
// the `if (baselineWatts[ch] > 0)` guard was permanently false and no adaptive detection
// existed. The backend had no threshold endpoints either. Both are implemented now.
//
// Behaviour:
//   TRAINING  — for TRAINING_DURATION_MS the firmware records the maximum wattage seen on a
//               channel, logging each running-max sample to the backend so the convergence
//               curve can be analysed (this is the source data for Q3.4).
//   ARMED     — threshold = learned_max x LAYER2_MULTIPLIER. Exceeding it trips the relay.
//   PERSISTED — the threshold is written to the backend AND to on-device NVS. On boot the
//               ESP32 tries the backend first, falls back to NVS if the backend is
//               unreachable, and falls back to Layer 1 alone if neither is available —
//               so there is never a window without protection (Q3.5).
// ============================================================
const float LAYER2_MULTIPLIER = 1.20;              // paper: 1.20 x learned maximum
const unsigned long TRAINING_DURATION_MS = 1800000UL; // 30 min per the Data Collection Procedure
const unsigned long TRAINING_LOG_EVERY_MS = 30000UL;  // log a convergence sample every 30 s

bool  trainingActive[3]   = {false, false, false};
unsigned long trainingStart[3] = {0, 0, 0};
unsigned long trainingLastLog[3] = {0, 0, 0};
float trainingMax[3]      = {0, 0, 0};
int   trainingSamples[3]  = {0, 0, 0};
float learnedThreshold[3] = {0, 0, 0};   // 0 = not learned; Layer 1 only
const char* thresholdSource[3] = {"none", "none", "none"}; // backend | nvs | trained | none

Preferences prefs;

// Which channels physically have a current sensor fitted. Only ch1 as of Session 32 —
// one SCT-013-030 was purchased. Channels 2-3 must never train or arm Layer 2, or they
// would learn a threshold from pure ADC noise.
const bool CHANNEL_INSTRUMENTED[3] = {true, false, false};

// Auto-start a training window on boot for any instrumented channel with no learned
// threshold. Set false once training is done if you don't want a reboot to retrain
// (a reboot with a stored threshold will NOT retrain either way — that is the Q3.5 test).
const bool TRAIN_ON_BOOT = true;

// v0.5: how often to re-fetch thresholds from the backend while running. Without this,
// changing a threshold requires an ESP32 reboot — which across 30 detection trials means
// 30 reboots. With it, POST a new threshold and the next trial can start within a minute.
// This is also what backs the per-channel "Recalibrate" control described for the Status tab.
const unsigned long THRESHOLD_REFRESH_MS = 60000UL; // 60 s
unsigned long lastThresholdRefresh = 0;

// Relay state — true = normal (ON), false = tripped (OFF)
bool relayState[3] = {true, true, true};

// Cumulative energy per channel (kWh) — persists across loop cycles
float energyKwh[3] = {0, 0, 0};

// millis() timestamp of last send cycle — used instead of delay() for accurate intervals
unsigned long lastSendTime = 0;
unsigned long cycleCount = 0; // v0.4: counts completed send-cycles; gates the Layer 1 warm-up

// millis() timestamp of last relay poll — relay is manual control, no need to poll every 5s
unsigned long lastRelayPollTime = 0;
// v0.5 (Session 32): 30000 -> 2000 ms.
// The 30 s interval dated from Session 19, chosen to reduce HTTP blocking per cycle. It was
// measured on 2026-08-29 to give a 19-second command-to-action latency, against Q4.2's target
// of under 2 seconds — the poll interval IS the parameter that determines that figure, so it
// had to change. /api/relay/pending is a cheap indexed lookup plus a small UPDATE, not one of
// the heavy aggregation endpoints that caused the Session 24-25 event-loop stalls.
// ⚠ Watch the gap rate after this change; if gaps regress, this is the first thing to revisit.
const int RELAY_POLL_INTERVAL_MS = 2000; // poll relay commands every 2 seconds (Q4.2: <2s target)

// EmonLib instances — one per current channel
EnergyMonitor emon[3];

// Session 21: Ring buffer to preserve readings when POST fails (WiFi blip / backend outage)
struct CycleData {
  float voltages[3];
  float currents[3];
  float powers[3];
  float kwhs[3];
  float costs[3];
  float carbons[3];
  unsigned long uptime_ms;
};
CycleData ringBuffer[BUFFER_SIZE];
int bufferWriteIdx = 0;  // next slot to write to
int bufferCount = 0;     // current items in buffer (0 to BUFFER_SIZE)

// Session 21 hotfix: global static JSON document — avoids heap fragmentation crashes
// (DynamicJsonDocument(8192) per-cycle was causing ESP32 to reboot every ~8 minutes)
StaticJsonDocument<8192> sendDoc;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Relay pins — set HIGH first (active-LOW, so HIGH = relay open = appliance ON)
  pinMode(PIN_RELAY_CH1, OUTPUT); digitalWrite(PIN_RELAY_CH1, HIGH);
  pinMode(PIN_RELAY_CH2, OUTPUT); digitalWrite(PIN_RELAY_CH2, HIGH);
  pinMode(PIN_RELAY_CH3, OUTPUT); digitalWrite(PIN_RELAY_CH3, HIGH);

  // EmonLib setup — SCT_CAL is the starting calibration constant for SCT-013-030.
  // Refine against the plug-in watt reader during Phase 1 calibration:
  //   adjustment = (watt_reader_amps / esp32_reported_amps); SCT_CAL_NEW = SCT_CAL * adjustment.
  emon[0].current(PIN_CURRENT_CH1, SCT_CAL);
  emon[1].current(PIN_CURRENT_CH2, SCT_CAL);
  emon[2].current(PIN_CURRENT_CH3, SCT_CAL);

  // v0.4: pre-settle the EmonLib DC-offset filter before loop() ever runs.
  // Without this the first reading of every boot is inflated far past LAYER1_POWER_MAX
  // and instantly trips all three relays. Results here are discarded on purpose.
  if (CURRENT_SENSOR_CONNECTED) {
    Serial.print("[INIT] Settling EmonLib filters");
    for (int pass = 0; pass < EMON_SETTLE_PASSES; pass++) {
      for (int ch = 0; ch < 3; ch++) emon[ch].calcIrms(1480);
      Serial.print(".");
    }
    Serial.println(" done");
    for (int ch = 0; ch < 3; ch++) {
      Serial.print("[INIT] ch"); Serial.print(ch + 1);
      Serial.print(" settled Irms = "); Serial.println(emon[ch].calcIrms(1480), 3);
    }

    // v0.4 ADC PROBE — diagnostic for the EmonLib ADC_BITS mismatch.
    // EmonLib scales by (Vsupply/1000)/ADC_COUNTS where ADC_COUNTS = 1<<ADC_BITS.
    // On AVR that is 1024; the ESP32's analogRead() returns 0-4095. If EmonLib was
    // compiled with ADC_BITS=10 on this platform, every current reads 4x too high.
    // Printing the observed raw range settles it: a span reaching ~4095 means 12-bit
    // hardware, so ADC_COUNTS must be 4096 for the scaling to be correct.
    int rawMin = 4095, rawMax = 0;
    for (int i = 0; i < 3000; i++) {
      int r = analogRead(PIN_CURRENT_CH1);
      if (r < rawMin) rawMin = r;
      if (r > rawMax) rawMax = r;
    }
    Serial.print("[ADC] ch1 raw min="); Serial.print(rawMin);
    Serial.print(" max="); Serial.print(rawMax);
    Serial.print("  EmonLib ADC_COUNTS="); Serial.println(ADC_COUNTS);
  }

  WiFi.setAutoReconnect(true); // auto-reconnect on drop without needing watchdog to catch it
  esp_wifi_set_ps(WIFI_PS_NONE); // lower-level power save disable — more reliable than WiFi.setSleep(false) alone
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // max TX power — stabilizes radio, reduces periodic background task interference
  connectWiFi();

  // v0.5: load the persisted Layer 2 thresholds. Backend first, NVS fallback, Layer 1 alone
  // if neither — so there is never a cold-start window without protection (Q3.5).
  loadThresholds(true);   // verbose on boot

  // v0.5: start a training window automatically if a channel has a sensor fitted but no
  // learned threshold yet. TRAIN_ON_BOOT can be set false to suppress this once training
  // is complete and you don't want a reboot to retrain.
  if (TRAIN_ON_BOOT) {
    for (int ch = 0; ch < 3; ch++) {
      if (CHANNEL_INSTRUMENTED[ch] && learnedThreshold[ch] <= 0) startTraining(ch);
    }
  }

  // Session 21: Hardware watchdog — auto-reset ESP32 if loop() stops feeding it
  // Backup safety net for any firmware lockup we haven't anticipated
  // IDF 5.x API: use config struct (legacy 2-arg form removed)
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,    // don't watch core idle tasks (we only care about loopTask)
    .trigger_panic = true   // panic + reset on timeout
  };
  // Arduino framework auto-inits TWDT on recent board packages — try init, reconfigure if already on
  esp_err_t wdtErr = esp_task_wdt_init(&wdtConfig);
  if (wdtErr != ESP_OK) {
    esp_task_wdt_reconfigure(&wdtConfig); // override framework defaults with our 90s timeout
  }
  esp_task_wdt_add(NULL);   // add current task (loopTask) to watchdog
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  esp_task_wdt_reset(); // Session 21: feed the hardware watchdog every iteration

  // WiFi watchdog — reconnect if dropped, don't just skip silently
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WARN] WiFi disconnected — reconnecting...");
    connectWiFi();
    return;
  }

  // millis()-based interval: measured from cycle START not END
  // Prevents HTTP call duration from adding to the gap between readings
  unsigned long now = millis();
  // v0.5 FIX (Session 32): the relay poll MUST run before the send-interval early-return.
  // It previously sat below that return, so it was only reached once per INTERVAL_MS (7 s)
  // and RELAY_POLL_INTERVAL_MS had no effect whatsoever. Measured consequence: manual control
  // latency 0.42-7.43 s, mean 3.97 s over 20 trials — against Q4.2's target of under 2 s.
  // Polling here makes the 2 s interval real. checkRelayCommands() is a single cheap indexed
  // GET, so running it between send cycles does not meaningfully load the backend.
  if (now - lastRelayPollTime >= RELAY_POLL_INTERVAL_MS) {
    lastRelayPollTime = now;
    checkRelayCommands();
  }

  if (now - lastSendTime < INTERVAL_MS) return;
  lastSendTime = now;
  cycleCount++; // v0.4: drives the Layer 1 warm-up guard

  // v0.5: periodically re-fetch thresholds so a threshold changed on the backend takes effect
  // without a reboot. Skipped while any channel is training — a mid-training refresh would
  // overwrite the value the training run is about to produce.
  bool anyTraining = trainingActive[0] || trainingActive[1] || trainingActive[2];
  if (!anyTraining && (now - lastThresholdRefresh >= THRESHOLD_REFRESH_MS)) {
    lastThresholdRefresh = now;
    float before[3] = { learnedThreshold[0], learnedThreshold[1], learnedThreshold[2] };
    loadThresholds(false); // quiet on periodic refresh; changes are logged below
    for (int c = 0; c < 3; c++) {
      if (learnedThreshold[c] != before[c]) {
        Serial.print("[LAYER2] ch"); Serial.print(c + 1);
        Serial.print(" threshold updated "); Serial.print(before[c], 1);
        Serial.print("W -> "); Serial.print(learnedThreshold[c], 1);
        Serial.println("W (backend refresh)");
      }
    }
  }
  if (cycleCount == (unsigned long)LAYER1_WARMUP_CYCLES) {
    Serial.println("[LAYER1] warm-up complete — safety floor ARMED");
  }

  // Collect all readings first, then send in one batch POST
  float voltages[3], currents[3], powers[3], costs[3], carbons[3];

  // v0.4 ZMPT CALIBRATION DIAGNOSTIC — ch1 only. Prints raw ADC min/max and RMS counts
  // without touching the power math, so ZMPT_CAL can be derived safely.
  // ZMPT_CAL = watt_reader_volts / rms_counts. Clipping shows as min near 0 or max near 4095.
  // v0.4 FIX: the [VOLT] diagnostic used to call sampleVoltageRaw() itself, which meant it
  // reported a DIFFERENT sample from the one readVoltage() fed to the Layer 1 check — an
  // over-voltage trip could fire on a value that never appeared in the log. The raw
  // voltage figures are now printed on the [READ] line below, straight from the cached
  // values that the power math and the trip logic actually used. One sample, one truth.

  // v0.4 CALIBRATION DIAGNOSTIC — prints every cycle so readings are visible during
  // Phase 1 calibration against the plug-in watt reader. Remove or gate behind a flag
  // once SCT_CAL is finalised; it is noisy for long unattended runs.
  Serial.print("[READ] ");

  for (int ch = 0; ch < 3; ch++) {
    voltages[ch] = readVoltage(ch);
    currents[ch] = readCurrent(ch);
    powers[ch]   = calculatePower(voltages[ch], currents[ch]);

    Serial.print("ch"); Serial.print(ch + 1);
    Serial.print(" V="); Serial.print(voltages[ch], 1);
    Serial.print(" I="); Serial.print(currents[ch], 3);
    Serial.print("A P="); Serial.print(powers[ch], 1);
    Serial.print("W  ");
    if (ch == 0 && VOLTAGE_DIAGNOSTIC) {
      // ch1 raw voltage detail — same cached sample the trip logic used.
      Serial.print("[Iraw="); Serial.print(lastRawCurrent[0], 3);
      Serial.print("A  Vspan="); Serial.print(lastVmax[0] - lastVmin[0]);
      Serial.print(" rms="); Serial.print(lastVrmsCounts[0], 2);
      Serial.print("]  ");
    }
    if (ch == 2) Serial.println();

    // Accumulate energy.
    // ⚠ UNITS BUG FIXED 2026-08-29 (Session 31). The original line was:
    //     energyKwh[ch] += powers[ch] * (INTERVAL_MS / 1000.0 / 3600.0);
    // which is  W x h = WATT-hours, stored in a field named kWh. The /1000.0 conversion
    // to kilowatt-hours was missing, so every energy value — and therefore every cost and
    // carbon figure derived from it — has been exactly 1000x too large for the entire life
    // of the project. Confirmed against live data: at 251.363 W the per-cycle increment was
    // 0.4667, matching the Wh expectation of 0.4887, not the kWh expectation of 0.00049.
    // calculateCost() and calculateCarbon() apply no compensating factor, so both inherit it.
    //
    // HISTORICAL DATA IS RECOVERABLE, NOT LOST: every stored energy_kwh / cost_php /
    // carbon_kg value is correct once divided by 1000. See knowledge/data-export.md.
    energyKwh[ch] += powers[ch] * (INTERVAL_MS / 1000.0 / 3600.0) / 1000.0;

    costs[ch]   = calculateCost(energyKwh[ch]);
    carbons[ch] = calculateCarbon(energyKwh[ch]);

    // Layer 1 — Hardcoded Safety Floor (per paper/rebrand-plan.md §5.1)
    // Always-on, runs BEFORE Layer 2 adaptive check. No debounce — catastrophic-fault path.
    // OVER_VOLTAGE only checks when ZMPT is wired (otherwise V is stuck at 220 placeholder).
    // OVER_POWER fires once SCT is wired and any channel exceeds the absolute ceiling.
    // v0.4: Layer 1 stays disarmed for the first LAYER1_WARMUP_CYCLES cycles so a
    // still-converging EmonLib filter can never trip the relays immediately after boot.
    bool layer1Armed = (cycleCount >= (unsigned long)LAYER1_WARMUP_CYCLES);
    bool layer1Trip = false;
    const char* layer1Reason = nullptr;
    float layer1Threshold = 0.0;
    if (layer1Armed && VOLTAGE_SENSOR_CONNECTED && voltages[ch] > LAYER1_VOLTAGE_MAX) {
      layer1Trip = true;
      layer1Reason = "OVER_VOLTAGE";
      layer1Threshold = LAYER1_VOLTAGE_MAX;
    } else if (layer1Armed && powers[ch] > LAYER1_POWER_MAX) {
      layer1Trip = true;
      layer1Reason = "OVER_POWER";
      layer1Threshold = LAYER1_POWER_MAX;
    }
    if (layer1Trip && relayState[ch]) {
      long startTime = millis();
      tripRelay(ch);
      long responseTime = millis() - startTime;
      sendAnomaly(ch, powers[ch], layer1Threshold, 0.0, responseTime);
      Serial.print("[LAYER1] ");
      Serial.print(layer1Reason);
      Serial.print(" trip ch");
      Serial.println(ch + 1);
    }

    // v0.5: feed the training window if one is running on this channel.
    updateTraining(ch, powers[ch]);

    // Layer 2 — Per-Appliance Adaptive Threshold (IMPLEMENTED v0.5, Session 32).
    // Threshold = learned maximum x LAYER2_MULTIPLIER, learned during a training window and
    // persisted to the backend + NVS so it survives a reboot (Q3.5).
    // Suppressed while training is active on this channel — the appliance is deliberately
    // being run at its normal maximum during training and must not trip on itself.
    if (learnedThreshold[ch] > 0 && !trainingActive[ch]) {
      float threshold = learnedThreshold[ch];
      if (powers[ch] > threshold && relayState[ch]) {
        long startTime = millis();
        tripRelay(ch);
        long responseTime = millis() - startTime;
        // v0.5: deviation is measured against the LEARNED MAXIMUM, not the dead baselineWatts[]
        // field (which was never assigned and would divide by zero, sending Inf to the backend).
        float learnedMax = threshold / LAYER2_MULTIPLIER;
        float deviation = (learnedMax > 0) ? ((powers[ch] - learnedMax) / learnedMax) * 100.0 : 0.0;
        sendAnomaly(ch, powers[ch], threshold, deviation, responseTime);
        Serial.print("[LAYER2] over-wattage trip ch"); Serial.print(ch + 1);
        Serial.print(" — "); Serial.print(powers[ch], 1);
        Serial.print("W exceeded learned threshold "); Serial.print(threshold, 1);
        Serial.print("W (+"); Serial.print(deviation, 1); Serial.println("% over learned max)");
      }
    }
  }
  // One HTTP call for all 3 channels — prevents sequential POST stalls
  sendAllReadings(voltages, currents, powers, energyKwh, costs, carbons);
}

// ============================================================
// SENSOR READING FUNCTIONS
// v0.3: Real SCT-013 reads via EmonLib when CURRENT_SENSOR_CONNECTED=true.
// ZMPT101B reads still pending — flips on when VOLTAGE_SENSOR_CONNECTED=true.
// ============================================================
// Samples the ZMPT output over VOLT_WINDOW_US and stores RMS counts + min/max for the
// channel. Always safe to call — it only reads the ADC and never affects power math.
void sampleVoltageRaw(int channel) {
  int pin = PIN_VOLTAGE[channel];
  float rms[VOLT_WINDOWS];
  int   mn[VOLT_WINDOWS], mx[VOLT_WINDOWS];

  for (int w = 0; w < VOLT_WINDOWS; w++) {
    unsigned long start = micros();
    unsigned long n = 0;
    double sum = 0, sumSq = 0;
    int vmin = 4095, vmax = 0;

    while (micros() - start < VOLT_WINDOW_US) {
      int r = analogRead(pin);
      sum   += r;
      sumSq += (double)r * (double)r;
      if (r < vmin) vmin = r;
      if (r > vmax) vmax = r;
      n++;
    }

    if (n == 0) { rms[w] = 0; mn[w] = 0; mx[w] = 0; continue; }
    double mean = sum / (double)n;
    double var  = (sumSq / (double)n) - (mean * mean);
    if (var < 0) var = 0; // floating-point guard
    rms[w] = sqrt(var);
    mn[w]  = vmin;
    mx[w]  = vmax;
  }

  // Median of 3 — rejects a single window corrupted by a scheduling stall.
  int mid;
  if (rms[0] <= rms[1]) mid = (rms[1] <= rms[2]) ? 1 : ((rms[0] <= rms[2]) ? 2 : 0);
  else                  mid = (rms[0] <= rms[2]) ? 0 : ((rms[1] <= rms[2]) ? 2 : 1);

  lastVrmsCounts[channel] = rms[mid];
  lastVmin[channel]       = mn[mid];
  lastVmax[channel]       = mx[mid];
}

float readVoltage(int channel) {
  if (!VOLTAGE_SENSOR_CONNECTED) {
    return VOLTAGE_NOMINAL_FALLBACK; // In effect until ZMPT is wired AND calibrated.
  }
  // Channels without a physical ZMPT borrow the reference channel's reading (single-phase
  // supply — see VOLTAGE_REF_CH). Only the source channel actually samples, so we spend one
  // VOLT_WINDOW_US per cycle, not three. Relies on loop() processing ch0 first.
  int src = ZMPT_WIRED[channel] ? channel : VOLTAGE_REF_CH;
  if (channel == src) sampleVoltageRaw(src);

  // Validity guard — see VOLT_MIN_VALID_COUNTS. A collapsed signal means the sensor has lost
  // its AC input, NOT that the mains has dropped to 15 V.
  if (lastVrmsCounts[src] < VOLT_MIN_VALID_COUNTS) {
    if (voltageSensorHealthy[src]) {           // edge-triggered so we warn once, not every cycle
      voltageSensorHealthy[src] = false;
      Serial.print("[FAULT] Voltage sensor ch"); Serial.print(src + 1);
      Serial.print(" signal collapsed (rms="); Serial.print(lastVrmsCounts[src], 1);
      Serial.print(" counts, need >"); Serial.print(VOLT_MIN_VALID_COUNTS, 0);
      Serial.print("). Check the AC pigtail is plugged in and the screw terminals are tight. ");
      Serial.print("Falling back to "); Serial.print(VOLTAGE_NOMINAL_FALLBACK, 1);
      Serial.println(" V — rows logged during this period are NOT measured voltage.");
    }
    return VOLTAGE_NOMINAL_FALLBACK;
  }

  float v = lastVrmsCounts[src] * ZMPT_CAL[src];

  // v0.6: upper sanity bound — see VOLT_MAX_VALID_V. An impossible reading must not be
  // allowed to trip Layer 1 over-voltage.
  if (v > VOLT_MAX_VALID_V) {
    if (voltageSensorHealthy[src]) {
      voltageSensorHealthy[src] = false;
      Serial.print("[FAULT] Voltage sensor ch"); Serial.print(src + 1);
      Serial.print(" implausible reading "); Serial.print(v, 1);
      Serial.print(" V (max plausible "); Serial.print(VOLT_MAX_VALID_V, 0);
      Serial.println(" V) — sensor fault, not a grid event. Check the AC connection.");
    }
    return VOLTAGE_NOMINAL_FALLBACK;
  }

  if (!voltageSensorHealthy[src]) {
    voltageSensorHealthy[src] = true;
    Serial.print("[OK] Voltage sensor ch"); Serial.print(src + 1);
    Serial.println(" recovered — real measurement resumed.");
  }
  return v;
}

float readCurrent(int channel) {
  if (CURRENT_SENSOR_CONNECTED) {
    // EmonLib calcIrms — 1480 samples is the EmonLib standard for 60Hz mains.
    // Returns RMS current in amperes.
    // v0.5: a channel with no current transformer fitted has nothing to measure. Its ADC pin
    // is only jumpered to the bias midpoint, so anything it "reads" is induced noise — and
    // that noise was being reported as real current (observed up to 0.57 A / 125 W on ch2,
    // spiking in lockstep with ch3 whenever the ch2/ch3 relay coils energised nearby).
    // Reporting it also accumulated fake energy and cost on channels that do not exist.
    // No sensor => no measurement => zero. This is the honest value, not a suppression.
    if (!CHANNEL_INSTRUMENTED[channel]) { lastRawCurrent[channel] = 0.0; return 0.0; }

    // v0.5: if this channel's relay is OPEN the circuit is physically interrupted, so no
    // current path exists and the true current is necessarily zero. Anything the transformer
    // reports in that state is induced artifact, not measurement.
    // Measured 2026-08-29: with the relay open the raw reading rose from ~0.17-0.21 A to
    // ~0.29-0.34 A, straddling the noise floor and producing a flickering 0-40 W on a circuit
    // that had just been physically disconnected. Cause is the relay module itself — it is
    // active-LOW, so an OPEN relay means an ENERGISED coil drawing ~80 mA and radiating from a
    // component sitting beside the sensor wiring. Closed relay = de-energised coil = quiet.
    // NOTE: this is a correctness guard, not a substitute for physical separation. Keep the
    // CT cable away from the relay module and its mains wiring.
    if (!relayState[channel]) { lastRawCurrent[channel] = 0.0; return 0.0; }

    float raw = emon[channel].calcIrms(1480);
    lastRawCurrent[channel] = raw; // v0.5: exposed for noise-floor re-measurement

    // v0.4: quadrature noise-floor subtraction. See NOISE_FLOOR_A for rationale and
    // the disclosure requirement. Clamps to 0 rather than returning an imaginary root
    // when the reading sits at or below the floor (i.e. no detectable load).
    float floorA = NOISE_FLOOR_A[channel];
    if (raw <= floorA) return 0.0;
    return sqrt((raw * raw) - (floorA * floorA));
  }
  return 0.0; // Placeholder while SCT-013 not yet wired.
}

// ============================================================
// CALCULATION FUNCTIONS
// ============================================================
float calculatePower(float voltage, float current) {
  return voltage * current; // apparent power in watts
}

float calculateCost(float kwh) {
  return kwh * VECO_RATE;
}

float calculateCarbon(float kwh) {
  return kwh * CARBON_FACTOR;
}

// ============================================================
// ANOMALY DETECTION + RELAY CONTROL
// ============================================================
// ============================================================
// v0.5 — LAYER 2 ADAPTIVE THRESHOLD FUNCTIONS
// ============================================================

// Persist a learned threshold to on-device NVS so it survives a reboot even with no network.
void saveThresholdNVS(int ch, float thr, float learnedMax) {
  prefs.begin("reap", false);
  prefs.putFloat((String("thr") + ch).c_str(), thr);
  prefs.putFloat((String("max") + ch).c_str(), learnedMax);
  prefs.end();
}

float loadThresholdNVS(int ch) {
  prefs.begin("reap", true); // read-only
  float thr = prefs.getFloat((String("thr") + ch).c_str(), 0.0);
  prefs.end();
  return thr;
}

// Push the finished threshold to the backend so it survives even a full device replacement.
void postThreshold(int ch, float learnedMax, float thr, int samples, unsigned long trainSecs) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(BACKEND_URL_THRESHOLD);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", API_KEY);
  http.setTimeout(HTTP_TIMEOUT_MS);
  StaticJsonDocument<256> doc;
  doc["channel"]          = ch + 1;
  doc["appliance"]        = APPLIANCE_NAMES[ch];
  doc["learned_max_w"]    = learnedMax;
  doc["threshold_w"]      = thr;
  doc["multiplier"]       = LAYER2_MULTIPLIER;
  doc["samples"]          = samples;
  doc["training_seconds"] = trainSecs;
  String payload; serializeJson(doc, payload);
  http.POST(payload);
  http.end();
}

// One convergence sample. Q3.4 is analysed from the series of these.
void postTrainingSample(int ch, unsigned long elapsedS, int n, float p, float runMax) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(BACKEND_URL_THRESHOLD_TRAIN);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", API_KEY);
  http.setTimeout(HTTP_TIMEOUT_MS);
  StaticJsonDocument<200> doc;
  doc["channel"]       = ch + 1;
  doc["elapsed_s"]     = elapsedS;
  doc["sample_n"]      = n;
  doc["power_w"]       = p;
  doc["running_max_w"] = runMax;
  String payload; serializeJson(doc, payload);
  http.POST(payload);
  http.end();
}

// Boot-time load: backend first, NVS fallback, Layer 1 alone if neither. This ordering is
// what Q3.5 tests — the threshold must survive a reboot with no manual retraining.
void loadThresholds(bool verbose) {
  for (int ch = 0; ch < 3; ch++) {
    // Skip channels with no sensor fitted — they can never hold a meaningful threshold,
    // and skipping avoids two pointless HTTP round-trips on every refresh.
    if (!CHANNEL_INSTRUMENTED[ch]) continue;
    float thr = 0.0; const char* src = "none";

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(String(BACKEND_URL_THRESHOLD) + "/" + String(ch + 1));
      http.setTimeout(HTTP_TIMEOUT_MS);
      int code = http.GET();
      if (code == 200) {
        String body = http.getString();
        if (body.length() > 4 && body != "null") {
          StaticJsonDocument<384> doc;
          if (!deserializeJson(doc, body) && doc["threshold_w"].is<float>()) {
            thr = doc["threshold_w"].as<float>();
            src = "backend";
          }
        }
      }
      http.end();
    }

    if (thr <= 0) {                       // backend unreachable or nothing stored
      thr = loadThresholdNVS(ch);
      if (thr > 0) src = "nvs";
    }

    learnedThreshold[ch] = thr;
    thresholdSource[ch]  = src;

    if (verbose) {
      Serial.print("[LAYER2] ch"); Serial.print(ch + 1);
      if (thr > 0) {
        Serial.print(" threshold "); Serial.print(thr, 1);
        Serial.print("W (source: "); Serial.print(src); Serial.println(")");
      } else {
        Serial.println(" no learned threshold — Layer 1 floor only");
      }
    }
  }
}

// Begin a training window on a channel.
void startTraining(int ch) {
  trainingActive[ch]  = true;
  trainingStart[ch]   = millis();
  trainingLastLog[ch] = 0;
  trainingMax[ch]     = 0;
  trainingSamples[ch] = 0;
  Serial.print("[LAYER2] training started on ch"); Serial.print(ch + 1);
  Serial.print(" for "); Serial.print(TRAINING_DURATION_MS / 60000UL);
  Serial.println(" minutes — Layer 1 floor stays active throughout");
}

// Called once per cycle per channel while training is active.
void updateTraining(int ch, float powerW) {
  if (!trainingActive[ch]) return;
  unsigned long now = millis();
  unsigned long elapsed = now - trainingStart[ch];

  trainingSamples[ch]++;
  if (powerW > trainingMax[ch]) trainingMax[ch] = powerW;

  if (now - trainingLastLog[ch] >= TRAINING_LOG_EVERY_MS) {
    trainingLastLog[ch] = now;
    postTrainingSample(ch, elapsed / 1000UL, trainingSamples[ch], powerW, trainingMax[ch]);
  }

  if (elapsed >= TRAINING_DURATION_MS) {
    trainingActive[ch] = false;
    if (trainingMax[ch] <= 0) {
      Serial.print("[LAYER2] ch"); Serial.print(ch + 1);
      Serial.println(" training ended with no load observed — threshold NOT set");
      return;
    }
    float thr = trainingMax[ch] * LAYER2_MULTIPLIER;
    learnedThreshold[ch] = thr;
    thresholdSource[ch]  = "trained";
    saveThresholdNVS(ch, thr, trainingMax[ch]);
    postThreshold(ch, trainingMax[ch], thr, trainingSamples[ch], elapsed / 1000UL);
    Serial.print("[LAYER2] ch"); Serial.print(ch + 1);
    Serial.print(" training complete — learned max "); Serial.print(trainingMax[ch], 1);
    Serial.print("W, threshold "); Serial.print(thr, 1);
    Serial.print("W over "); Serial.print(trainingSamples[ch]);
    Serial.println(" samples (saved to NVS + backend)");
  }
}

void tripRelay(int channel) {
  int relayPins[3] = {PIN_RELAY_CH1, PIN_RELAY_CH2, PIN_RELAY_CH3};
  digitalWrite(relayPins[channel], LOW); // active-LOW: LOW = tripped
  relayState[channel] = false;
  Serial.print("Relay tripped — channel ");
  Serial.println(channel + 1);
}

void resetRelay(int channel) {
  int relayPins[3] = {PIN_RELAY_CH1, PIN_RELAY_CH2, PIN_RELAY_CH3};
  digitalWrite(relayPins[channel], HIGH);
  relayState[channel] = true;
}

// ============================================================
// WIFI
// ============================================================
void connectWiFi() {
  // Static IP — bypasses DHCP entirely, eliminates periodic DHCP renewal disruptions
  IPAddress localIP(192, 168, 1, 57);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);
  WiFi.config(localIP, gateway, subnet, dns);

  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi");
  // Session 21: Bail-out + restart if reconnect takes too long
  // Fixes the bug where this loop ran forever (caused 64-min gap on 2026-04-28 after 802.11n mode change)
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttempt > WIFI_RECONNECT_TIMEOUT_MS) {
      Serial.println("\n[FATAL] WiFi reconnect timed out — restarting ESP32");
      delay(100);
      ESP.restart();
    }
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected. IP: " + WiFi.localIP().toString());
  Serial.println("MAC: " + WiFi.macAddress());
  WiFi.setSleep(false); // disable modem sleep — prevents periodic radio powerdown causing 20-40s gaps
}

// ============================================================
// DATA TRANSMISSION (with ring buffer — Session 21)
// ============================================================
// Push a cycle's data into the ring buffer (called when POST fails or WiFi is down)
// If buffer is full, oldest entry is overwritten silently (FIFO drop, only loses data on >12min outages)
void bufferPush(float v[], float c[], float p[], float k[], float co[], float ca[], unsigned long up) {
  CycleData* slot = &ringBuffer[bufferWriteIdx];
  memcpy(slot->voltages, v, sizeof(slot->voltages));
  memcpy(slot->currents, c, sizeof(slot->currents));
  memcpy(slot->powers, p, sizeof(slot->powers));
  memcpy(slot->kwhs, k, sizeof(slot->kwhs));
  memcpy(slot->costs, co, sizeof(slot->costs));
  memcpy(slot->carbons, ca, sizeof(slot->carbons));
  slot->uptime_ms = up;
  bufferWriteIdx = (bufferWriteIdx + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) bufferCount++;
}

// Add 3 channel readings from a CycleData struct to a JSON array (helper)
void addCycleToArray(JsonArray& arr, CycleData* d) {
  for (int ch = 0; ch < 3; ch++) {
    JsonObject r = arr.createNestedObject();
    r["channel"]    = ch + 1;
    r["appliance"]  = APPLIANCE_NAMES[ch];
    r["voltage"]    = d->voltages[ch];
    r["current_a"]  = d->currents[ch];
    r["power_w"]    = d->powers[ch];
    r["energy_kwh"] = d->kwhs[ch];
    r["cost_php"]   = d->costs[ch];
    r["carbon_kg"]  = d->carbons[ch];
    r["uptime_ms"]  = d->uptime_ms;
  }
}

// sendAllReadings — POST current cycle + drain up to DRAIN_PER_CYCLE buffered cycles in same payload
// Session 21: Ring buffer logic — preserves data through WiFi blips and brief backend outages
// Backend already accepts arrays of any size (CODE/backend-phone.md POST /api/readings handler)
void sendAllReadings(float voltages[], float currents[], float powers[],
                     float kwhs[], float costs[], float carbons[]) {
  unsigned long uptimeMs = millis();

  if (WiFi.status() != WL_CONNECTED) {
    // No WiFi — buffer current cycle and bail. Will retry-drain when WiFi returns.
    bufferPush(voltages, currents, powers, kwhs, costs, carbons, uptimeMs);
    return;
  }

  // Build payload: drain up to DRAIN_PER_CYCLE buffered cycles + current cycle
  int drainN = (bufferCount < DRAIN_PER_CYCLE) ? bufferCount : DRAIN_PER_CYCLE;
  sendDoc.clear(); // Session 21 hotfix: reuse global static doc instead of per-cycle DynamicJsonDocument
  JsonArray arr = sendDoc.to<JsonArray>();

  // Add buffered cycles first (oldest first, chronological order)
  int readStart = (bufferWriteIdx - bufferCount + BUFFER_SIZE) % BUFFER_SIZE;
  for (int i = 0; i < drainN; i++) {
    int idx = (readStart + i) % BUFFER_SIZE;
    addCycleToArray(arr, &ringBuffer[idx]);
  }

  // Add current cycle (build temp CycleData)
  CycleData current;
  memcpy(current.voltages, voltages, sizeof(current.voltages));
  memcpy(current.currents, currents, sizeof(current.currents));
  memcpy(current.powers, powers, sizeof(current.powers));
  memcpy(current.kwhs, kwhs, sizeof(current.kwhs));
  memcpy(current.costs, costs, sizeof(current.costs));
  memcpy(current.carbons, carbons, sizeof(current.carbons));
  current.uptime_ms = uptimeMs;
  addCycleToArray(arr, &current);

  String payload;
  serializeJson(sendDoc, payload);

  // Retry up to 3x on failure (existing logic from Session 21 retry-timing fix)
  // Budget = 3*(1500+200)-200 = 4900ms, well under 7000ms cycle interval
  for (int attempt = 0; attempt < 3; attempt++) {
    if (WiFi.status() != WL_CONNECTED) break;
    HTTPClient http;
    http.begin(BACKEND_URL_READINGS);
    http.setConnectTimeout(HTTP_TIMEOUT_MS); // Session 21 hotfix: bound TCP connect time (default lwIP is 30s)
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + String(API_KEY));
    int httpCode = http.POST(payload);
    http.end();
    if (httpCode == 200 || httpCode == 201) {
      // Success — dequeue drained cycles
      if (drainN > 0) {
        bufferCount -= drainN;
        Serial.println("[BUFFER] Drained " + String(drainN) + " cycles, remaining: " + String(bufferCount));
      }
      return;
    }
    if (attempt < 2) delay(200);
  }

  // All retries failed — preserve current cycle (don't lose data)
  bufferPush(voltages, currents, powers, kwhs, costs, carbons, uptimeMs);
  Serial.println("[BUFFER] All retries failed, enqueued. Buffer size: " + String(bufferCount));
}

void sendAnomaly(int ch, float detected, float threshold,
                 float deviation, long responseMs) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(BACKEND_URL_ANOMALIES);
  http.setConnectTimeout(HTTP_TIMEOUT_MS); // Session 21 hotfix: bound TCP connect time
  http.setTimeout(HTTP_TIMEOUT_MS); // cap at 3s
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(API_KEY));

  StaticJsonDocument<256> doc;
  doc["channel"]           = ch + 1;
  doc["appliance"]         = APPLIANCE_NAMES[ch];
  doc["detected_wattage"]  = detected;
  doc["threshold_wattage"] = threshold;
  doc["deviation_pct"]     = deviation;
  doc["response_time_ms"]  = responseMs;
  doc["action_taken"]      = "relay_off_ch" + String(ch + 1);

  String payload;
  serializeJson(doc, payload);
  http.POST(payload);
  http.end();
}

// ============================================================
// RELAY COMMAND POLLING
// ============================================================
// checkRelayCommands — polls backend for pending dashboard relay commands
// Called every loop cycle. GET is open (no auth). ACK POST requires auth.
void checkRelayCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(BACKEND_URL_RELAY_PENDING);
  http.setConnectTimeout(HTTP_TIMEOUT_MS); // Session 21 hotfix: bound TCP connect time
  http.setTimeout(HTTP_TIMEOUT_MS);
  int httpCode = http.GET();

  if (httpCode != 200) {
    http.end();
    return;
  }

  String body = http.getString();
  http.end();

  // null response means no pending command
  if (body == "null" || body.length() < 2) return;

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) return;

  int cmdId      = doc["id"];
  int cmdChannel = doc["channel"];
  const char* cmdAction = doc["action"];

  if (cmdChannel < 1 || cmdChannel > 3) return;
  int ch = cmdChannel - 1;

  if (strcmp(cmdAction, "off") == 0) {
    tripRelay(ch);
    Serial.println("[RELAY] Remote OFF — ch" + String(cmdChannel));
  } else if (strcmp(cmdAction, "on") == 0) {
    resetRelay(ch);
    Serial.println("[RELAY] Remote ON — ch" + String(cmdChannel));
  }

  // Acknowledge the command so it isn't re-executed
  HTTPClient ackHttp;
  ackHttp.begin(BACKEND_URL_RELAY_ACK);
  ackHttp.setConnectTimeout(HTTP_TIMEOUT_MS); // Session 21 hotfix: bound TCP connect time
  ackHttp.setTimeout(HTTP_TIMEOUT_MS);
  ackHttp.addHeader("Content-Type", "application/json");
  ackHttp.addHeader("Authorization", "Bearer " + String(API_KEY));

  StaticJsonDocument<64> ackDoc;
  ackDoc["id"] = cmdId;
  String ackPayload;
  serializeJson(ackDoc, ackPayload);
  ackHttp.POST(ackPayload);
  ackHttp.end();
}
