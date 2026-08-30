# PROJECT R.E.A.P.

**Repurposed Electronics for Automated Power-safety**

Source code for an edge-to-smartphone energy monitoring and electrical safety system, built around an ESP32 microcontroller and an idle smartphone repurposed as the backend server.

This repository accompanies the research paper of the same name. Chapter VI of that paper explains the algorithms reproduced here; this repository holds the complete files.

---

## What the system does

An ESP32 reads current and voltage on a monitored appliance circuit, computes power, accumulated energy, cost and carbon locally, and evaluates a two-layer safety threshold. When a threshold is exceeded it drives a relay to disconnect the appliance — without any network round trip, so the safety function does not depend on WiFi.

Readings are transmitted over WiFi to a Node.js server running under Termux on a repurposed Tecno Pova 4, which stores them in SQLite and serves a web dashboard.

### Two-layer threshold

- **Layer 1** — a fixed safety floor of 250 V and 1500 W per channel, always active, including during training and immediately after a reboot.
- **Layer 2** — a per-appliance threshold learned as 1.20 × the maximum wattage observed during a 30-minute training window, persisted to non-volatile storage so it survives restarts of either device.

---

## Repository contents

| Path | Description | Lines |
|---|---|---|
| `firmware/reap_firmware_v06.ino` | ESP32 firmware — sensing, calculation, threshold evaluation, relay control | 1,211 |
| `backend/server.js` | Node.js + Express server — REST API, SQLite schema, relay command queue, gap aggregation | 686 |
| `dashboard/dashboard.html` | Single-file web dashboard — six tabs, Tabler CSS + Chart.js | 1,925 |
| `docs/circuit-schematic.png` | Channel 1 signal conditioning, sensing and relay wiring |
| `docs/system-architecture.png` | System architecture and data flow |
| `docs/conceptual-framework.png` | Conceptual framework |

---

## Hardware

- ESP32-WROOM-32 development board
- SCT-013-030 clamp-type current transformer (30 A / 1 V, internal burden resistor)
- ZMPT101B voltage sensor module, supplied at **3.3 V** — 5 V would exceed the ADC input tolerance
- 4-channel relay module, 5 V, active-LOW, wired **normally closed** so an unpowered controller leaves the appliance connected
- Bias network: two 10 kΩ resistors forming a 1.65 V divider, with a 10 µF decoupling capacitor

The current transformer bridges the bias midpoint and the ADC pin. It is a non-contact sensor and makes no electrical connection to the conductor it measures.

---

## Calibration constants

These values were determined against reference instruments and are specific to this build:

```
SCT_CAL      = 7.75      // current transformer
ZMPT_CAL[0]  = 0.6036    // voltage sensor, channel 1
NOISE_FLOOR_A[0] = 0.300 // A, quadrature-subtracted
```

Two notes for anyone reproducing this:

1. **The measurement library assumes a 10-bit ADC.** The ESP32 is 12-bit. Without correcting for this, current reads roughly 4× high.
2. **RMS current cannot average to zero**, so an unloaded channel shows a standing phantom current. The firmware subtracts the noise floor in quadrature, which returns a true zero when idle. This sets a limit of detection of 0.30 A, about 66 W.

---

## Scope

The system was built and evaluated as a **single instrumented channel** monitoring one appliance. The architecture, firmware and backend provide for three channels, but only one current transformer was obtained within the study period. Channels 2 and 3 are present in the code and report zero rather than fabricated values.

---

## Redactions

Network credentials and the backend API key have been replaced with placeholders:

```
const char* SSID     = "<network SSID redacted>";
const char* PASSWORD = "<password redacted>";
const char* API_KEY  = "<API key redacted>";
```

Everything else is reproduced exactly as deployed.

---

## Licence

Released for academic review and reference.
