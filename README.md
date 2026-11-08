# wmbus-testing

Standalone test kit for validating a wM-Bus T1-mode **receiver** over real RF,
using a *separate* ESP32 + SX126x board as the transmitter.

This repo is independent of `radugeo74/esphome-sx1262` (the project being
tested). It does not copy or modify any of that project's decode logic
(`t1_3of6.h`, `pure_dll_crc.h`). It only *reuses* that project's existing,
unmodified `sx126x` ESPHome component (which already exposes a public
`transmit_packet()` method) as an `external_components` source, the same way
any ESPHome user would reuse any third-party component.

## What's here

- **`wmbus_tx_encoder.h`** — a header-only C++ encoder that:
  1. Takes a CRC-*stripped* wM-Bus telegram (the format wmbusmeters itself
     stores in its own public test fixtures, e.g.
     `simulations/simulation_t1.txt`),
  2. Re-inserts the EN 13757-4 CRC blocks (Format A: 10+2, then 16+2 chunks),
  3. 3-of-6 symbol-encodes the result for T1-mode transmission.

  Every constant in it (the CRC polynomial, the 3-of-6 `code_of_nibble[]`
  table) is sourced from two independent, public places: the EN 13757-4
  standard's published CRC parameters, and `rtl-wmbus`
  (https://github.com/xaelsouth/rtl-wmbus, BSD-2-Clause) — **not** from the
  audited `esphome-sx1262` repo, so this encoder is not a self-fulfilling
  mirror of the code it's meant to test against.

  Two ready-to-send test payloads are included, copied verbatim from
  wmbusmeters' own public fixtures
  (https://github.com/wmbusmeters/wmbusmeters/blob/master/simulations/simulation_t1.txt):
  - `kLansenPuTelegramHex` — Lansen PU pulse counter, id `00010206`,
    `a_counter=4711`, `b_counter=1234`. 42 bytes after CRC re-insertion
    (even length — no padding ambiguity).
  - `kFhkvDataIIITelegramHex` — Techem FHKV data II/III, id `11776622`.
    61 bytes after CRC re-insertion (**odd** length — exercises the
    not-yet-verified-on-air padding path noted in the header; test this one
    *after* Lansen PU works).

- **`firmware/tx-lab.yaml`** — an ESPHome config for a *separate* ESP32+SX126x
  board whose only job is to transmit the two test telegrams above on button
  press. Radio parameters (frequency, bitrate, deviation, bandwidth, sync
  word) are copied from `esphome-sx1262`'s own
  `experimental/semtech-radio/heltec-sx126x-combined-lab.yaml`, so the TX
  board matches what the receiver under test expects to hear.

- **`firmware/secrets.yaml.example`** — placeholder for Wi-Fi/OTA secrets
  referenced by the YAML. Copy it to `firmware/secrets.yaml` and fill in your
  own values; `secrets.yaml` is gitignored so it never gets committed.

## How to use

### 1. Host-side sanity check (no hardware needed)

```sh
g++ -std=c++17 -DWMBUS_TX_ENCODER_TEST_MAIN -Wall -Wextra -x c++ \
    -o /tmp/tx_test wmbus_tx_encoder.h
/tmp/tx_test
```

This prints, for both test telegrams: the CRC-stripped input, the frame with
CRC blocks re-inserted, and the final 3-of-6 packed bytes ready for
`transmit_packet()`. For Lansen PU it also confirms the exact byte count
(42 → 63 bytes, 2-source-bytes-to-3-packed-bytes exactly, no padding
needed). For FHKV data III it prints a note that the trailing 12 bits were
padded — that's the one part of the encoder not yet confirmed against a real
on-air capture.

### 2. Flash the TX board

1. `cp firmware/secrets.yaml.example firmware/secrets.yaml` and fill in your
   Wi-Fi credentials.
2. Open `firmware/tx-lab.yaml` and **adjust the `spi:`/`sx126x:` pin
   assignments** to match your actual board wiring — the ones in the file
   are placeholders, not verified against any specific board. Everything
   *except* the pins (frequency, bitrate, deviation, bandwidth, sync word)
   is copied from the receiver project's own lab YAML, so leave those as-is.
3. `esphome run firmware/tx-lab.yaml` (or compile+flash however you normally
   do it).
4. In Home Assistant (or the ESPHome web UI), you'll see two buttons:
   "Send Lansen PU test telegram" and "Send FHKV data III test telegram".

### 3. Validate the receiver

On the *receiver* board (the one running `esphome-sx1262`), start with its
plain diagnostic YAML (logs raw received packets, no decode pipeline) to
confirm the frame is physically received before trusting the full decode→
sensor pipeline. Press "Send Lansen PU test telegram" on the TX board and
confirm the receiver logs a packet. Only after that works, move to the full
lab YAML with `meter_id: "00010206"` configured and confirm
`a_counter=4711` / `b_counter=1234` get published.

Test Lansen PU first (even-length frame, no padding ambiguity) before FHKV
data III (odd-length frame, exercises the unverified padding path).
