# Teachtaire

Teachtaire is the telemetry and navigation board in the Ogma stack. It carries the LoRa radio path and GNSS receiver used to locate the vehicle and move useful flight data away from the stack.

## Role In Ogma

- Provides LoRa radio transmit/receive hardware.
- Provides GNSS position, satellite, altitude, and velocity data.
- Shares the common STM32F072 + CAN architecture with the other STM boards.
- Serves as the focused bench-test target for radio and GNSS bring-up.

## Firmware

Firmware lives in `firmware/`.

Useful environments:

```bash
cd firmware
pio run -e teachtaire_flight
pio run -e teachtaire_lora_tx
pio run -e teachtaire_lora_rx
```

## Ogma Console Support

Ogma Console can:

- identify Teachtaire over SWD using `ogma_board_identity`,
- build/flash the Teachtaire PlatformIO environments,
- read the live `report` SRAM block,
- display LoRa counters/status,
- display GNSS parse/fix/satellite/location fields,
- configure, flash, and verify the versioned radio schedule.

## Host-Visible Symbols

- `ogma_board_identity`
- `report`

## Notes

- Current firmware reports enough state for bench diagnosis.
- The fitted radio is confirmed as `SX1272`. The schematic naming an SX1261 is incorrect and must be corrected; firmware targets SX1272.
- CAN behavior should remain separate from SWD debug/control work.
- GNSS and LoRa status are published through both CAN diagnostics and the SWD `report` block.
- LoRa TX has timeout/reinit telemetry so a stuck radio path is visible in Ogma Console.
- Hardware watchdog starts immediately after clock setup; reset status, telemetry queue health, and radio config magic/schema/CRC are reported in `report` version 6.
- Radio protocol v1 carries raw CAN bundles with CRC16. Default schedule: rotating core flight/IMU data at 5 Hz, GPS and slow data at 1 Hz, events immediately, and deep heartbeat diagnostics at 0.2 Hz. Ogma Console can change the bounded schedule and seal it into `teachtaire_radio_config.h`.
- Groundstation receives the same protocol and bridges app-compatible JSON/CAN lines over USB.

## Dependency Lock

Use the exact shared-library pins in `../dependencies.lock.json`:

- `braiteoiri`: `ogma/flight-hardening`
- `comheadan`: `ogma/flight-hardening`

Ogma Console doctor fails a board when these submodule SHAs do not match the lock file.
