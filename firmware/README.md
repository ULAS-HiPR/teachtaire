# Teachtaire Firmware

STM32F072 firmware for Teachtaire, the Ogma telemetry/GNSS board.

## Build Targets

```bash
pio run -e teachtaire_flight
pio run -e teachtaire_lora_tx
pio run -e teachtaire_lora_rx
```

Target purpose:

- `teachtaire_flight`: main GNSS + SX1272 telemetry firmware (`teachtaire_empty` remains a legacy alias).
- `teachtaire_lora_tx`: LoRa transmit test build.
- `teachtaire_lora_rx`: LoRa receive test build.

## Hardware Functions

- LoRa radio over SPI.
- MAX-M10S GNSS over UART.
- STM32F072 host MCU.
- ST-Link/SWD debug path.

## Radio Driver Note

The fitted radio is `SX1272`; firmware includes and instantiates `SX1272`. The hardware schematic naming `SX1261IMLTRT` is incorrect and must be corrected.

## Host Status Block

The firmware exposes a volatile `report` struct for Ogma Console. It includes:

- clock/GPIO/SPI/UART status,
- LoRa init/status/counters/RSSI/SNR,
- GNSS parse counters,
- GNSS fix, satellite count, latitude, longitude, altitude, velocity,
- low-level bus error/status words.

The firmware also exposes `ogma_board_identity` for board detection.

## Radio Telemetry

`include/ogma_radio_protocol.h` defines protocol v1. Packets use `OG` magic, protocol version/type, sequence, uptime, flags, payload, and CRC16-CCITT. CAN records remain raw so Ogma Console uses the canonical CAN definition CSV/header for decoding.

- Core: flight state, Kalman, barometer, acceleration at 5 Hz.
- Slow: power, Pleasc status, actuator command at 1 Hz.
- GPS: full latitude/longitude/altitude/velocity/fix at 1 Hz.
- Events: flight-state and Pleasc ACK/status changes immediately.
- Deep: rotating node heartbeats every 5 s.

## Setup

```bash
git clone git@github.com:ULAS-HiPR/teachtaire.git
cd teachtaire
git submodule update --init --recursive
```

## Libraries

- `braiteoiri`: radio and GNSS drivers.
- `comheadan`: SPI, UART, CAN support.
- STM32Cube HAL.
