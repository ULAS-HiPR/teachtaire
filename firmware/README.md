# Teachtaire

Teachtaire is the primary comms board. It has LoRa radio and gnss for finding the rocket during/after launch.

# Set-Up

```bash
git clone git@github.com:ULAS-HiPR/teachtaire.git
cd teachtaire
git submodule update --init --recursive
```

# Libraries used:

## From Braiteoiri:

- Radio: SX1261
- GNSS
- LED

## From Comheadan:

- CAN
- SPI
- UART

## Other:

- FreeRTOS