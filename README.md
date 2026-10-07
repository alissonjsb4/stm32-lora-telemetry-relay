# stm32-lora-telemetry-relay

Long-range telemetry link for an RS41 radiosonde, built on STM32WL55 boards (NUCLEO-WL55JC1). The sonde sends its data over serial to a field node, which validates each frame and relays it over LoRa at 915 MHz; a base station receives it, decodes it and prints it to a PC. A repeater node for longer paths is in progress.

## Usage

Hardware: two NUCLEO-WL55JC1 boards (three with the repeater), 915 MHz antennas, and an RS41 or any source sending the telemetry frame over serial at 19200 baud.

1. Import `src/Field_Node/` and `src/Base_Station/` (and `src/Repeater_Node/` if used) into STM32CubeIDE and build.
2. Flash each firmware to its board.
3. Open a serial terminal on the base station at 115200 baud, or run `python teste.py /dev/ttyACM0` (needs pyserial), to see the decoded telemetry: packet ID, position, altitude, voltage, temperature and GPS status.

## How it works

- Field node: UART reception by DMA in circular mode, so the CPU never blocks; a state machine finds each frame by its sync word (`0xAA`) and checks the checksum before transmitting over LoRa.
- Base station: interrupt-driven continuous receive; it decodes the binary payload and formats it for the PC over USART2.
- Repeater node: receives a packet and sends it again unchanged, without decoding it.
- `src/common/protocol.h`, shared by all nodes, holds the telemetry payload, the LoRa settings and a command format (read, write, execute, request) for remote control of the sonde.
- `src/UHF_LoRa/` is the sonde side: a fork of the RS41HUP amateur-radio firmware (GPL v2), with its upstream README.

| LoRa parameter | Value |
|---|---|
| Frequency | 915.0 MHz |
| Spreading factor | 12 |
| Bandwidth | 125 kHz |
| Coding rate | 4/8 |
| TX power | 22 dBm |

## Results

End-to-end link validated on the bench in August 2025: sonde frames passed the checksum, were relayed and were decoded at the base with RSSI -50 dBm and SNR 9. Full write-up, in Portuguese, in `docs/`.

## Notes

- A high spreading factor with coding rate 4/8 trades data rate for range and robustness; a short telemetry payload doesn't need bandwidth. The bench result above predates the move from SF 10 to SF 12 in January 2026.
- Radio sleep on the field node is still pending for battery operation.
- The repeater is an MVP: it relays packets as received, and the call that would decode them on that node is still commented out.

## Layout

    src/Field_Node/      transmitter: UART/DMA capture, validation state machine, LoRa TX
    src/Base_Station/    receiver: interrupt-driven LoRa RX, decoding, USART2 to the PC
    src/Repeater_Node/   repeater (MVP)
    src/common/          protocol.h shared by all nodes
    src/UHF_LoRa/        RS41 firmware fork (RS41HUP, GPL v2)
    teste.py             serial console for the base station
    docs/                technical report (PDF, Portuguese)

## Authors

Alisson Jaime Sales Barros and Danilo Mota Alencar Filho, with contributions from [@penafortemarco](https://github.com/penafortemarco) (repeater node, shared protocol header).
