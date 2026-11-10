# Host-side test tools

Python scripts that drive the bridge over its selected host interface. They
were used for the bench validation described in the top-level `README.md`.

## Requirements

- Python 3.8+
- `pyserial`

```
pip install pyserial
```

## Configuration

Edit `PORT` and `BAUD` at the top of each script. The bench used `COM5` at
`115200` (USART variant). For the UART build, `AT` commands and the binary
frame share the same line; the scripts send one or the other.

## Scripts

| Script | What it does |
|--------|--------------|
| `test_uart.py` | Sends the AT command set and prints the responses. |
| `test_binary.py` | Binary protocol: reset, search, get devices, per-device convert/read. |
| `test_all.py` | Broadcast convert + read all temperatures, raw scratchpad of the first DS18B20. |
| `test_power.py` | `TEMP_DETECT_P` then convert/read with parasite on and off. |
| `test_devices.py` | DS2431 memory/scratchpad read, DS2413 PIO write/read, config/stats. |
| `test_timing.py` | Measures the AT command response time. |

## Notes

- The binary protocol is `AA 55 [cmd][len][data...] [crc8]` and the response is
  `5A A5 [status][cmd][len][data...] [crc8]` (Dallas/Maxim CRC-8, poly 0x8C).
  The scripts skip `BUSY` responses and wait for the final one.
- `AT+TEMP` takes about one second: it runs a broadcast Convert T (750 ms at
  12-bit) and then reads every DS18B20 scratchpad.
- The SPI and I2C builds need a corresponding master; these scripts only cover
  the UART transport. Use the binary frame directly for the other transports.
