import serial
import time


PORT = "COM5"
BAUD = 115200


def crc8(data):
    crc = 0
    for b in data:
        for _ in range(8):
            mix = (crc ^ b) & 1
            crc >>= 1
            if mix:
                crc ^= 0x8C
            b >>= 1
    return crc


class Bridge:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

    def close(self):
        self.ser.close()

    def _read_frame(self, deadline):
        prev = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if prev == b"\x5a" and b == b"\xa5":
                break
            prev = b
        else:
            return None
        hdr = b""
        while len(hdr) < 3 and time.time() < deadline:
            hdr += self.ser.read(1)
        if len(hdr) < 3:
            return None
        status, rcmd, rlen = hdr[0], hdr[1], hdr[2]
        body = b""
        while len(body) < rlen + 1 and time.time() < deadline:
            body += self.ser.read(rlen + 1 - len(body))
        if len(body) < rlen + 1:
            return None
        return status, rcmd, body[:rlen], body[rlen]

    def cmd(self, opcode, data=b"", limit=6.0):
        frame = bytes([0xAA, 0x55, opcode, len(data)]) + data
        frame += bytes([crc8(frame[2:])])
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        deadline = time.time() + limit
        while time.time() < deadline:
            fr = self._read_frame(deadline)
            if fr is None:
                return None
            if fr[0] == 0x01:
                continue
            return fr
        return None


def main():
    b = Bridge(PORT, BAUD)
    time.sleep(0.3)

    b.cmd(0x09, bytes([0xF0, 0x00, 16]))
    fr = b.cmd(0x0E, bytes([0, 16]))
    roms = [fr[2][i:i + 8] for i in range(0, len(fr[2]), 8)]
    eeprom = next(r for r in roms if r[0] == 0x2D)
    pio = next(r for r in roms if r[0] == 0x3A)
    print("DS2431:", eeprom.hex())
    print("DS2413:", pio.hex())

    print("\n-- DS2431 read memory (read-only) --")
    for addr in (0, 8, 16, 24):
        fr = b.cmd(0x30, eeprom + bytes([addr, 8]))
        print("  [%3d] status=%d %s" % (addr, fr[0], fr[2].hex(" ")))

    print("\n-- DS2431 read scratchpad --")
    fr = b.cmd(0x32, eeprom)
    print("  status=%d %s" % (fr[0], fr[2].hex(" ")))

    print("\n-- DS2413 read/write PIO --")
    for st in (0x00, 0x01, 0x02, 0x03, 0x00):
        wr = b.cmd(0x41, pio + bytes([st]))
        rd = b.cmd(0x40, pio)
        print("  write=%02X -> wr_status=%d rd_status=%d pio=%s" %
              (st, wr[0], rd[0], rd[2].hex(" ") if rd else "?"))

    print("\n-- config / stats --")
    fr = b.cmd(0x50)
    print("  CONFIG_GET status=%d len=%d" % (fr[0], len(fr[2])))
    fr = b.cmd(0x54)
    print("  STATS_GET  status=%d len=%d" % (fr[0], len(fr[2])))

    b.close()


if __name__ == "__main__":
    main()
