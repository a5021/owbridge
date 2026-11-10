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


def rom_str(rom):
    return ":".join(f"{b:02X}" for b in rom)


def s16(payload):
    v = (payload[0] << 8) | payload[1]
    return v - 0x10000 if v & 0x8000 else v


def main():
    b = Bridge(PORT, BAUD)
    time.sleep(0.3)

    print("OW_RESET:", b.cmd(0x01))
    print("OW_SEARCH:", b.cmd(0x09, bytes([0xF0, 0x00, 16])))

    fr = b.cmd(0x0E, bytes([0, 16]))
    st, cmd, body, crc = fr
    roms = [body[i:i + 8] for i in range(0, len(body), 8)]
    ds = [r for r in roms if r[0] == 0x28]
    print("devices:", len(roms), "DS18B20:", len(ds))

    print("\n-- broadcast convert + read all --")
    print("TEMP_CONVERT:", b.cmd(0x20, b"", limit=3.0))
    fr = b.cmd(0x22, b"", limit=6.0)
    if fr:
        st, cmd, body, crc = fr
        print("TEMP_READ_ALL status", st, "len", len(body))
        for i in range(0, len(body) - 1, 2):
            t = s16(body[i:i + 2])
            print("  sensor %d: %.1f C" % (i // 2, t / 10.0))

    print("\n-- raw scratchpad of first DS18B20 --")
    rom = ds[0]
    print("ROM:", rom_str(rom))
    print("MATCH+0xBE:", b.cmd(0x06, rom + bytes([0xBE])))
    fr = b.cmd(0x05, bytes([9]))
    if fr:
        print("raw scratchpad:", fr[2].hex(" "))
        raw = fr[2]
        if len(raw) >= 9:
            print("  temp code: 0x%04X" % ((raw[1] << 8) | raw[0]))
            print("  TH=%02X TL=%02X cfg=%02X" % (raw[2], raw[3], raw[4]))
            print("  crc=%02X (calc %02X)" % (raw[8], crc8(raw[:8])))

    b.close()


if __name__ == "__main__":
    main()
