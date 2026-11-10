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


def s16(payload):
    v = (payload[0] << 8) | payload[1]
    return v - 0x10000 if v & 0x8000 else v


def main():
    b = Bridge(PORT, BAUD)
    time.sleep(0.3)

    print("OW_SEARCH:", b.cmd(0x09, bytes([0xF0, 0x00, 16])))
    fr = b.cmd(0x0E, bytes([0, 16]))
    roms = [fr[2][i:i + 8] for i in range(0, len(fr[2]), 8)]
    ds = [r for r in roms if r[0] == 0x28]
    rom = ds[0]

    print("TEMP_DETECT_P:", b.cmd(0x29, rom))

    for parasite in (1, 0):
        print("\n--- SET_POWER parasite=%d ---" % parasite)
        print("SET_POWER:", b.cmd(0x0C, bytes([parasite])))
        print("TEMP_CONVERT:", b.cmd(0x20, b"", limit=3.0))
        fr = b.cmd(0x22, b"", limit=6.0)
        if fr:
            st, cmd, body, crc = fr
            temps = [s16(body[i:i + 2]) / 10.0 for i in range(0, len(body) - 1, 2)]
            print("TEMP_READ_ALL:", temps)

    b.close()


if __name__ == "__main__":
    main()
