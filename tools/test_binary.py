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
        # Align on 5A A5.
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

    def cmd(self, opcode, data=b"", limit=5.0):
        frame = bytes([0xAA, 0x55, opcode, len(data)]) + data
        frame += bytes([crc8(frame[2:])])
        self.ser.reset_input_buffer()
        self.ser.write(frame)

        deadline = time.time() + limit
        while time.time() < deadline:
            fr = self._read_frame(deadline)
            if fr is None:
                return None
            status, rcmd, payload, got_crc = fr
            if status == 0x01:  # BUSY: wait for the final response
                continue
            return fr
        return None


def rom_to_str(rom):
    return ":".join(f"{b:02X}" for b in rom)


def temp_from_payload(payload):
    if len(payload) < 2:
        return None
    v = (payload[0] << 8) | payload[1]
    if v & 0x8000:
        v -= 0x10000
    return v / 10.0


def main():
    b = Bridge(PORT, BAUD)
    time.sleep(0.3)

    print("OW_RESET     :", b.cmd(0x01))
    print("OW_SEARCH    :", b.cmd(0x09, bytes([0xF0, 0x00, 16])))

    fr = b.cmd(0x0E, bytes([0, 16]))
    if fr is None:
        print("OW_GET_DEVICES: no response")
        b.close()
        return
    st, cmd, body, crc = fr
    print("OW_GET_DEVICES status", st, "len", len(body))
    roms = [body[i:i + 8] for i in range(0, len(body), 8)]
    for r in roms:
        print("  ", rom_to_str(r), "family", hex(r[0]))

    ds = [r for r in roms if r[0] == 0x28]
    if not ds:
        print("no DS18B20 found")
        b.close()
        return

    rom = ds[0]
    print("\nDS18B20", rom_to_str(rom))
    print("TEMP_CONVERT :", b.cmd(0x20, rom, limit=3.0))
    print("TEMP_READ    :", b.cmd(0x21, rom, limit=3.0))

    fr = b.cmd(0x21, rom, limit=3.0)
    if fr:
        print("temperature  : %.1f C" % (temp_from_payload(fr[2]) or 0.0))

    b.close()


if __name__ == "__main__":
    main()
