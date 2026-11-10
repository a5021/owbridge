import serial
import time


PORT = "COM5"
BAUD = 115200


def read_all(ser, quiet=0.25, limit=2.0):
    """Read until the line has been quiet for `quiet` seconds or `limit` elapses."""
    start = time.time()
    last = start
    data = b""
    while time.time() - start < limit:
        n = ser.in_waiting
        if n:
            data += ser.read(n)
            last = time.time()
        elif time.time() - last > quiet:
            break
        else:
            time.sleep(0.01)
    return data


def main():
    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()

        commands = [
            "AT",
            "AT+VER",
            "AT+OWRST",
            "AT+OWSRCH",
            "AT+OWCNT",
            "AT+TEMP",
            "AT+STAT",
            "AT+DEVS",
        ]

        for cmd in commands:
            ser.write((cmd + "\r\n").encode())
            time.sleep(0.15)
            resp = read_all(ser, 2.0, 6.0)
            print("> " + cmd)
            print(resp.decode(errors="replace").strip())
            print("-" * 50)


if __name__ == "__main__":
    main()
