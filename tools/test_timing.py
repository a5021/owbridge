import serial
import time

ser = serial.Serial("COM5", 115200, timeout=0.2)
time.sleep(0.3)
ser.reset_input_buffer()

for cmd in ["AT+OWSRCH", "AT+TEMP", "AT+TEMP"]:
    ser.reset_input_buffer()
    t0 = time.time()
    ser.write((cmd + "\r\n").encode())
    data = b""
    while time.time() - t0 < 12:
        n = ser.in_waiting
        if n:
            data += ser.read(n)
            if b"\n" in data:
                break
        time.sleep(0.01)
    print("%-12s %.2f s  %s" % (cmd, time.time() - t0, data.decode(errors="replace").strip()))

ser.close()
