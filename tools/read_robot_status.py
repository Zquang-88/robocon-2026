import time
import serial

port = serial.Serial("COM5", 57600, timeout=0.05)
buffer = bytearray()
lines = []
try:
    time.sleep(0.5)
    port.reset_input_buffer()
    port.write(b"STATUS\n")
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        buffer.extend(port.read(2048))
        while b"\n" in buffer:
            raw, _, buffer = buffer.partition(b"\n")
            line = raw.rstrip(b"\r").decode("ascii", errors="replace")
            if line.startswith(("SYS,", "AUTO,", "HDG,", "TOF,", "TOFI,", "POSE,", "FAULT,", "STATE,", "LINE_HOLD", "LINE_CENTER")):
                lines.append(line)
finally:
    port.close()

print("\n".join(lines[-80:]))
print("COM5_CLOSED")
