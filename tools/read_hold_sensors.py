import time
import serial

port = serial.Serial("COM5", 57600, timeout=0.1)
lines = []
buffer = bytearray()
try:
    time.sleep(0.5)
    port.reset_input_buffer()
    port.write(b"TEST_LINE_STOP\n")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        buffer.extend(port.read(2048))
        while b"\n" in buffer:
            raw, _, remainder = buffer.partition(b"\n")
            buffer[:] = remainder
            line = raw.rstrip(b"\r").decode("ascii", errors="replace")
            if line.startswith(("LINE_HOLD", "LINE_STOP", "ERR", "FAULT")):
                lines.append(line)
finally:
    port.write(b"STOP\n")
    time.sleep(0.2)
    port.close()

print("\n".join(lines[-24:]))
print("STOP_SENT_COM5_CLOSED")
