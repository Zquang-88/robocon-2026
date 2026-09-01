import time

import serial


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def main() -> None:
    port = serial.Serial("COM5", 57600, timeout=0.03)
    buffer = bytearray()
    print("COM5_OPEN")
    try:
        time.sleep(1.2)
        port.reset_input_buffer()
        for _ in range(3):
            send(port, "PING")
            time.sleep(0.1)
        send(port, "RESET")
        time.sleep(0.2)
        send(port, "TEST_LINE_CENTER")
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            data = port.read(1024)
            if not data:
                continue
            buffer.extend(data)
            while b"\n" in buffer:
                raw, _, buffer = buffer.partition(b"\n")
                line = raw.rstrip(b"\r").decode("ascii", errors="backslashreplace")
                if line.startswith(("ACK,", "ERR,", "SYS,", "TOF,", "LINE_CENTER")):
                    print(line)
    finally:
        send(port, "STOP")
        time.sleep(0.2)
        port.close()
        print("STOP_SENT_COM5_CLOSED")


if __name__ == "__main__":
    main()
