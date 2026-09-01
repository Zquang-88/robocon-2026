import time
from pathlib import Path

import serial


LOG_PATH = Path(__file__).with_name("bridge_test_last.log")


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def main() -> None:
    port = serial.Serial("COM5", 57600, timeout=0.02)
    result = "TIMEOUT"
    buffer = bytearray()
    print("COM5_OPEN")
    try:
        time.sleep(1.2)
        port.reset_input_buffer()
        for _ in range(3):
            send(port, "PING")
            time.sleep(0.1)
        for command in ("RESET", "MODE,AUTO", "FIELD,RED"):
            send(port, command)
            time.sleep(0.25)
        send(port, "HEARTBEAT")
        send(port, "START_BRIDGE_TEST")
        print("SENT START_BRIDGE_TEST FIELD RED")

        start = time.monotonic()
        last_heartbeat = 0.0
        with LOG_PATH.open("wb") as log_file:
            while time.monotonic() - start < 45.0:
                now = time.monotonic()
                if now - last_heartbeat >= 0.1:
                    send(port, "HEARTBEAT")
                    last_heartbeat = now
                data = port.read(1024)
                if not data:
                    time.sleep(0.005)
                    continue
                log_file.write(data)
                log_file.flush()
                buffer.extend(data)
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    line = raw.rstrip(b"\r").decode("ascii", errors="backslashreplace")
                    if line.startswith(("ACK,", "ERR,", "FAULT,", "STATE,", "AUTO,")):
                        print(line)
                    if line == "STATE,BRIDGE_ENTRY_STOP" or line == "ACK,AUTO_BRIDGE_ENTRY_COMPLETE":
                        result = "BRIDGE_ENTRY_STOP"
                        return
                    if line == "STATE,FAULT_STOP" or line.startswith("FAULT,") or line.startswith("ERR,AUTO_TEST"):
                        result = line
                        return
    finally:
        try:
            send(port, "STOP")
            time.sleep(0.2)
        finally:
            port.close()
        print(f"RESULT={result}")
        print("STOP_SENT_COM5_CLOSED")
        print(f"LOG={LOG_PATH}")


if __name__ == "__main__":
    main()
