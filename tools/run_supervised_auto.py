import time
from pathlib import Path

import serial


PORT = "COM5"
BAUD = 57600
MAX_RUN_SECONDS = 100.0
LOG_PATH = Path(__file__).with_name("auto_test_last.log")


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def main() -> None:
    port = serial.Serial(PORT, BAUD, timeout=0.02)
    print(f"{PORT}_OPEN")
    started = False
    result = "TIMEOUT"
    receive_buffer = bytearray()

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
        send(port, "START_TEST")
        started = True
        print("SENT START_TEST FIELD RED")

        started_at = time.monotonic()
        last_heartbeat = 0.0
        with LOG_PATH.open("wb") as log_file:
            while time.monotonic() - started_at < MAX_RUN_SECONDS:
                now = time.monotonic()
                if now - last_heartbeat >= 0.10:
                    send(port, "HEARTBEAT")
                    last_heartbeat = now

                data = port.read(1024)
                if not data:
                    time.sleep(0.005)
                    continue

                log_file.write(data)
                log_file.flush()
                receive_buffer.extend(data)
                while b"\n" in receive_buffer:
                    raw_line, _, receive_buffer = receive_buffer.partition(b"\n")
                    line = raw_line.rstrip(b"\r").decode("ascii", errors="backslashreplace")
                    if line.startswith(("ACK,", "ERR,", "FAULT,", "STATE,", "ESP_RX,", "AUTO,")):
                        print(line)

                    if line == "STATE,BRIDGE_ENTRY_STOP" or line == "ACK,AUTO_BRIDGE_ENTRY_COMPLETE":
                        result = "BRIDGE_ENTRY_STOP"
                        return
                    if line == "STATE,FAULT_STOP" or line.startswith("FAULT,") or line.startswith("ERR,AUTO_TEST"):
                        result = line
                        return
                    if started and line == "STATE,WAIT_START" and now - started_at > 2.0:
                        result = "UNEXPECTED_WAIT_START"
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
