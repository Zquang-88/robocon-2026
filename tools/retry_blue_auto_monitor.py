import time
from datetime import datetime
from pathlib import Path

import serial


PORT = "COM5"
BAUD = 57600
MAX_RUN_SECONDS = 180.0
MAX_STATE_SECONDS = 30.0
MAX_SILENCE_SECONDS = 4.0


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()
    print(f"TX,{command}", flush=True)


def main() -> None:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = Path(__file__).with_name(f"auto-blue-retry-{stamp}.log")
    port = serial.Serial(PORT, BAUD, timeout=0.02)
    rx = bytearray()
    current_state = "UNKNOWN"
    state_since = time.monotonic()
    last_rx = time.monotonic()
    result = "NOT_STARTED"
    started = False

    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        for command in ("RESET", "MODE,AUTO", "FIELD,BLUE", "AUTO_TELEM,ON_REQUEST"):
            send(port, command)
            time.sleep(0.15)

        # Wait for a fresh snapshot proving that BNO085 is initialized and valid.
        bno_deadline = time.monotonic() + 8.0
        last_snapshot = 0.0
        with log_path.open("wb") as log_file:
            while time.monotonic() < bno_deadline:
                now = time.monotonic()
                if now - last_snapshot >= 0.8:
                    send(port, "AUTO_TELEM,SNAPSHOT")
                    last_snapshot = now
                data = port.read(4096)
                if not data:
                    continue
                log_file.write(data)
                log_file.flush()
                rx.extend(data)
                while b"\n" in rx:
                    raw, _, rx = rx.partition(b"\n")
                    line = raw.rstrip(b"\r").decode("ascii", errors="replace")
                    if line.startswith(("HDG,", "STATE,", "FAULT,", "ERR,", "SAFETY,")):
                        print(line, flush=True)
                    if line.startswith("HDG,"):
                        parts = [part.strip() for part in line.split(",")]
                        if len(parts) >= 3 and parts[-2:] == ["1", "1"]:
                            result = "BNO_OK"
                            break
                if result == "BNO_OK":
                    break

            if result != "BNO_OK":
                result = "BNO_NOT_READY"
                return

            send(port, "START")
            started = True
            result = "RUNNING"
            run_started = time.monotonic()
            last_rx = run_started
            last_snapshot = 0.0

            while time.monotonic() - run_started < MAX_RUN_SECONDS:
                now = time.monotonic()
                if now - last_snapshot >= 1.0:
                    send(port, "AUTO_TELEM,SNAPSHOT")
                    last_snapshot = now

                data = port.read(4096)
                if data:
                    last_rx = now
                    log_file.write(data)
                    log_file.flush()
                    rx.extend(data)

                while b"\n" in rx:
                    raw, _, rx = rx.partition(b"\n")
                    line = raw.rstrip(b"\r").decode("ascii", errors="replace")
                    interesting = (
                        "STATE,", "SAFETY,", "FAULT,", "ERR,", "AUTO_BRIDGE",
                        "BRIDGE_", "CENTER_LINE", "LINE_", "ESP_", "ACK,FIELD",
                        "MECH_", "THA_", "POINT_", "POST_BRIDGE", "AUTO_SENSORS",
                    )
                    if line.startswith(interesting):
                        print(line, flush=True)

                    if line.startswith("STATE,"):
                        next_state = line.split(",", 1)[1].strip()
                        if next_state != current_state:
                            current_state = next_state
                            state_since = now
                    if line == "SAFETY,AUTO_COMPLETE" or current_state == "FINISH":
                        result = "AUTO_COMPLETE"
                        return
                    if line.startswith("FAULT,") or current_state == "FAULT_STOP":
                        result = line if line.startswith("FAULT,") else "FAULT_STOP"
                        return
                    if line.startswith("SAFETY,STOP_INPUT"):
                        result = "STOP_INPUT"
                        return

                if started and now - last_rx > MAX_SILENCE_SECONDS:
                    result = "TELEMETRY_SILENCE"
                    return
                if current_state not in ("UNKNOWN", "WAIT_START", "FINISH") and now - state_since > MAX_STATE_SECONDS:
                    result = f"STATE_TIMEOUT,{current_state}"
                    return
                time.sleep(0.002)

            result = "RUN_TIMEOUT"
    finally:
        try:
            send(port, "STOP")
            time.sleep(0.25)
        except serial.SerialException:
            pass
        port.close()
        print(f"RESULT={result}", flush=True)
        print(f"LAST_STATE={current_state}", flush=True)
        print(f"LOG={log_path}", flush=True)
        print("STOP_SENT_COM5_CLOSED", flush=True)


if __name__ == "__main__":
    main()
