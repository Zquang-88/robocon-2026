import time
from datetime import datetime
from pathlib import Path

import serial


COMMAND_PORT = "COM14"
COMMAND_BAUD = 115200
TELEMETRY_PORT = "COM5"
TELEMETRY_BAUD = 57600
MAX_RUN_SECONDS = 70.0
POINT_A_WAIT_LIMIT_SECONDS = 9.0


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def main() -> None:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = Path(__file__).with_name(f"auto-point-a-{stamp}.log")
    telemetry = serial.Serial(TELEMETRY_PORT, TELEMETRY_BAUD, timeout=0.01)
    command = serial.Serial(COMMAND_PORT, COMMAND_BAUD, timeout=0.01)
    command.dtr = True
    command_alive = True
    receive_buffer = bytearray()
    usb_buffer = bytearray()
    current_state = "UNKNOWN"
    state_started_at = time.monotonic()
    result = "TIMEOUT"

    try:
        time.sleep(0.5)
        telemetry.reset_input_buffer()
        command.reset_input_buffer()

        for frame in (
            "RESET",
            "MODE,AUTO",
            "FIELD,RED",
            "AUTO_MECH,SIM_4S",
            "AUTO_TELEM,ON_REQUEST",
        ):
            send(command, frame)
            time.sleep(0.15)

        send(command, "HEARTBEAT")
        send(command, "START_TEST")
        print("START_SENT=AUTO,RED,SIM_4S")

        started_at = time.monotonic()
        last_heartbeat = 0.0
        last_snapshot = 0.0
        with log_path.open("wb") as log_file:
            while time.monotonic() - started_at < MAX_RUN_SECONDS:
                now = time.monotonic()
                if now - last_heartbeat >= 0.10:
                    send(telemetry, "HEARTBEAT")
                    if command_alive:
                        try:
                            send(command, "HEARTBEAT")
                        except serial.SerialException:
                            command_alive = False
                            print("COM14_DROPPED_HEARTBEAT_CONTINUES_ON_COM5")
                    last_heartbeat = now
                if now - last_snapshot >= 1.0:
                    send(telemetry, "AUTO_TELEM,SNAPSHOT")
                    last_snapshot = now

                if command_alive:
                    try:
                        usb_data = command.read(2048)
                        if usb_data:
                            usb_buffer.extend(usb_data)
                            while b"\n" in usb_buffer:
                                raw, _, usb_buffer = usb_buffer.partition(b"\n")
                                line = raw.rstrip(b"\r").decode("ascii", errors="replace")
                                if line.startswith("USB_RX,") and not line.endswith("HEARTBEAT"):
                                    print(line)
                    except serial.SerialException:
                        command_alive = False
                        print("COM14_DROPPED_HEARTBEAT_CONTINUES_ON_COM5")

                data = telemetry.read(4096)
                if not data:
                    time.sleep(0.002)
                    continue
                log_file.write(data)
                log_file.flush()
                receive_buffer.extend(data)

                while b"\n" in receive_buffer:
                    raw, _, receive_buffer = receive_buffer.partition(b"\n")
                    line = raw.rstrip(b"\r").decode("ascii", errors="replace")
                    if line.startswith(("STATE,", "SAFETY,", "FAULT,", "ERR,", "ACK,AUTO_MECH")):
                        print(line)

                    if line.startswith("STATE,"):
                        current_state = line.split(",", 1)[1]
                        state_started_at = now
                        if current_state == "BU_TAM_B":
                            result = "POINT_A_PASSED_AND_REACHED_POINT_B_ALIGNMENT"
                            return
                        if current_state == "FAULT_STOP":
                            result = "FAULT_STOP"
                            return
                    if line.startswith("FAULT,"):
                        result = line
                        return
                    if current_state == "DOI_GAP_A" and now - state_started_at > POINT_A_WAIT_LIMIT_SECONDS:
                        result = "POINT_A_WAIT_EXCEEDED_9S"
                        return
    finally:
        try:
            try:
                send(telemetry, "STOP")
            except serial.SerialException:
                pass
            if command_alive:
                try:
                    send(command, "STOP")
                except serial.SerialException:
                    pass
            time.sleep(0.25)
        finally:
            try:
                command.close()
            except serial.SerialException:
                pass
            telemetry.close()
        print(f"RESULT={result}")
        print(f"LAST_STATE={current_state}")
        print(f"LOG={log_path}")
        print("STOP_SENT_COM14_COM5_CLOSED")


if __name__ == "__main__":
    main()
