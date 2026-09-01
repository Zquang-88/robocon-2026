import time

import serial


def main() -> None:
    port = serial.Serial("COM5", 57600, timeout=0.05)
    print("COM5_OPEN")
    try:
        time.sleep(1.2)
        for _ in range(3):
            port.write(b"PING\n")
            time.sleep(0.1)

        port.write(b"RESET\n")
        time.sleep(0.3)
        port.write(b"ESP_TEST,POINT_A\n")
        print("SENT ESP_TEST,POINT_A")

        end_time = time.time() + 12.5
        while time.time() < end_time:
            data = port.read(512)
            if data:
                print(data.decode("ascii", errors="backslashreplace"), end="")
            time.sleep(0.01)
    finally:
        port.write(b"STOP\n")
        time.sleep(0.2)
        port.close()
        print("\nSTOP_SENT_COM5_CLOSED")


if __name__ == "__main__":
    main()
