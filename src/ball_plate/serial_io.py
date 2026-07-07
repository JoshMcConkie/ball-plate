import time

from serial import Serial
from serial.tools import list_ports

from ball_plate.state import IMUReading, ControlCommand


def open_serial(preferred_port: str, baud_rate: float, timeout: float = .001) -> Serial:
    """Open the serial connection to the ESP32.

    Tries the preferred port first, then falls back to auto-detecting any
    connected USB/ACM serial device (ESP32 boards commonly enumerate as
    /dev/ttyACM* rather than /dev/ttyUSB*). Raises a clear error if nothing
    suitable is found instead of a raw FileNotFoundError.
    """
    candidates = []
    available = [p.device for p in list_ports.comports()]

    if preferred_port in available:
        candidates.append(preferred_port)
    # Add any other connected serial-looking devices as fallbacks.
    candidates += [d for d in available
                   if d not in candidates and ("USB" in d or "ACM" in d)]
    # Include the preferred port even if it wasn't enumerated, as a last try.
    if preferred_port not in candidates:
        candidates.append(preferred_port)

    last_error = None
    for port in candidates:
        try:
            return Serial(port, baud_rate, timeout=timeout)
        except Exception as e:
            last_error = e

    raise RuntimeError(
        f"Could not open a serial port. Preferred '{preferred_port}' is "
        f"unavailable and no connected USB/ACM device worked. "
        f"Detected ports: {available or 'none'}. "
        f"Plug in the ESP32 (and check the cable), update SERIAL_PORT in "
        f"config.py to the correct device, or set SERIAL_ON = False to run "
        f"without hardware. Last error: {last_error}"
    )


def send_packet(control_cmd: ControlCommand, ser: Serial)->bool:
    # build a packet
    packet = control_cmd.get_cmd_str()
    try: # send/recieve from esp32 through serial
        ser.write(packet.encode('utf-8'))
        # print(packet)
    except Exception as e:
        print("Serial write failed: ", e)
        return False
    return True

def fetch_packet(ser: Serial)->IMUReading:
    try: # send/recieve from esp32 through serial
        echo = ser.readline().decode(errors="ignore").strip()
        if not echo:
            return None # no complete line available yet, not an error
        values = list(map(float, echo.split()))

        if len(values) != 6:
            raise ValueError(f"Expected 6 IMU values, got {len(values)}: {echo!r}")

        imu_data = IMUReading(time.time(), *values)
        return imu_data
    except Exception as e:
        print("Serial read failed: ", e)
        return None
