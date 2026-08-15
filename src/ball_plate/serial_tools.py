import time

from serial import Serial
from serial.tools import list_ports

from ball_plate.config import SerialConfig
from ball_plate.state import ControlCommand

class SerialIO:
    def __init__(self, serial_config: SerialConfig):
        self.preferred_port = serial_config.preferred_port
        self.baud_rate = serial_config.baud_rate
        self.timeout_s = self.timeout_s
        self.ser: Serial | None = None
            
            

    def open_serial(self) -> Serial:
        """Open the serial connection to the ESP32.

        Tries the preferred port first, then falls back to auto-detecting any
        connected USB/ACM serial device (ESP32 boards commonly enumerate as
        /dev/ttyACM* rather than /dev/ttyUSB*). Raises a clear error if nothing
        suitable is found instead of a raw FileNotFoundError.
        """
        candidates = []
        available = [p.device for p in list_ports.comports()]

        if self.preferred_port in available:
            candidates.append(self.preferred_port)
        # Add any other connected serial-looking devices as fallbacks.
        candidates += [d for d in available
                    if d not in candidates and ("USB" in d or "ACM" in d)]
        # Include the preferred port even if it wasn't enumerated, as a last try.
        if self.preferred_port not in candidates:
            candidates.append(self.preferred_port)

        last_error = None
        for port in candidates:
            try:
                return Serial(port, self.baud_rate, timeout=self.timeout_s)
            except Exception as e:
                last_error = e
        
        raise RuntimeError(
            f"Could not open a serial port. Preferred '{self.preferred_port}' is "
            f"unavailable and no connected USB/ACM device worked. "
            f"Detected ports: {available or 'none'}. "
            f"Plug in the ESP32 (and check the cable), update SERIAL_PORT in "
            f"config.py to the correct device, or set SERIAL_ON = False to run "
            f"without hardware. Last error: {last_error}"
        )

    def begin(self):
        self.ser = self.open_serial()
        print("Serial connected on:", self.ser.port)
        time.sleep(2)
        banner = self.ser.readline().decode(errors='ignore').strip()
        print("Banner:", banner)
        

    def send_packet(self,control_cmd: ControlCommand | str)->bool:
        # build a packet
        try: # send/recieve from esp32 through serial
            assert self.ser is not None
            self.ser.write(control_cmd.encode('utf-8'))
            # print(packet)
        except AssertionError as e:
            print("Please begin serial using the .begin() method")
            return False
        except Exception as e:
            print("Serial write failed: ", e)
            return False
        return True

    def fetch_values(self)->list[float] | None:
        try: # send/recieve from esp32 through serial
            assert self.ser is not None
            echo = self.ser.readline().decode(errors="ignore").strip()
            if not echo:
                return None # no complete line available yet, not an error
            values = list(map(float, echo.split()))
            if len(values) != 6:
                raise ValueError(f"Expected 6 IMU values, got {len(values)}: {echo!r}")
            return values
        except AssertionError as e:
            print("Please begin serial using the .begin() method")
            return None
        except Exception as e:
            print("Serial read failed: ", e)
            return None
