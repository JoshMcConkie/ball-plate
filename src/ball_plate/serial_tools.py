import math
import time

from serial import Serial
from serial.tools import list_ports

from ball_plate.config import SerialConfig
from ball_plate.state import ControlCommand


class SerialIO:
    def __init__(self, serial_config: SerialConfig):
        self.preferred_port = serial_config.preferred_port
        self.baud_rate = serial_config.baud_rate
        self.timeout_s = serial_config.timeout_s
        self.ser: Serial | None = None
        self._rx_buffer = bytearray()
        self._discarded_lines = 0
            
            

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
        self.ser.reset_input_buffer()
        self._rx_buffer.clear()


    def _report_discarded_line(self, line: str) -> None:
        self._discarded_lines += 1

        if self._discarded_lines <= 3 or self._discarded_lines % 100 == 0:
            print(
                f"Ignoring malformed serial line #{self._discarded_lines}: "
                f"{line!r}"
            )
        

    def send_packet(self,control_cmd: ControlCommand)->bool:
        # build a packet
        try: # send/recieve from esp32 through serial
            assert self.ser is not None
            self.ser.write(control_cmd.encode('utf-8'))
            # print(packet)
        except AssertionError:
            print("Please begin serial using the .begin() method")
            return False
        except Exception as e:
            print("Serial write failed: ", e)
            return False
        return True

    def fetch_values(self)->list[float] | None:
        try: # send/recieve from esp32 through serial
            assert self.ser is not None
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self._rx_buffer.extend(chunk)
            newest_values: list[float] | None = None

            while True:
                newline_index = self._rx_buffer.find(b'\n')
                if newline_index == -1:
                    break
                raw_line = bytes(self._rx_buffer[:newline_index + 1])
                del self._rx_buffer[:newline_index + 1]
                line = raw_line.decode(errors="ignore").strip()
                if not line:
                    continue
                fields = line.split()
                if len(fields) != 6:
                    self._report_discarded_line(line)
                    continue

                try:
                    values = [float(field) for field in fields]

                except ValueError:
                    self._report_discarded_line(line)
                    continue

                if not all(math.isfinite(value) for value in values):
                    self._report_discarded_line(line)
                    continue

                newest_values = values

            if len(self._rx_buffer) > 4096:
                self._rx_buffer.clear()
                print("Serial receive buffer reset: no newline within 4096 bytes")

            return newest_values

        except Exception as exc:
            print("Serial read failed:", exc)
            return None
