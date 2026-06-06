import time

from serial import Serial

from ball_plate.state import IMUReading, ControlCommand


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
        values = list(map(float, echo.split()))

        if len(values) != 6:
            raise ValueError(f"Expected 6 IMU values, got {len(values)}: {echo!r}")

        imu_data = IMUReading(time.time(), *values)
        return imu_data
    except Exception as e:
        print("Serial read failed: ", e)
        return None
