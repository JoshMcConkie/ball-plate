from ball_plate.config import SerialConfig
from ball_plate.serial_tools import SerialIO


class FakeSerial:
    def __init__(self) -> None:
        self._data = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self._data)

    def extend(self, data: bytes) -> None:
        self._data.extend(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._data[:size])
        del self._data[:size]
        return chunk


def test_fetch_values_buffers_partials_and_skips_malformed_lines():
    serial_io = SerialIO(
        SerialConfig(
            preferred_port="unused",
            baud_rate=1,
            timeout_s=0.0,
        )
    )
    fake_serial = FakeSerial()
    serial_io.ser = fake_serial

    fake_serial.extend(b"1 2 3")
    assert serial_io.fetch_values() is None

    fake_serial.extend(b" 4 5 6\nmalformed\n7 8 9 10 11 12\n")
    assert serial_io.fetch_values() == [7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
    assert serial_io._discarded_lines == 1
