import time
import struct
from typing import List, Tuple
from dataclasses import dataclass
from serial import Serial
from serial.tools import list_ports


@dataclass
class ColorObject:
    """Color object for LED control."""
    r: int
    g: int
    b: int
    brightness: int

    def __bytes__(self):
        return struct.pack('BBB', self.r, self.g, self.b , self.brightness)

    @classmethod
    def from_bytes(cls, data: bytes):
        r, g, b = struct.unpack('BBB', data)
        return cls(r, g, b)

    def __str__(self):
        return f'ColorObject(r={self.r}, g={self.g}, b={self.b} , brightness={self.brightness})'


# Battery Class. Send struct for battery status.
@dataclass
class BatteryObject:
    """Battery object for voltage"""

    voltage: float

    def __bytes__(self):
        return struct.pack('f', self.voltage)

    @classmethod
    def from_bytes(cls, data: bytes):
        voltage, = struct.unpack('f', data)
        return cls(voltage)

    def __str__(self):
        return f'BatteryObject(voltage={self.voltage})'


@dataclass
class SerialObject:
    """Serial object for serial communication."""
    port: str
    baudrate: int
    timeout: float

    def __enter__(self):
        self.ser = Serial(
            port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        return self.ser

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ser.close()


class SerialConnection:
    """
    SerialConnection is a class to manage serial connections.

    Attributes:
        port (str): The serial port to connect to.
        ser (Serial): The serial connection object.

    Methods:
        __init__(port: str):
            Initializes the SerialConnection with the specified port.

        connect():
            Connects to the serial port specified during initialization.
            Raises an exception if no device is found on the specified port.

        write(data: bytes):
            Writes data to the serial port.
            Raises an exception if the serial connection is not established.

        read(size: int) -> bytes:
            Reads data from the serial port.
            Raises an exception if the serial connection is not established.
    """

    def __init__(self, port: str):
        self.port = port
        self.ser = None

    def connect(self):
        """Method to connect to the serial port."""
        ports = list_ports.grep(self.port)
        try:
            comport = next(ports)
            self.ser = Serial(comport.device, baudrate=115200, timeout=0.1)
        except StopIteration:
            raise Exception(f"No device found on port {self.port}")

    def write(self, data: bytes):
        """Method to write data to the serial port."""
        if self.ser:
            self.ser.write(data)
        else:
            raise Exception("Serial connection not established.")

    def read(self, size: int) -> bytes:
        """Method to read data from the serial port."""
        if self.ser:
            return self.ser.read(size)
        else:
            raise Exception("Serial connection not established.")


# LED Class. Send struct for RGB values.
class LED:

    def __init__(self, ser: SerialConnection):
        self.ser = ser

    def set_color(self, color: ColorObject):
        """Method to set the color of the LED."""
        self.ser.write(bytes(color))

    def get_color(self) -> ColorObject:
        """Method to get the color of the LED."""
        data = self.ser.read(len(ColorObject))
        return ColorObject.from_bytes(data)


class Battery:

    def __init__(self, ser: SerialConnection):
        self.ser = ser

    def get_voltage(self) -> BatteryObject:
        """Method to get the battery voltage."""
        self.ser.write(b'v')
        data = self.ser.read(4)
        return BatteryObject.from_bytes(data)
    
# Test the classes

if __name__ == '__main__':
    with SerialObject(port='/dev/ttyS4', baudrate=115200, timeout=1) as ser:
        connection = SerialConnection(port='/dev/ttyS4')
        connection.ser = ser

        led = LED(connection)
        color = ColorObject(255, 255, 255 , 50)
        led.set_color(color)

        time.sleep(1)

        # Uncomment to test battery voltage reading
        # battery = Battery(connection)
        # voltage = battery.get_voltage()
        # print(voltage)