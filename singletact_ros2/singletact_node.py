"""
A basic node that publishes the sensor values of a SingleTact Pressure sensor
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import serial
import time

# SingleTact Arduino protocol
I2C_ADDRESS = 0x04
READ = 0x01
REGISTER = 128
NBYTES = 6
BAUD = 115200

class SingleTactNode(Node):
    def __init__(self, sensor_rating: int, sensor_port: int, read_rate: int):
        super().__init__("SingleTact_node")

        self.read_rate = read_rate
        if self.read_rate > 100:
            self.get_logger().warning(f"SingleTact Read rate cannot surpass 100Hz! Clamping to 100Hz.")
            self.read_rate = 100

        self.sensor_rating = sensor_rating
        self.ser = serial.Serial(
            sensor_port,
            BAUD,
            timeout=0.5,
            rtscts=False,
            dsrdtr=False,
        )

        # Resets the Leonardo using DTR and waits 2 seconds.
        self.ser.dtr = True
        time.sleep(0.01)
        self.ser.dtr = False
        time.sleep(2)

        self.ser.reset_input_buffer()

        self.command_id = 0

        self.get_logger().info(f"Connected to {sensor_port}")
        self.get_logger().info(f"Reading SingleTact Sensor")

        self.publisher = self.create_publisher(Float32, 'singletact/newtons', 10)
        self.create_timer(1 / self.read_rate, self._publish)
                
    def _publish(self) -> None:
        msg = Float32()
        msg.data = self.convert_to_N(self.read_sensor())
        self.publisher.publish(msg)

    def convert_to_N(self, value: int) -> float:
        return (value*self.sensor_rating) / 512
 
    def read_sensor(self) -> int | None:
        """Returns the raw sensor value
        """
        #GenerateReadCommand()
        packet = bytes([
            0xFF, 0xFF, 0xFF, 0xFF,
            I2C_ADDRESS,
            100,             # timeout
            self.command_id,
            READ,
            REGISTER,
            NBYTES,
            0xFF,
            0xFE, 0xFE, 0xFE, 0xFE,
        ])

        expected_id = self.command_id
        self.command_id = (self.command_id + 1) & 0xFF

        self.ser.write(packet)
        self.ser.flush()

        # Arduino response:
        #
        # FF FF FF FF
        # I2C address
        # timeout
        # command ID
        # 4-byte timestamp
        # number of data bytes
        # data
        # FE FE FE FE
        #
        # For a 6-byte read the complete packet is 22 bytes.

        deadline = time.monotonic() + 0.5
        buffer = bytearray()

        while time.monotonic() < deadline:

            data = self.ser.read(self.ser.in_waiting or 1)

            if data:
                buffer.extend(data)

            # Find FF FF FF FF header
            while len(buffer) >= 4:

                if buffer[:4] != b"\xff\xff\xff\xff" and \
                buffer[:4] != b"\xaa\xaa\xaa\xaa":
                    buffer.pop(0)
                    continue

                # Need byte 11 to know the payload length.
                if len(buffer) < 12:
                    break

                payload_length = buffer[11]

                # Header/info + payload + footer
                packet_length = 16 + payload_length

                if len(buffer) < packet_length:
                    break

                response = bytes(buffer[:packet_length])
                del buffer[:packet_length]

                # Check footer
                if response[-4:] != b"\xfe\xfe\xfe\xfe":
                    continue

                # Check command ID
                if response[6] != expected_id:
                    continue

                # Check Arduino didn't report an I2C timeout
                if response[5] != 0:
                    raise RuntimeError("SingleTact reported an I2C timeout")

                payload = response[12:12 + payload_length]

                if len(payload) != 6:
                    continue

                # The C# implementation reads 6 bytes from register 128.
                #
                # payload[0:2] = iteration/sample number
                # payload[2:4] = reserved/other sensor data
                # payload[4:6] = actual 16-bit sensor value
                #
                # The C# code extracts bytes 8 and 9 of the complete
                # timestamp+payload array, i.e. payload[4] and payload[5].

                value = (payload[4] << 8) | payload[5]
                value -= 0xFF

                return value

        raise TimeoutError("No response from SingleTact")


def main():
    rclpy.init()

    try:
        n = SingleTactNode(45, "/dev/ttyACM2", 50)
        rclpy.spin(n)
    except KeyboardInterrupt:
        n.destroy_node()
        rclpy.shutdown

if __name__ == '__main__':
    main()
