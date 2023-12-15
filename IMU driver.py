from pyb import I2C
import struct

def get_bits(byte_array, mask_length):
    """
    Extracts bits from a byte array using the specified mask length.

    Args:
        byte_array (list): List of bytes.
        mask_length (int): The number of bits in each mask.

    Returns:
        list: List of binary strings representing extracted bits.

    """
    result = []
    for byte in byte_array:
        bitmask = (2 ** mask_length) - 1
        for i in range(0, 8, mask_length):
            bits = (byte & bitmask) >> i
            buffer = f'{bits:0{mask_length}b}'
            bitmask = bitmask << mask_length
            result.append(buffer)
    return result

class imu:
    """
    Represents an Inertial Measurement Unit (IMU) class.

    Attributes:
        i2c (I2C): An I2C object for communication.
        
    Methods:
        __init__(): Initializes the IMU object and configures the IMU settings.
        cali_status(): Retrieves calibration status from the IMU.
        cali_coeff_read(): Reads calibration coefficients from the IMU.
        Euler_Angle(): Retrieves Euler angles from the IMU.

    """

    def __init__(self):
        """
        Initializes the IMU object and configures the IMU settings.

        """
        self.i2c = I2C(1, baudrate=200000)  # create and init as a controller
        self.i2c.mem_write(0b0001011, 40, 0x3D, timeout=100)  # Set IMU mode

        # Lines up axes to align with Romi
        self.i2c.mem_write(0x24, 40, 0x41, timeout=100)
        self.i2c.mem_write(0x06, 40, 0x42, timeout=100)
        self.i2c.mem_write(0b11111111, 40, 0x3B, timeout=100)  # Set IMU to radians mode
        # self.i2c.mem_write(0b00000000, 40, 0x3B, timeout=100)  # Set IMU to degrees mode

    def cali_status(self):
        """
        Retrieves calibration status from the IMU.

        Returns:
            list: List of binary strings representing calibration status.

        """
        status = self.i2c.mem_read(22, 40, 0x35)
        return get_bits(status, 2)

    def cali_coeff_read(self):
        """
        Reads calibration coefficients from the IMU.

        Returns:
            list: List of binary strings representing calibration coefficients.

        """
        read_data = self.i2c.mem_read(22, 40, 0x6A)
        return get_bits(read_data, 8)

    def Euler_Angle(self):
        """
        Retrieves Euler angle heading from the IMU.

        Returns:
            int: Unsigned Integer Heading, where 900 represents one radian, or 16 represents one degree.

        """
        Z_ang = self.i2c.mem_read(2, 40, 0x1A)
        Num = struct.unpack("H", Z_ang)
        Out = int(''.join(map(str, Num)))  # Convert tuple to int
        return Out  # Returns integer angle
   
  