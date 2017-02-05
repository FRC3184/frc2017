import wpilib

ADDRESS = 0x29
COMMAND_BIT = 0x80
ENABLE = 0x00


class TCS34725:
    def __init__(self):
        self.i2c = wpilib.I2C(wpilib.I2C.Port.kOnboard, ADDRESS)

    def write8(self, reg, value):
        self.i2c.write(COMMAND_BIT | reg, value & 0xFF)

    def read8(self, reg):
        return self.i2c.read(COMMAND_BIT | reg, 8)