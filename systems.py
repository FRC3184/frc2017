import wpilib

import ColorSensor
from ColorSensor import TCS34725
from command_based import Subsystem


class Climber(Subsystem):
    def __init__(self, my_robot, climber_motor):
        super().__init__(my_robot)
        self.motor = climber_motor

    def active(self):
        self.motor.set(1)

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()


class FuelTank(Subsystem):
    def __init__(self, my_robot, intake_motor, blender_motor):
        super().__init__(my_robot)
        self.intake_motor = intake_motor
        self.my_robot = my_robot

        self.blender_motor = blender_motor

    def intake_active(self):
        self.intake_motor.set(.5)

    def intake_inactive(self):
        self.intake_motor.set(0)

    def blender_active(self):
        if self.blender_motor.get() == 0:
            self.blender_motor.set(0.07)
        else:
            k = 0.1
            x = self.blender_motor.get()
            self.blender_motor.set(x + k*x*(1-x))

    def blender_inactive(self):
        self.blender_motor.set(0)

    def default(self):
        self.intake_inactive()


class GearLifter(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)

        if wpilib.hal.HALIsSimulation():
            return
        self.color_sensor = TCS34725()
        self.color_sensor.enable()

    def print_color(self):
        r, g, b, c = self.color_sensor.get_raw_data()
        print("R: {} G: {} B: {} C: {}".format(r, g, b, c))
        print("Color temp: {}".format(ColorSensor.calculate_color_temperature(r, g, b)))


class Shooter(Subsystem):
    def __init__(self, my_robot, shooter_motor):
        super().__init__(my_robot)
        self.motor = shooter_motor

    def active(self):
        self.motor.set(.7)  # TODO feedback

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()
