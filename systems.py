import ColorSensor
from ColorSensor import TCS34725
from command_based import Subsystem


class Climber(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.motor = my_robot.spark_climber

    def active(self):
        self.motor.set(1)

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()


class FuelTank(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.intake_motor = my_robot.victor_intake
        self.my_robot = my_robot

        self.blender_motor = my_robot.victor_blender

    def intake_active(self):
        self.intake_motor.set(.5)

    def intake_inactive(self):
        self.intake_motor.set(0)

    def blender_active(self):
        js_left = self.my_robot.js_left
        self.blender_motor.set(js_left.getRawAxis(3))

    def blender_inactive(self):
        self.blender_motor.set(0)

    def default(self):
        self.intake_inactive()


class GearLifter(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.color_sensor = TCS34725()
        self.color_sensor.enable()

    def print_color(self):
        r, g, b, c = self.color_sensor.get_raw_data()
        print("R: {} G: {} B: {} C: {}".format(r, g, b, c))
        print("Color temp: {}".format(ColorSensor.calculate_color_temperature(r, g, b)))


class Shooter(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.motor = my_robot.talon_shooter

    def active(self):
        self.motor.set(.7)  # TODO feedback

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()
