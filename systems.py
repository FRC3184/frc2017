import ctre
import wpilib
import wpilib.interfaces

import ColorSensor
from ColorSensor import TCS34725
from command_based import Subsystem
from dashboard import dashboard2


class Climber(Subsystem):
    def __init__(self, my_robot, climber_motor):
        super().__init__(my_robot)
        self.motor = climber_motor

    def active(self, power):
        self.motor.set(power)

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

    def intake_reverse(self):
        self.intake_motor.set(-.5)

    def intake_inactive(self):
        self.intake_motor.set(0)

    def blender_active(self):
        if self.blender_motor.get() == 0:
            self.blender_motor.set(0.07)
        else:
            k = 0.1
            x = self.blender_motor.get()
            self.blender_motor.set(x + k*x*(0.5 - x))

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

        self.my_robot = my_robot
        self.motor = shooter_motor
        dashboard2.graph("Shooter current", shooter_motor.getOutputCurrent)

        source = wpilib.interfaces.PIDSource.from_obj_or_callable(self.motor.getSpeed)
        source.gerPIDSourceType = lambda: wpilib.interfaces.PIDSource.PIDSourceType.kRate  # FUCK wpilib
        self.controller = wpilib.PIDController(Kp=0.1, Ki=0.0, Kd=0.0, kf=3 / 18700,
                                               source=source, output=self.motor.set)
        self.controller.setContinuous(False)
        self.controller.setOutputRange(-1, 1)
        self.controller.setInputRange(0, 18700 / 3)
        self.motor.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def active(self):
        self.controller.enable()
        self.controller.setSetpoint(4500)

    def reverse(self):
        self.controller.disable()
        self.motor.set(-0.5)

    def inactive(self):
        self.controller.disable()
        self.motor.set(0)

    def default(self):
        self.inactive()
