import threading

import ctre
import wpilib
import wpilib.interfaces

import ColorSensor
import control
import robot_time
from ColorSensor import TCS34725
from command_based import Subsystem
from control import RPMController
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
        self.blender_motor.set(0.75)

    def blender_inactive(self):
        self.blender_motor.set(0)

    def default(self):
        self.intake_inactive()


class GearLifter(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)

        self.grab = wpilib.DoubleSolenoid(0, 1)
        self.lift = wpilib.DoubleSolenoid(2, 3)

        self.arm_up_last = True
        self.grab_close_last = True

    def release_grab(self):
        self.grab.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.grab_close_last = False

    def close_grab(self):
        self.grab.set(wpilib.DoubleSolenoid.Value.kForward)
        self.grab_close_last = True

    def down(self):
        # arm down
        self.lift.set(wpilib.DoubleSolenoid.Value.kForward)
        self.arm_up_last = False

    def up(self):
        # arm up
        self.lift.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.arm_up_last = True

    def default(self):
        if self.arm_up_last:
            self.up()
        else:
            self.down()
        if self.grab_close_last:
            self.close_grab()
        else:
            self.release_grab()

    def print_color(self):
        pass


class Shooter(Subsystem):
    def __init__(self, my_robot, shooter_motor):
        super().__init__(my_robot)

        self.my_robot = my_robot
        self.motor = shooter_motor

        dashboard2.graph("Shooter current", shooter_motor.getOutputCurrent)

        target_rpm = 4100
        self.controller = control.RPMController(target_rpm, 6322, 0.00023, self.motor.getSpeed, self.motor.set)
        self.controller.start()
        dashboard2.graph("Error", self.controller.error)
        self.motor.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def active(self):
        self.controller.enabled = True

    def reverse(self):
        self.controller.enabled = False
        self.motor.set(-0.5)

    def inactive(self):
        self.controller.enabled = False
        self.motor.set(0)

    def set_target_rpm(self, target):
        self.controller.target_speed = target

    def get_target_rpm(self):
        return self.controller.target_speed

    def default(self):
        self.inactive()
