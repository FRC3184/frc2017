import threading

import ctre
import wpilib
import wpilib.interfaces

import ColorSensor
import robot_time
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

        self.grab = wpilib.DoubleSolenoid(0, 1)
        self.lift = wpilib.DoubleSolenoid(2, 3)

    def down(self):
        # Grab release, arm down
        self.grab.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.lift.set(wpilib.DoubleSolenoid.Value.kForward)

    def up(self):
        # Grab closed, arm up
        self.grab.set(wpilib.DoubleSolenoid.Value.kForward)
        self.lift.set(wpilib.DoubleSolenoid.Value.kReverse)

    def default(self):
        self.up()

    def print_color(self):
        pass


class Shooter(Subsystem):
    def __init__(self, my_robot, shooter_motor):
        super().__init__(my_robot)

        self.my_robot = my_robot
        self.motor = shooter_motor

        dashboard2.graph("Shooter current", shooter_motor.getOutputCurrent)

        class BangbangController:
            def __init__(self, target_speed, source, output):
                self.enabled = False
                self.target_speed = target_speed
                self.source = source
                self.output = output

            def run(self):
                while True:
                    if self.enabled:
                        err = self.error()
                        power = 4500 / 6322
                        power += 0.00023 * err
                        self.output(power)
                    robot_time.sleep(millis=10)

            def error(self):
                return self.target_speed - self.source()
        self.controller = BangbangController(4500, self.motor.getSpeed, self.motor.set)
        self.control_thread = threading.Thread(target=self.controller.run)
        self.control_thread.start()
        dashboard2.graph("Error", self.controller.error)
        dashboard2.graph("Output", self.motor.get)
        self.motor.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def active(self):
        self.controller.enabled = True

    def reverse(self):
        self.controller.enabled = False
        self.motor.set(-0.5)

    def inactive(self):
        self.controller.enabled = False
        self.motor.set(0)

    def default(self):
        self.inactive()
