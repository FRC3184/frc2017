import wpilib
import math
from robotpy_ext.common_drivers.navx.ahrs import AHRS

import mathutils
from command_based import Subsystem
from dashboard import dashboard2


class Drivetrain(wpilib.RobotDrive, Subsystem):
    def __init__(self, *args, **kwargs):
        wpilib.RobotDrive.__init__(self, *args)
        Subsystem.__init__(self)
        self.robot_width = kwargs.pop("robot_width", 29.25 / 12)
        self.max_turn_radius = kwargs.pop("max_radius", 10)
        self.wheel_diameter = kwargs.pop("wheel_diameter", 4)
        self.max_speed = kwargs.pop("max_speed", 13)

        self.ahrs = AHRS.create_i2c()

        dashboard2.graph("Heading", lambda: self.get_heading())

    def radius_turn(self, pow, radius, velocity=False):
        D = self.robot_width / 2
        turn_dir = mathutils.sgn(radius)
        radius = abs(radius)
        Vo = pow
        Vi = Vo * (radius - D) / (radius + D)

        if turn_dir > 0:
            self.setLeftRightMotorOutputs(Vo, Vi, velocity=velocity)
        else:
            self.setLeftRightMotorOutputs(Vi, Vo, velocity=velocity)

    def radius_drive(self, forward_power, turn_power, power_factor, velocity=False):
        if abs(turn_power) < 0.05:
            self.setLeftRightMotorOutputs(forward_power * power_factor, forward_power * power_factor, velocity=velocity)
            return
        if abs(forward_power) < 0.05:
            self.setLeftRightMotorOutputs(0, 0, velocity=velocity)
            return
        turn_power = mathutils.signed_power(turn_power, 1/3)
        radius = self.max_turn_radius * (1 - abs(turn_power))
        self.radius_turn(forward_power * power_factor,
                         radius * mathutils.sgn(turn_power),
                         velocity=velocity)

    def turn_to_angle(self, angle, allowable_error=2):
        err = angle - self.get_heading()
        if abs(err) < allowable_error:
            self.arcadeDrive(0, 0)
            return True
        min_power = .6
        p = err / 200
        if abs(p) < min_power:
            p = mathutils.sgn(p) * min_power
        if abs(p) > 1:
            p = mathutils.sgn(p)
        self.arcadeDrive(0, -p)
        return False

    def arcade_velocity(self, move, rotate):
        if move > 0.0:
            if rotate > 0.0:
                leftMotorSpeed = move - rotate
                rightMotorSpeed = max(move, rotate)
            else:
                leftMotorSpeed = max(move, -rotate)
                rightMotorSpeed = move + rotate
        else:
            if rotate > 0.0:
                leftMotorSpeed = -max(-move, rotate)
                rightMotorSpeed = move + rotate
            else:
                leftMotorSpeed = move - rotate
                rightMotorSpeed = -max(-move, -rotate)
        self.setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed, velocity=True)

    def get_heading(self):
        return self.ahrs.getYaw()

    def default(self):
        self.setLeftRightMotorOutputs(0, 0)

    def setLeftRightMotorOutputs(self, leftOutput, rightOutput, velocity=False):
        if velocity:
            ratio = self.get_fps_rev_ratio()  # max speed in rpm
            leftOutput *= ratio
            rightOutput *= ratio
            if self.frontLeftMotor:
                self.frontLeftMotor.set(leftOutput)
            self.rearLeftMotor.set(leftOutput)
            if self.frontRightMotor:
                self.frontRightMotor.set(rightOutput)
            self.rearRightMotor.set(-rightOutput)

            self.feed()
        else:
            super().setLeftRightMotorOutputs(leftOutput, rightOutput)

    def get_fps_rev_ratio(self):
        return 60 * self.max_speed / (math.pi * 4 * 12)

