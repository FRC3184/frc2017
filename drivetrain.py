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
        if "robot_width" in kwargs.keys():
            self.robot_width = kwargs['robot_width']
        else:
            self.robot_width = 29.25 / 12

        if "max_radius" in kwargs.keys():
            self.max_turn_radius = kwargs['max_radius']
        else:
            self.max_turn_radius = 10

        if "wheel_diameter" in kwargs.keys():
            self.wheel_diameter = kwargs['wheel_diameter']
        else:
            self.wheel_diameter = 4
        if "max_speed" in kwargs.keys():
            self.max_speed = kwargs['max_speed']
        else:
            self.max_speed = 13

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
            # self.setLeftRightMotorOutputs(turn_power * power_factor, -turn_power * power_factor, velocity=velocity)
            self.setLeftRightMotorOutputs(0, 0, velocity=velocity)
            return
        turn_power = mathutils.valroot(turn_power, 3)
        radius = self.max_turn_radius * (1 - abs(turn_power))
        self.radius_turn(forward_power * power_factor,
                         radius * mathutils.sgn(turn_power), # * mathutils.sgn(forward_power),
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
            ratio = 60 * self.max_speed / (math.pi * 4 * 12)  # max speed in rpm
            leftOutput *= ratio
            rightOutput *= ratio
            self.rearLeftMotor.set(leftOutput)
            self.rearRightMotor.set(-rightOutput)  # TODO make this apply to both motors
        else:
            super().setLeftRightMotorOutputs(leftOutput, rightOutput)

