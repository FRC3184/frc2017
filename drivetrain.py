import wpilib
import math
from robotpy_ext.common_drivers.navx.ahrs import AHRS

import mathutils


class drivetrain(wpilib.RobotDrive):
    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        if "robot_width" in kwargs.keys():
            self.robot_width = kwargs['robot_width']
        else:
            self.robot_width = 27.75 / 12

        if "max_radius" in kwargs.keys():
            self.max_turn_radius = kwargs['max_radius']
        else:
            self.max_turn_radius = 10

        self.ahrs = AHRS.create_i2c()

    def radius_turn(self, pow, radius):
        D = self.robot_width / 2
        Vo = pow
        Vi = Vo * (radius - D) / (radius + D)

        if radius > 0:
            self.setLeftRightMotorOutputs(Vo, Vi)
        else:
            self.setLeftRightMotorOutputs(Vi, Vo)

    def radius_drive(self, forward_power, turn_power):
        if abs(turn_power) < 0.05:
            self.setLeftRightMotorOutputs(forward_power, forward_power)
            return
        if abs(forward_power) < 0.05:
            self.setLeftRightMotorOutputs(turn_power, -turn_power)
            return
        turn_power = turn_power**1/3
        radius = self.max_turn_radius * (1 - abs(turn_power))
        self.radius_turn(forward_power, radius * mathutils.sgn(turn_power))

    def turn_to_angle(self, angle, allowable_error=2):
        err = angle - self.ahrs.getYaw()
        if abs(err) < allowable_error:
            self.arcadeDrive(0, 0)
            return True
        min_power = .3
        p = err / 180
        if abs(p) < min_power:
            p = mathutils.sgn(p) * min_power
        if abs(p) > 1:
            p = mathutils.sgn(p)
        self.arcadeDrive(0, -p)
        return False

    def get_heading(self):
        return self.ahrs.getYaw()

    def motion_profile_drive(self, points):
        pass

