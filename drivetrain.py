import wpilib
import math


def _sgn(x):
    return x/abs(x) if x != 0 else 0


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
        self.radius_turn(forward_power, radius * _sgn(turn_power))