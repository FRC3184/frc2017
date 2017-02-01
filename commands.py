import mathutils
from command_based import Command
import ctre
import math


class TurnToAngleCommand(Command):
    def __init__(self, my_robot, angle):
        super().__init__(my_robot)
        self.result = False
        self.angle = angle

    def can_run(self):
        return not self.my_robot.drive.is_occupied

    def init(self):
        self.my_robot.drive.occupy()

    def is_finished(self):
        return self.result

    def finish(self):
        self.my_robot.drive.release()

    def run_periodic(self):
        self.result = self.my_robot.drive.turn_to_angle(self.angle)


class OpDriveCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.manually_finish = False

    def can_run(self):
        return not self.my_robot.drive.is_occupied

    def init(self):
        self.my_robot.drive.occupy()

    def is_finished(self):
        return not self.my_robot.isOperatorControl() or self.manually_finish

    def finish(self):
        self.my_robot.drive.release()
        self.manually_finish = False

    def run_periodic(self):
        js_left = self.my_robot.js_left
        js_right = self.my_robot.js_right
        self.my_robot.drive.arcadeDrive(-js_left.getY(), -js_right.getX())
        if js_left.getRawButton(1):
            self.manually_finish = True
            self.my_robot.cmd_queue.append(TurnToAngleCommand(self.my_robot, 0))
            self.my_robot.cmd_queue.append(self)


class MotionProfileDriveCommand(Command):
    def __init__(self, my_robot, dist, vel, acc, margin=2):
        super().__init__(my_robot)
        self.drive = my_robot.drive
        self.dist = dist
        self.vel = vel
        self.acc = acc
        self.margin = margin

        self.scale_factor = (math.pi * self.drive.wheel_diameter / 12)

    def can_run(self):
        return not self.drive.is_occupied

    def init(self):
        self.drive.occupy()

        vel_rpm = (self.vel / self.scale_factor) / 60
        acc_rpm = (self.acc / self.scale_factor) / 60 ** 2
        self.my_robot.talon_left.setMotionMagicCruiseVelocity(vel_rpm)
        self.my_robot.talon_right.setMotionMagicCruiseVelocity(vel_rpm)
        self.my_robot.talon_left.setMotionMagicAcceleration(acc_rpm)
        self.my_robot.talon_right.setMotionMagicAcceleration(acc_rpm)

        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.MotionMagic)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.MotionMagic)

        dist_revs = self.dist / self.scale_factor
        self.my_robot.talon_left.set(dist_revs)

    def is_finished(self):
        left_on = self.my_robot.talon_left.getClosedLoopError() / self.scale_factor < self.margin
        right_on = self.my_robot.talon_right.getClosedLoopError() / self.scale_factor < self.margin
        return left_on and right_on

    def finish(self):
        self.drive.release()
        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def run_periodic(self):
        pass


class MotionProfileRadiusDriveCommand(Command):
    def __init__(self, my_robot, radius, angle, vel, acc, margin=2):
        super().__init__(my_robot)
        self.drive = my_robot.drive
        self.radius = abs(radius)
        self.dir = mathutils.sgn(radius)
        self.angle = angle
        self.vel = vel
        self.acc = acc
        self.margin = margin

        self.scale_factor = (math.pi * self.drive.wheel_diameter / 12)
        self.wheel_ratio = (radius - self.drive.robot_width / 2) / (radius + self.drive.robot_width / 2)

    def can_run(self):
        return not self.drive.is_occupied

    def init(self):
        self.drive.occupy()

        vel_rpm_outer = (self.vel / self.scale_factor) / 60
        acc_rpm_outer = (self.acc / self.scale_factor) / 60 ** 2
        vel_rpm_inner = self.wheel_ratio * vel_rpm_outer
        acc_rpm_inner = self.wheel_ratio * acc_rpm_outer

        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.MotionMagic)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.MotionMagic)

        dist_revs_outer = self.angle * (self.radius + self.drive.robot_width) / self.scale_factor
        dist_revs_inner = self.angle * (self.radius - self.drive.robot_width) / self.scale_factor
        if self.dir == 1:
            outer_talon = self.my_robot.talon_left
            inner_talon = self.my_robot.talon_right
        else:
            outer_talon = self.my_robot.talon_right
            inner_talon = self.my_robot.talon_left
            
        outer_talon.setMotionMagicCruiseVelocity(vel_rpm_outer)
        outer_talon.setMotionMagicCruiseVelocity(vel_rpm_outer)
        outer_talon.setMotionMagicAcceleration(acc_rpm_outer)
        outer_talon.set(dist_revs_outer)
        
        inner_talon.setMotionMagicCruiseVelocity(vel_rpm_inner)
        inner_talon.setMotionMagicCruiseVelocity(vel_rpm_inner)
        inner_talon.setMotionMagicAcceleration(acc_rpm_inner)
        inner_talon.set(dist_revs_inner)

    def is_finished(self):
        left_on = self.my_robot.talon_left.getClosedLoopError() / self.scale_factor < self.margin
        right_on = self.my_robot.talon_right.getClosedLoopError() / self.scale_factor < self.margin
        return left_on and right_on

    def finish(self):
        self.drive.release()
        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def run_periodic(self):
        pass
