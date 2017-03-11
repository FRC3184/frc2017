import wpilib

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


class OpFuelTankCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)

    def can_run(self):
        return not self.my_robot.fueltank.is_occupied

    def init(self):
        self.my_robot.fueltank.occupy()

    def is_finished(self):
        return not self.my_robot.isOperatorControl()

    def finish(self):
        self.my_robot.fueltank.release()

    def run_periodic(self):
        gamepad = self.my_robot.gamepad
        fueltank = self.my_robot.fueltank
        if gamepad.getBButton():
            fueltank.blender_active()
        else:
            fueltank.blender_inactive()
        if gamepad.getBumper(wpilib.GenericHID.Hand.kLeft):
            fueltank.intake_active()
        elif gamepad.getXButton():
            fueltank.intake_reverse()
        else:
            fueltank.intake_inactive()


class OpGearCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)

    def can_run(self):
        return not self.my_robot.gear_lifter.is_occupied

    def init(self):
        self.my_robot.gear_lifter.occupy()

    def is_finished(self):
        return not self.my_robot.isOperatorControl()

    def finish(self):
        self.my_robot.gear_lifter.release()

    def run_periodic(self):
        gamepad = self.my_robot.gamepad
        gear_lifter = self.my_robot.gear_lifter

        if gamepad.getTriggerAxis(wpilib.GenericHID.Hand.kLeft) > 0.5:
            gear_lifter.down()
        else:
            gear_lifter.up()


class OpShooterCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)

    def can_run(self):
        return not self.my_robot.shooter.is_occupied

    def init(self):
        self.my_robot.shooter.occupy()

    def is_finished(self):
        return not self.my_robot.isOperatorControl()

    def finish(self):
        self.my_robot.shooter.release()

    def run_periodic(self):
        gamepad = self.my_robot.gamepad
        if gamepad.getBumper(wpilib.GenericHID.Hand.kRight):
            self.my_robot.shooter.active()
        elif gamepad.getYButton():
            self.my_robot.shooter.reverse()
        else:
            self.my_robot.shooter.inactive()


class OpClimberCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)

    def can_run(self):
        return not self.my_robot.climber.is_occupied

    def init(self):
        self.my_robot.climber.occupy()

    def is_finished(self):
        return not self.my_robot.isOperatorControl()

    def finish(self):
        self.my_robot.climber.release()

    def run_periodic(self):
        gamepad = self.my_robot.gamepad
        if gamepad.getAButton():
            power = (self.my_robot.gamepad.getY(wpilib.GenericHID.Hand.kRight) + 1) / 2
            self.my_robot.climber.active(power)
        else:
            self.my_robot.climber.inactive()


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
        spenner = 0.7
        if js_left.getRawButton(1):
            spenner = 1

        self.my_robot.drive.arcadeDrive(-spenner * js_left.getY(), -spenner * js_right.getX())

        if js_left.getRawButton(5):
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
        self.my_robot.drive.setSafetyEnabled(enabled=False)

        vel_rpm = (self.vel / self.scale_factor) / 60
        acc_rpm = (self.acc / self.scale_factor) / 60 ** 2
        self.my_robot.talon_left.setMotionMagicCruiseVelocity(vel_rpm)
        self.my_robot.talon_right.setMotionMagicCruiseVelocity(vel_rpm)
        self.my_robot.talon_left.setMotionMagicAcceleration(acc_rpm)
        self.my_robot.talon_right.setMotionMagicAcceleration(acc_rpm)

        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.MotionMagic)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.MotionMagic)
        print("Set mode to MM {} ".format(self.my_robot.talon_left.getControlMode()))

        dist_revs = self.dist / self.scale_factor
        self.my_robot.talon_left.set(dist_revs)
        self.my_robot.talon_right.set(dist_revs)

    def is_finished(self):
        left_on = self.my_robot.talon_left.getClosedLoopError() / self.scale_factor < self.margin
        right_on = self.my_robot.talon_right.getClosedLoopError() / self.scale_factor < self.margin
        return False  # left_on and right_on

    def finish(self):
        self.drive.release()
        self.my_robot.drive.setSafetyEnabled(enabled=True)
        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        print("Finished motion profile")

    def run_periodic(self):
        dist_revs = self.dist / self.scale_factor
        self.my_robot.talon_left.set(dist_revs)
        self.my_robot.talon_right.set(dist_revs)


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
