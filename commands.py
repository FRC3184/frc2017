import wpilib
from networktables import NetworkTables

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
        print("Turn started")
        self.my_robot.drive.ahrs.reset()
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
        self.timer = wpilib.Timer()
        self.timer.start()
        self.was_down_last = False

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
            if gamepad.getTriggerAxis(wpilib.GenericHID.Hand.kRight) > 0.5:
                gear_lifter.close_grab()
            else:
                gear_lifter.release_grab()
            if not self.was_down_last:
                self.timer.reset()
                self.was_down_last = True
            if self.timer.get() > 0.1:
                gear_lifter.down()
        else:
            gear_lifter.close_grab()
            if self.was_down_last:
                self.timer.reset()
                self.was_down_last = False
            if self.timer.get() > 0.1:
                gear_lifter.up()


class AutoGearCommand(Command):
    class State:
        down = 0
        up = 1

    def __init__(self, my_robot, state):
        super().__init__(my_robot)
        self.timer = wpilib.Timer()
        self.timer.start()
        self.was_down_last = False
        self.state = state

        self.delay = 0.1
        self.wait_after_delay = 0.2

    def can_run(self):
        return not self.my_robot.gear_lifter.is_occupied

    def init(self):
        gear_lifter = self.my_robot.gear_lifter

        gear_lifter.occupy()
        self.timer.reset()
        if self.state == self.State.down:
            gear_lifter.release_grab()
            print("Gear down")
        elif self.state == self.State.up:
            print("Gear up")
            gear_lifter.close_grab()

    def is_finished(self):
        return self.timer.get() > self.delay + self.wait_after_delay

    def finish(self):
        self.my_robot.gear_lifter.release()

    def run_periodic(self):
        gear_lifter = self.my_robot.gear_lifter

        if self.timer.get() > self.delay:
            if self.state == self.State.up:
                gear_lifter.up()
            elif self.state == self.State.down:
                gear_lifter.down()


class OpShooterCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)

    def can_run(self):
        return not self.my_robot.shooter.is_occupied

    def init(self):
        self.my_robot.shooter.occupy()
        self.my_robot.shooter.set_target_rpm(3500)

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
        pov = gamepad.getPOV(0)
        if pov == 0:
            rpm = self.my_robot.shooter.get_target_rpm() + 50
            self.my_robot.shooter.set_target_rpm(rpm)
            print("New RPM: {}".format(rpm))
        if pov == 180:
            rpm = self.my_robot.shooter.get_target_rpm() - 50
            self.my_robot.shooter.set_target_rpm(rpm)
            print("New RPM: {}".format(rpm))


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
            limit = 0.7
            power = limit * (self.my_robot.gamepad.getY(wpilib.GenericHID.Hand.kRight) + 1) / 2
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
        spenner_button = 1
        tank_button = 3
        if js_left.getRawButton(spenner_button) or js_right.getRawButton(spenner_button):
            spenner = 1
        if js_left.getRawButton(tank_button) or js_right.getRawButton(tank_button):
            self.my_robot.drive.tankDrive(-spenner * js_left.getY(), -spenner * js_right.getY())
        else:
            self.my_robot.drive.arcadeDrive(-spenner * js_left.getY(), -spenner * js_right.getX())


class DistanceDriveCommand(Command):
    def __init__(self, my_robot, dist, percent_vbus, margin=2):
        super().__init__(my_robot)
        self.drive = my_robot.drive
        self.dist = dist
        self.vbus = percent_vbus
        self.margin = margin

        self.scale_factor = (math.pi * self.drive.wheel_diameter)
        self.dist_revs = self.dist / self.scale_factor
        self.angle = 0

    def can_run(self):
        return not self.drive.is_occupied

    def init(self):
        print("{}in drive started".format(self.dist))
        self.drive.occupy()
        self.my_robot.talon_left.setPosition(0)
        self.my_robot.talon_right.setPosition(0)
        self.angle = self.drive.get_heading()

    def is_finished(self):
        left_on = abs(self.my_robot.talon_left.getPosition() * self.scale_factor) > self.dist
        right_on = abs(self.my_robot.talon_right.getPosition() * self.scale_factor) > self.dist
        return left_on or right_on

    def finish(self):
        self.drive.release()
        self.my_robot.talon_left.setPosition(0)
        self.my_robot.talon_right.setPosition(0)

    def run_periodic(self):
        err = self.angle - self.drive.get_heading()
        self.drive.arcadeDrive(self.vbus, -err / 180)


class TimeDriveCommand(Command):
    def __init__(self, my_robot, time, percent_vbus):
        super().__init__(my_robot)
        self.drive = my_robot.drive
        self.vbus = percent_vbus
        self.time = time
        self.angle = self.drive.get_heading()

        self.timer = wpilib.Timer()

    def can_run(self):
        return not self.drive.is_occupied

    def init(self):
        self.drive.occupy()
        self.timer.reset()

    def is_finished(self):
        return self.timer.get() > self.time

    def finish(self):
        self.drive.release()

    def run_periodic(self):
        err = self.angle - self.drive.get_heading()
        self.drive.arcadeDrive(self.vbus, -err / 180)


class MotionProfileDriveCommand(Command):
    def __init__(self, my_robot, dist, vel, acc, margin=1/12):
        super().__init__(my_robot)
        self.drive = my_robot.drive
        self.dist = dist
        self.vel = vel  # ft/s
        self.acc = acc  # ft/s^2
        self.margin = margin

        self.scale_factor = math.pi * self.drive.wheel_diameter / 12  # ft/rev

    def can_run(self):
        return not self.drive.is_occupied

    def init(self):
        self.drive.occupy()
        self.my_robot.drive.setSafetyEnabled(enabled=False)

        vel_rpm = (self.vel / self.scale_factor) * 60  # RPM
        acc_rpm = (self.acc / self.scale_factor) * 60  # RPM/sec
        print("RPM: {}".format(vel_rpm))
        print("RPM/s: {}".format(acc_rpm))
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
        left_on = self.my_robot.talon_left.getClosedLoopError() * self.scale_factor < self.margin
        right_on = self.my_robot.talon_right.getClosedLoopError() * self.scale_factor < self.margin
        return False  # left_on and right_on

    def finish(self):
        self.drive.release()
        self.my_robot.drive.setSafetyEnabled(enabled=True)
        self.my_robot.talon_left.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.my_robot.talon_right.setControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        print("Finished motion profile")

    def run_periodic(self):
        dist_revs = self.dist / self.scale_factor
        print("Err L: {}".format(self.my_robot.talon_left.getPosition() - dist_revs))
        print("Err R: {}".format(self.my_robot.talon_right.getPosition() - dist_revs))


class AutoShooterCommand(Command):
    def __init__(self, my_robot, time, rpm):
        super().__init__(my_robot)
        self.shooter = my_robot.shooter
        self.timer = wpilib.Timer()
        self.time = time
        self.rpm = rpm
        self.fueltank = my_robot.fueltank

    def can_run(self):
        return (not self.shooter.is_occupied) and (not self.fueltank.is_occupied)

    def init(self):
        self.shooter.occupy()
        self.fueltank.occupy()
        self.timer.reset()
        self.timer.start()
        self.shooter.set_target_rpm(self.rpm)

    def is_finished(self):
        return self.timer.get() > self.time

    def finish(self):
        self.shooter.release()
        self.fueltank.release()

    def run_periodic(self):
        self.shooter.active()
        if self.timer.get() > 1:
            self.fueltank.blender_active()
            self.fueltank.intake_active()


class AutoFuelTankCommand(Command):
    def __init__(self, my_robot, time):
        super().__init__(my_robot)
        self.fueltank = my_robot.fueltank
        self.timer = wpilib.Timer()
        self.time = time

    def can_run(self):
        return not self.fueltank.is_occupied

    def init(self):
        self.fueltank.occupy()
        self.timer.reset()
        self.timer.start()

    def is_finished(self):
        return self.timer.get() > self.time

    def finish(self):
        self.fueltank.release()

    def run_periodic(self):
        self.fueltank.blender_active()
        self.fueltank.intake_active()


class TurnToBoilerCommand(Command):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.result = False
        self.angle = 0
        self.state = 0
        self.vision_table = NetworkTables.getTable("vision")

    def can_run(self):
        return not self.my_robot.drive.is_occupied

    def init(self):
        self.my_robot.drive.occupy()
        self.result = False
        self.angle = 0
        self.state = 0
        self.vision_table = NetworkTables.getTable("vision")

    def is_finished(self):
        return self.result

    def finish(self):
        self.my_robot.drive.release()

    def run_periodic(self):
        if self.state == 0:
            if self.vision_table.getBoolean("seeBoiler", False):
                self.state = 1
                self.angle = self.vision_table.getNumber("angle") * 0.5
                self.my_robot.drive.ahrs.reset()
            else:
                self.my_robot.drive.arcadeDrive(0, 0.5)

        elif self.state == 1:
            self.result = self.my_robot.drive.turn_to_angle(self.angle, allowable_error=1.5)
            # self.state = 2
        elif self.state == 2:
            self.angle = self.vision_table.getNumber("angle")
            if abs(self.angle) < 1:
                self.result = True
            else:
                self.state = 1
                self.angle = self.vision_table.getNumber("angle") * 0.5
                self.my_robot.drive.ahrs.reset()


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
