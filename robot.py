import threading

import math
import wpilib

import navigation
import robot_time
import ctre.cantalon

from command_based import Command
from dashboard import dashboard2
from drivetrain import drivetrain


class MyRobot(wpilib.SampleRobot):
    def __init__(self):
        super().__init__()
        self.delay_millis = 50
        self.drive = None
        self.talon_left_front = None
        self.talon_left_rear = None
        self.talon_right_rear = None
        self.talon_right_front = None
        self.talon_right = None
        self.talon_left = None
        self.victor_intake = None
        self.spark_climber = None

        self.js_left = None
        self.js_right = None

        self.cmd_queue = []
        self.current_commands = []

    def periodic(self):
        """
        Used for code that should be called on a constant loop whether the robot is disabled, auto, or teleop
        :return:
        """
        time_now = robot_time.get_match_time()
        for new_cmd in self.cmd_queue:
            if new_cmd.can_run():
                self.cmd_queue.remove(new_cmd)
                self.current_commands.append(new_cmd)
                new_cmd.init()
        for cmd in self.current_commands:
            if cmd.is_finished():
                cmd.finish()
                self.current_commands.remove(cmd)
            else:
                cmd.run_periodic()

        dashboard2.update(time_now)

    def robotInit(self):
        dashboard2.run()

        self.talon_left_front = ctre.CANTalon(0)
        self.talon_left_rear = ctre.CANTalon(1)
        self.talon_right_rear = ctre.CANTalon(2)
        self.talon_right_front = ctre.CANTalon(3)
        self.victor_intake = wpilib.VictorSP(0)
        self.spark_climber = wpilib.Spark(1)

        self.talon_left_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_left_rear.set(0)
        self.talon_right_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_right_rear.set(3)

        self.talon_left = self.talon_left_front
        self.talon_right = self.talon_right_front

        self.talon_left.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_right.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)

        self.drive = drivetrain(self.talon_left_front,
                                self.talon_right_front)
        self.drive.setInvertedMotor(wpilib.RobotDrive.MotorType.kRearRight, True)

        dashboard2.graph("Heading", self.drive.get_heading)

        self.js_left = wpilib.Joystick(0)
        self.js_right = wpilib.Joystick(1)

    def autonomous(self):
        # Init
        while self.isAutonomous():
            # Loop

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

    def disabled(self):
        # Init
        while self.isDisabled():
            # Loop

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

    def operatorControl(self):
        # Init
        self.cmd_queue.append(OpDriveCommand(self))

        while self.isOperatorControl():
            # Loop

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)


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

        if js_left.getRawButton(4):
            self.my_robot.spark_climber.set(1)
        elif js_left.getRawButton(5):
            self.my_robot.spark_climber.set(-1)
        else:
            self.my_robot.spark_climber.set(0)


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


if __name__ == '__main__':
    wpilib.run(MyRobot)
