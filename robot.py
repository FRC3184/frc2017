import wpilib

import robot_time
import ctre.cantalon

import state_logging
from commands import OpDriveCommand
from dashboard import dashboard2
from drivetrain import Drivetrain
from motor import PWMMotor


class MyRobot(wpilib.SampleRobot):
    def __init__(self):
        super().__init__()
        self.delay_millis = 50
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

        self.drive = None
        self.climber = None
        self.intake = None

        self.systems = {"drive": self.drive,
                        "climber": self.climber,
                        "intake": self.intake}

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
        for system in self.systems.values():
            if system is not None and not system.is_occupied:
                system.default()

        dashboard2.update(time_now)

    def robotInit(self):
        dashboard2.run()

        self.talon_left_front = ctre.CANTalon(0)
        self.talon_left_rear = ctre.CANTalon(1)
        self.talon_right_rear = ctre.CANTalon(2)
        self.talon_right_front = ctre.CANTalon(3)
        self.victor_intake = PWMMotor(wpilib.VictorSP, pwm_port=0, pdp_port=0)
        self.spark_climber = PWMMotor(wpilib.Spark, pwm_port=1, pdp_port=1)  # TODO actually get these values

        self.talon_left_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_left_rear.set(0)
        self.talon_right_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_right_rear.set(3)

        self.talon_left = self.talon_left_front
        self.talon_right = self.talon_right_front

        self.talon_left.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_right.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)

        self.drive = Drivetrain(self.talon_left_front,
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


if __name__ == '__main__':
    wpilib.run(MyRobot)
