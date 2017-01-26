import wpilib
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
        self.victor_intake = None

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

        self.drive = drivetrain(self.talon_left_front,
                                self.talon_left_rear,
                                self.talon_right_front,
                                self.talon_right_rear)

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


if __name__ == '__main__':
    wpilib.run(MyRobot)
