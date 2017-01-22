import wpilib
import robot_time
import ctre.cantalon as ctre
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

        self.js_left = None
        self.js_right = None

    def periodic(self):
        """
        Used for code that should be called on a constant loop whether the robot is disabled, auto, or teleop
        :return:
        """
        time_now = robot_time.get_match_time()

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

        while self.isOperatorControl():
            # Loop
            if self.js_left.getRawButton(1):
                self.drive.turn_to_angle(0)
            else:
                self.drive.arcadeDrive(-self.js_left.getY(), -self.js_right.getX())
            intake_pow = 0
            if self.js_left.getRawButton(4):
                intake_pow = 1
            elif self.js_left.getRawButton(5):
                intake_pow = -1
            self.victor_intake.set(intake_pow)

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

if __name__ == '__main__':
    wpilib.run(MyRobot)
