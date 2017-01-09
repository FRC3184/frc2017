import wpilib
import robot_time
from dashboard import dashboard2


class MyRobot(wpilib.SampleRobot):
    def __init__(self):
        super().__init__()
        self.delay_millis = 50

    def periodic(self):
        """
        Used for code that should be called on a constant loop whether the robot is disabled, auto, or teleop
        :return:
        """
        time_now = robot_time.get_match_time()

        dashboard2.update(time_now)

    def robotInit(self):
        dashboard2.run()
        dashboard2.graph("Time", robot_time.get_match_time)

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

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

if __name__ == '__main__':
    wpilib.run(MyRobot)
