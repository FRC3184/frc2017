# import robot


class Command:
    def __init__(self, my_robot):
        """

        :param my_robot: The robot
        :type my_robot: robot.MyRobot
        """
        self.my_robot = my_robot

    def init(self):
        pass

    def run_periodic(self):
        pass

    def finish(self):
        pass

    def is_finished(self):
        pass


class Subsystem:
    def __init__(self, is_exclusive=True):
        self.is_occupied = False
        self.is_exclusive = is_exclusive

    def occupy(self):
        if not self.is_occupied or not self.is_exclusive:
            self.is_occupied = True
        else:
            raise ExclusiveException("Subsystem is occupied")

    def release(self):
        self.is_occupied = False


class ExclusiveException(Exception):
    pass
