# import robot
import wpilib


class Command:
    def __init__(self, my_robot):
        """

        :param my_robot: The robot
        :type my_robot: robot.MyRobot
        """
        self.my_robot = my_robot

    def can_run(self):
        return True

    def init(self):
        pass

    def run_periodic(self):
        pass

    def finish(self):
        pass

    def is_finished(self):
        pass

    def cancel(self):
        self.is_finished = lambda: True


class CommandSequence(Command):
    def __init__(self, my_robot, cmds):
        """

        :param my_robot: The robot
        :type my_robot: robot.MyRobot
        """
        super().__init__(my_robot)
        self.cmds = cmds
        self.current = self.cmds.pop(0)

    def init(self):
        self.my_robot.cmd_queue.append(self.current)

    def run_periodic(self):
        if self.current.is_finished():
            self.current = self.cmds.pop(0)
            self.my_robot.cmd_queue.append(self.current)

    def finish(self):
        pass

    def is_finished(self):
        return len(self.cmds) == 0

    def cancel(self):
        self.is_finished = lambda: True


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

    def default(self):
        pass


class ExclusiveException(Exception):
    pass
