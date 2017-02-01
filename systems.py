from command_based import Subsystem


class ClimberSubsystem(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.motor = my_robot.spark_climber

    def active(self):
        self.motor.set(1)

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()


class IntakeSubsystem(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.motor = my_robot.victor_intake

    def active(self):
        self.motor.set(1)

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()

