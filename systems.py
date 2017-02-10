from command_based import Subsystem


class Climber(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.motor = my_robot.spark_climber

    def active(self):
        self.motor.set(1)

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()


class FuelTank(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.intake_motor = my_robot.victor_intake
        self.blender_motor = my_robot.victor_blender

    def intake_active(self):
        self.intake_motor.set(.5)

    def intake_inactive(self):
        self.intake_motor.set(0)

    def blender_active(self):
        self.blender_motor.set(.5)

    def blender_inactive(self):
        self.blender_motor.set(0)

    def default(self):
        self.intake_inactive()


class Shooter(Subsystem):
    def __init__(self, my_robot):
        super().__init__(my_robot)
        self.motor = my_robot.talon_shooter

    def active(self):
        self.motor.set(1)

    def inactive(self):
        self.motor.set(0)

    def default(self):
        self.inactive()