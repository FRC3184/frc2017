import ctre.cantalon
import time

import math
import wpilib
from networktables import NetworkTables

import robot_time
from command_based import CommandSequence
from commands import OpDriveCommand, MotionProfileDriveCommand, OpFuelTankCommand, OpShooterCommand, OpClimberCommand, \
    OpGearCommand, TurnToAngleCommand, AutoGearCommand, DistanceDriveCommand
from dashboard import dashboard2
from drivetrain import Drivetrain
from wpy.motor import PWMMotor
from systems import FuelTank, Shooter, GearLifter, Climber


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
        self.talon_shooter = None
        self.victor_intake = None
        self.spark_climber = None
        self.victor_blender = None

        self.js_left = None
        self.js_right = None
        self.gamepad = None

        self.cmd_queue = []
        self.current_commands = []

        self.drive = None
        self.climber = None
        self.fueltank = None
        self.shooter = None
        self.gear_lifter = None

        self.systems = {}

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

        dashboard2.chooser("Autonomous", ["None", "Gear Center", "Gear Left", "Gear Right"], default="None")

        self.talon_left_front = ctre.CANTalon(0)
        self.talon_left_rear = ctre.CANTalon(1)
        self.talon_right_rear = ctre.CANTalon(2)
        self.talon_right_front = ctre.CANTalon(3)
        self.talon_shooter = ctre.CANTalon(4)
        self.victor_intake = PWMMotor(wpilib.VictorSP, pwm_port=1, pdp_port=5)
        self.victor_blender = PWMMotor(wpilib.VictorSP, pwm_port=0, pdp_port=4)
        self.spark_climber = PWMMotor(wpilib.Spark, pwm_port=2, pdp_port=14)

        self.talon_shooter.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_shooter.reverseSensor(True)

        dashboard2.graph("Intake Current", self.victor_intake.get_current)
        dashboard2.graph("Blender Current", self.victor_blender.get_current)
        dashboard2.graph("Climber Current", self.spark_climber.get_current)
        dashboard2.graph("Total Current", wpilib.PowerDistributionPanel().getTotalCurrent)
        dashboard2.graph("Left", lambda: self.talon_left.getPosition() * (math.pi*4))
        dashboard2.graph("Right", lambda: self.talon_right.getPosition() * (math.pi*4))

        self.talon_left_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_left_rear.set(0)
        self.talon_right_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_right_rear.set(3)

        self.talon_left = self.talon_left_front
        self.talon_right = self.talon_right_front

        self.talon_left.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_right.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_right.reverseSensor(True)
        kF = 1023 / 5676
        kP = 0
        self.talon_left.setF(kF)
        self.talon_left.setP(kP)
        self.talon_right.setF(kF)
        self.talon_right.setP(kP)

        self.drive = Drivetrain(self.talon_left_front,
                                self.talon_right_front)
        self.drive.setInvertedMotor(Drivetrain.MotorType.kRearRight, True)
        self.fueltank = FuelTank(self, self.victor_intake, self.victor_blender)
        self.shooter = Shooter(self, self.talon_shooter)
        self.climber = Climber(self, self.spark_climber)
        self.gear_lifter = GearLifter(self)

        self.systems = {"drive": self.drive,
                        "climber": self.climber,
                        "intake": self.fueltank,
                        "shooter": self.shooter,
                        "gear_lifter": self.gear_lifter}

        self.js_left = wpilib.Joystick(0)
        self.js_right = wpilib.Joystick(1)
        self.gamepad = wpilib.XboxController(2)

        if not wpilib.hal.isSimulation():
            t_wait = 30
            print("Waiting for vision... ({}s max)".format(t_wait))
            _vision_table = NetworkTables.getTable("vision")
            t_begin = time.time()

            # Wait until either vision is ready or 30s has passed
            while not _vision_table.getBoolean("ready", False) and not (time.time() - t_begin) > t_wait:
                robot_time.sleep(seconds=1)
                break  # Skip vision since we aren't actually doing it
        print("Robot ready!")

    def autonomous(self):
        mode = dashboard2.get_chooser("Autonomous")

        self.talon_left.setPosition(0)
        self.talon_right.setPosition(0)

        if mode == "Gear Center":
            cmds = []
            cmds.append(AutoGearCommand(self, AutoGearCommand.State.up))
            cmds.append(DistanceDriveCommand(self, 80, 0.6))
            cmds.append(AutoGearCommand(self, AutoGearCommand.State.down))
            cmds.append(DistanceDriveCommand(self, 30, -0.8))
            self.cmd_queue.append(CommandSequence(self, cmds))
            # self.cmd_queue.append(MotionProfileDriveCommand(self, -35.29, 1, 1, margin=1))
        if mode == "Gear Left":
            cmds = []
            cmds.append(AutoGearCommand(self, AutoGearCommand.State.up))
            cmds.append(DistanceDriveCommand(self, 70, 0.6))
            cmds.append(TurnToAngleCommand(self, 60))
            cmds.append(DistanceDriveCommand(self, 40, 0.6))
            cmds.append(AutoGearCommand(self, AutoGearCommand.State.down))
            cmds.append(DistanceDriveCommand(self, 30, -0.8))
            self.cmd_queue.append(CommandSequence(self, cmds))
        if mode == "Gear Right":
            cmds = []
            cmds.append(AutoGearCommand(self, AutoGearCommand.State.up))
            cmds.append(DistanceDriveCommand(self, 80, 0.6))
            cmds.append(TurnToAngleCommand(self, 60))
            cmds.append(DistanceDriveCommand(self, 30, 0.6))
            cmds.append(AutoGearCommand(self, AutoGearCommand.State.down))
            cmds.append(DistanceDriveCommand(self, 30, -0.8))
            self.cmd_queue.append(CommandSequence(self, cmds))

        # Init
        while self.isAutonomous():
            # Loop

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

        for cmd in self.current_commands:
            cmd.cancel()

    def disabled(self):
        # Init
        while self.isDisabled():
            # Loop

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

        for cmd in self.current_commands:
            cmd.cancel()

    def operatorControl(self):
        # Init
        self.cmd_queue.append(OpDriveCommand(self))
        self.cmd_queue.append(OpFuelTankCommand(self))
        self.cmd_queue.append(OpShooterCommand(self))
        self.cmd_queue.append(OpClimberCommand(self))
        self.cmd_queue.append(OpGearCommand(self))

        while self.isOperatorControl():
            # Loop

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

        for cmd in self.current_commands:
            cmd.cancel()

if __name__ == '__main__':
    wpilib.run(MyRobot)
