import os

import ctre.cantalon
import time

import math
import wpilib
from networktables import NetworkTables

import robot_time
from command_based import CommandSequence
from commands import OpDriveCommand, MotionProfileDriveCommand, OpFuelTankCommand, OpShooterCommand, OpClimberCommand, \
    OpGearCommand, TurnToAngleCommand, AutoGearCommand, DistanceDriveCommand, TurnToBoilerCommand, AutoFuelTankCommand, \
    TimeDriveCommand, AutoShooterCommand
from dashboard import dashboard2
from drivetrain import SmartDrivetrain
from wpy.motor import PWMMotor
from systems import FuelTank, Shooter, GearLifter, Climber


class MyRobot(wpilib.SampleRobot):
    class State:
        DISABLED = 0
        AUTO = 1
        TELEOP = 2

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
        self.talon_climber = None
        self.victor_blender = None

        self.js_left = None
        self.js_right = None
        self.gamepad = None

        self.cmd_queue = []
        self.current_commands = []

        self._drive = None
        self._climber = None
        self._fueltank = None
        self._shooter = None
        self._gear_lifter = None

        self.current_state = MyRobot.State.DISABLED

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
        if wpilib.hal.isSimulation():
            basedir = ""
        else:
            basedir = "/home/lvuser/py"
        dashboard2.run(basedir)
        wpilib.CameraServer.launch()

        dashboard2.chooser("Autonomous", ["None", "Drive MotionMagic", "Align Shoot",
                                          "Gear Center", "Gear Left", "Gear Right", "Hopper/Boiler Blue",
                                          "Hopper/Boiler Red", "Boiler/Mobility Blue", "Boiler/Mobility Red"],
                           default="None")

        self.talon_left_front = ctre.CANTalon(0)
        self.talon_left_rear = ctre.CANTalon(1)
        self.talon_right_rear = ctre.CANTalon(2)
        self.talon_right_front = ctre.CANTalon(3)
        self.talon_shooter = ctre.CANTalon(4)
        self.victor_intake = PWMMotor(wpilib.VictorSP, pwm_port=1, pdp_port=5)
        self.victor_blender = PWMMotor(wpilib.VictorSP, pwm_port=0, pdp_port=4)
        self.talon_climber = PWMMotor(wpilib.Talon, pwm_port=2, pdp_port=14)

        self.talon_shooter.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_shooter.reverseSensor(True)

        self.talon_left_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_left_rear.set(0)
        self.talon_right_rear.setControlMode(ctre.CANTalon.ControlMode.Follower)
        self.talon_right_rear.set(3)

        self.talon_left = self.talon_left_front
        self.talon_right = self.talon_right_front

        self.talon_left.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_right.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
        self.talon_left.reverseSensor(True)
        f_l = 0.1808
        p_l = 1.28
        i_l = 0.005
        d_l = 12.8

        f_r = 0.1724
        p_r = 1.28
        i_r = 0.005
        d_r = 12.8

        self.talon_left.setP(p_l)
        self.talon_left.setI(i_l)
        self.talon_left.setD(d_l)
        self.talon_left.setF(f_l)

        self.talon_right.setP(p_r)
        self.talon_right.setI(i_r)
        self.talon_right.setD(d_r)
        self.talon_right.setF(f_r)

        volt_ramp_time = 1/3
        volt_ramp = 12 / volt_ramp_time
        # self.talon_left.setVoltageRampRate(volt_ramp)
        # self.talon_right.setVoltageRampRate(volt_ramp)

        dashboard2.graph("Blender Current", self.victor_blender.get_current)
        dashboard2.graph("Climber Current", self.talon_climber.get_current)
        dashboard2.graph("Drive Current", self.talon_left.getOutputCurrent)

        sensor_type = ctre.CANTalon.FeedbackDevice.CtreMagEncoder_Relative
        sensor_present = ctre.CANTalon.FeedbackDeviceStatus.Present
        dashboard2.indicator("Left Encoder", lambda: self.talon_left.isSensorPresent(sensor_type) == sensor_present)
        dashboard2.indicator("Right Encoder", lambda: self.talon_right.isSensorPresent(sensor_type) == sensor_present)

        self._drive = SmartDrivetrain(self.talon_left,
                                      self.talon_right)

        self._fueltank = FuelTank(self, self.victor_intake, self.victor_blender)
        self._shooter = Shooter(self, self.talon_shooter)
        self._climber = Climber(self, self.talon_climber)
        self._gear_lifter = GearLifter(self)

        dashboard2.number_input("Drive Vel", 0.01)
        dashboard2.number_input("Drive Acc", 0.001)
        dashboard2.number_input("Drive Dist", 10)

        self.systems = {"drive": self._drive,
                        "climber": self._climber,
                        "intake": self._fueltank,
                        "shooter": self._shooter,
                        "gear_lifter": self._gear_lifter}

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
        print("Robot ready!")

    def autonomous(self):
        if self.current_state != self.State.AUTO:
            mode = dashboard2.get_chooser("Autonomous")

            self.talon_left.setPosition(0)
            self.talon_right.setPosition(0)
            vel = float(dashboard2.number_inputs["Drive Vel"])
            acc = float(dashboard2.number_inputs["Drive Acc"])
            dist = float(dashboard2.number_inputs["Drive Dist"])

            drive_vel = 0.02
            drive_acc = 0.01

            def gear_side(is_right=True):
                cmds = []
                cmds.append(AutoGearCommand(self, AutoGearCommand.State.up))
                cmds.append(MotionProfileDriveCommand(self, (108 - 15.5 - 10 + 5 + (0 if is_right else 5)) / 12,
                                                      drive_vel, drive_acc))
                cmds.append(TurnToAngleCommand(self, (-1 if is_right else 1) * 53))
                cmds.append(MotionProfileDriveCommand(self, (44 - 4 - 5) / 12, drive_vel, drive_acc))
                cmds.append(AutoGearCommand(self, AutoGearCommand.State.down))
                cmds.append(MotionProfileDriveCommand(self, -30 / 12, drive_vel, drive_acc))
                return cmds
            
            def align_shoot(is_right=True):
                cmds = []
                cmds.append(TurnToAngleCommand(self, (1 if is_right else -1) * 90))
                cmds.append(TurnToBoilerCommand(self))
                cmds.append(AutoShooterCommand(self, 10, 3750))
                return cmds

            if mode == "Drive MotionMagic":
                cmds = []
                cmds.append(MotionProfileDriveCommand(self, dist, vel, acc))
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Align Shoot":
                cmds = []
                cmds += align_shoot(True)
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Gear Center":
                cmds = []
                cmds.append(AutoGearCommand(self, AutoGearCommand.State.up))
                cmds.append(MotionProfileDriveCommand(self, (110 - 15.5 - 4 - 10) / 12, drive_vel, drive_acc))
                cmds.append(AutoGearCommand(self, AutoGearCommand.State.down))
                cmds.append(MotionProfileDriveCommand(self, -40 / 12, drive_vel, drive_acc))
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Gear Left":
                cmds = gear_side(False)
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Gear Right":
                cmds = gear_side(True)
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Gear Left + Shoot":
                cmds = gear_side(False)
                cmds += align_shoot(False)
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Gear Right + Shoot":
                cmds = gear_side(True)
                cmds += align_shoot(True)
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Hopper/Boiler Blue":
                cmds = []
                cmds.append(DistanceDriveCommand(self, 93.5, 0.6))
                cmds.append(TurnToAngleCommand(self, 90))
                cmds.append(TimeDriveCommand(self, 2, 0.6))
                cmds.append(AutoShooterCommand(self, 10, 3700))
                cmds.append(AutoFuelTankCommand(self, 10))
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Hopper/Boiler Red":
                cmds = []
                cmds.append(DistanceDriveCommand(self, 93.5, 0.6))
                cmds.append(TurnToAngleCommand(self, 90))
                cmds.append(TimeDriveCommand(self, 2, -0.6))
                cmds.append(AutoShooterCommand(self, 10, 3700))
                cmds.append(AutoFuelTankCommand(self, 10))
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Boiler/Mobility Red":
                cmds = []
                cmds.append(AutoShooterCommand(self, 10, 3650))
                #cmds.append(AutoFuelTankCommand(self, 10))
                cmds.append(DistanceDriveCommand(self, 100, -0.6))
                self.cmd_queue.append(CommandSequence(self, cmds))
            if mode == "Boiler/Mobility Blue":
                cmds = []
                cmds.append(AutoShooterCommand(self, 10, 3650))
                #cmds.append(AutoFuelTankCommand(self, 10))
                cmds.append(DistanceDriveCommand(self, 100, 0.6))
                self.cmd_queue.append(CommandSequence(self, cmds))


        # Init
        while self.isAutonomous():
            # Loop
            self.current_state = MyRobot.State.AUTO
            
            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

        for cmd in self.current_commands:
            cmd.cancel()

    def disabled(self):
        # Init
        while self.isDisabled():
            # Loop
            self.current_state = MyRobot.State.DISABLED

            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

        for cmd in self.current_commands:
            cmd.cancel()

    def operatorControl(self):
        # Init
        if self.current_state != self.State.TELEOP:
            self.cmd_queue.append(OpDriveCommand(self))
            self.cmd_queue.append(OpFuelTankCommand(self))
            self.cmd_queue.append(OpShooterCommand(self))
            self.cmd_queue.append(OpClimberCommand(self))
            self.cmd_queue.append(OpGearCommand(self))

        while self.isOperatorControl():
            # Loop
            self.current_state = MyRobot.State.TELEOP
            print(self.js_left.getRawButton(1))
            self.periodic()
            robot_time.sleep(millis=self.delay_millis)

        for cmd in self.current_commands:
            cmd.cancel()

    @property
    def drive(self) -> SmartDrivetrain:
        return self._drive

    @property
    def climber(self) -> Climber:
        return self._climber

    @property
    def gear_lifter(self) -> GearLifter:
        return self._gear_lifter

    @property
    def fueltank(self) -> FuelTank:
        return self._fueltank

    @property
    def shooter(self) -> Shooter:
        return self._shooter




if __name__ == '__main__':
    wpilib.run(MyRobot)
