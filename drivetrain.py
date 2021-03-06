import threading
import warnings

import ctre
import wpilib
import math
from robotpy_ext.common_drivers.navx.ahrs import AHRS

import mathutils
import pose
import robot_time
from command_based import Subsystem
from dashboard import dashboard2


class SmartDrivetrain(Subsystem, wpilib.MotorSafety):
    class Mode:
        PercentVbus = ctre.CANTalon.ControlMode.PercentVbus
        Voltage = ctre.CANTalon.ControlMode.Voltage
        Speed = ctre.CANTalon.ControlMode.Speed
        MotionMagic = ctre.CANTalon.ControlMode.MotionMagic
        MotionProfile = ctre.CANTalon.ControlMode.MotionProfile

    def __init__(self, left_motor: ctre.CANTalon, right_motor: ctre.CANTalon, **kwargs):
        '''
        Represents a drivetrain that uses CANTalons and so manages those advanced features
        :param left_motor: 
        :param right_motor: 
        :param kwargs: 
        '''
        Subsystem.__init__(self)
        wpilib.MotorSafety.__init__(self)
        self.robot_width = kwargs.pop("robot_width", 29.25 / 12)
        self.max_turn_radius = kwargs.pop("max_radius", 10)
        self.wheel_diameter = kwargs.pop("wheel_diameter", 4)
        self.max_speed = kwargs.pop("max_speed", 13)

        self.ahrs = AHRS.create_i2c()
        self._left_motor = left_motor
        self._right_motor = right_motor

        self._model_left_dist = 0
        self._model_right_dist = 0
        self._model_last_time = robot_time.millis()

        pose.init(left_encoder_callback=self.get_left_distance,
                  right_encoder_callback=self.get_right_distance,
                  gyro_callback=(None if wpilib.hal.isSimulation() else self.get_heading_rads),
                  wheelbase=self.robot_width)
        dashboard2.graph("Pose X", lambda: pose.get_current_pose().x)
        dashboard2.graph("Pose Y", lambda: pose.get_current_pose().y)
        dashboard2.graph("Distance to target",
                         lambda: pose.get_current_pose().distance(mathutils.Vector2(6, -4)))

        self._max_output = 1
        self._mode = SmartDrivetrain.Mode.PercentVbus
        self.set_mode(self._mode)

        if wpilib.hal.isSimulation():
            model_thread = threading.Thread(target=self._update_model)
            model_thread.start()
        
        # Motor safety
        self.setSafetyEnabled(True)

        pose.init(left_encoder_callback=self.get_left_distance, right_encoder_callback=self.get_right_distance,
                  gyro_callback=(self.get_heading if not wpilib.hal.isSimulation() else None),
                  wheelbase=self.robot_width,
                  encoder_factor=self.get_fps_rpm_ratio())

        dashboard2.graph("Heading", lambda: pose.get_current_pose().heading * 180 / math.pi)

    def _update_model(self):
        while True:
            print("Model!")
            now = robot_time.millis()
            dt = (now - self._model_last_time) / 1000
            self._model_last_time = now
            if self._mode == SmartDrivetrain.Mode.PercentVbus:
                factor = 1
            elif self._mode == SmartDrivetrain.Mode.Voltage:
                factor = 1/12
            else:
                print("Can't update model outside of PercentVBus")
                continue
            self._model_left_dist += self._left_motor.get() * self.max_speed * dt * factor
            self._model_right_dist += self._right_motor.get() * self.max_speed * dt * factor
            robot_time.sleep(millis=20)

    def set_mode(self, mode):
        if self._mode != mode:
            self._mode = mode
            self._left_motor.setControlMode(self._mode)
            self._right_motor.setControlMode(self._mode)
            if self._mode == SmartDrivetrain.Mode.PercentVbus:
                self._max_output = 1
                self.setSafetyEnabled(True)
            elif self._mode == SmartDrivetrain.Mode.Voltage:
                self._max_output = 12
                self.setSafetyEnabled(True)
            elif self._mode == SmartDrivetrain.Mode.Speed:
                self._max_output = self.rpm_to_native_speed(self.get_fps_rpm_ratio())
                self.setSafetyEnabled(True)
            else:
                self.setSafetyEnabled(False)
                self._max_output = 0  # The idea of a max setpoint doesn't make sense for motion profiles

    def _radius_turn(self, pow, radius):
        D = self.robot_width / 2
        turn_dir = mathutils.sgn(radius)
        radius = abs(radius)
        Vo = pow
        Vi = Vo * (radius - D) / (radius + D)

        if turn_dir > 0:
            self._set_motor_outputs(Vo, Vi)
        else:
            self._set_motor_outputs(Vi, Vo)

    def radius_drive(self, forward_power, turn_power, power_factor, deadband=0.05):
        if self.is_manual_control_mode():
            if abs(turn_power) < deadband:
                self._set_motor_outputs(forward_power * power_factor, forward_power * power_factor)
                return
            if abs(forward_power) < deadband:
                self._set_motor_outputs(0, 0)
                return
            turn_power = mathutils.signed_power(turn_power, 1/3)
            radius = self.robot_width / 2 + self.max_turn_radius * (1 - abs(turn_power))
            self._radius_turn(forward_power * power_factor,
                              radius * mathutils.sgn(turn_power))
        else:
            warnings.warn("Not in a control mode for Radius Drive", RuntimeWarning)
            self._set_motor_outputs(0, 0)

    def motion_magic(self, distance: float, speed: float, acc: float, curvature: float = 0):
        """
        Set the talons to drive in an arc or straight at speed, accelerating at acc.
        Only needs to be called once.
        If curvature != 0, the outside wheel goes distance at speed and acc, and the inner wheel speed is decreased.
        :param distance: Distance in feet to travel
        :param speed: Speed (in feet/sec) to cruise at
        :param acc: Acceleration (in feet/sec^2) to accelerate/decelerate at
        :param curvature: 1/radius of turn. If 0, drive straight.
        :return: 
        """
        if curvature == 0:
            ratio = 1
            turn_dir = 1
        else:
            radius = 1 / curvature
            D = self.robot_width / 2
            turn_dir = mathutils.sgn(radius)
            radius = abs(radius)
            ratio = (radius - D) / (radius + D)

        # Change units to what the talons are expecting
        vel_rpm = self.fps_to_rpm(speed)
        acc_rpm = self.fps_to_rpm(acc)  # Works because required unit is rpm/sec for no real good reason.
        dist_revs = self.feet_to_revs(distance)
        print(dist_revs)

        # Don't set encoder position to 0, because that would mess up pose estimation
        # Instead, set to current position, plus however far we want to go
        left_current_pos = self._left_motor.getPosition()
        right_current_pos = self._right_motor.getPosition()


        # Set the talon parameters
        # If turn > 0, left is outside
        if turn_dir > 0:
            self._left_motor.setMotionMagicCruiseVelocity(vel_rpm)
            self._right_motor.setMotionMagicCruiseVelocity(vel_rpm * ratio)
            self._left_motor.setMotionMagicAcceleration(acc_rpm)
            self._right_motor.setMotionMagicAcceleration(acc_rpm * ratio)
            self._left_motor.set(left_current_pos + dist_revs)
            self._right_motor.set(right_current_pos + dist_revs * ratio)
        else:
            self._left_motor.setMotionMagicCruiseVelocity(vel_rpm * ratio)
            self._right_motor.setMotionMagicCruiseVelocity(vel_rpm)
            self._left_motor.setMotionMagicAcceleration(acc_rpm * ratio)
            self._right_motor.setMotionMagicAcceleration(acc_rpm)
            self._left_motor.set(left_current_pos + dist_revs * ratio)
            self._right_motor.set(right_current_pos + dist_revs)

    def tank_drive(self, left: float, right: float, deadband=0.05):
        if self.is_manual_control_mode():
            left = mathutils.deadband(left, deadband)
            right = mathutils.deadband(right, deadband)
            self._set_motor_outputs(left, right)
        else:
            warnings.warn("Not in a control mode for Tank Drive", RuntimeWarning)
            self._set_motor_outputs(0, 0)
    
    def arcade_drive(self, forward: float, turn: float, deadband=0.05):
        if self.is_manual_control_mode():
            forward = mathutils.deadband(forward, deadband)
            turn = mathutils.deadband(turn, deadband)

            if forward > 0.0:
                if turn > 0.0:
                    left = forward - turn
                    right = max(forward, turn)
                else:
                    left = max(forward, -turn)
                    right = forward + turn
            else:
                if turn > 0.0:
                    left = -max(-forward, turn)
                    right = forward + turn
                else:
                    left = forward - turn
                    right = -max(-forward, -turn)

            self._set_motor_outputs(left, right)
        else:
            warnings.warn("Not in a control mode for Arcade Drive", RuntimeWarning)
            self._set_motor_outputs(0, 0)

    def get_heading(self):
        return self.ahrs.getYaw()

    def get_heading_rads(self):
        return -self.ahrs.getYaw() * math.pi / 180

    def get_left_distance(self):
        if wpilib.hal.isSimulation():
            return self._model_left_dist
        else:
            return self._left_motor.getPosition() * (math.pi * self.wheel_diameter / 12)

    def get_right_distance(self):
        if wpilib.hal.isSimulation():
            return self._model_right_dist
        else:
            return self._right_motor.getPosition() * (math.pi * self.wheel_diameter / 12)

    def default(self):
        self._set_motor_outputs(0, 0)

    def get_fps_rpm_ratio(self):
        return 12 * 60 * self.max_speed / (math.pi * self.wheel_diameter)

    def fps_to_rpm(self, fps: float):
        return 60 * 12 * fps / (math.pi * self.wheel_diameter)

    def feet_to_revs(self, feet: float):
        return 12 * feet / (math.pi * self.wheel_diameter)

    def revs_to_feet(self, revs: float):
        try:
            return (math.pi * self.wheel_diameter) * revs / 12
        except ZeroDivisionError:
            return 0

    def rpm_to_native_speed(self, rpm: float):
        return rpm * 4096 / 600
    
    def _set_motor_outputs(self, left: float, right: float):
        if self._mode == SmartDrivetrain.Mode.Speed:
            left = self.fps_to_rpm(left * self.max_speed)
            right = self.fps_to_rpm(right * self.max_speed)
        self._left_motor.set(left)
        self._right_motor.set(right)
        self.feed()

    def has_finished_motion_magic(self, margin=1/12):
        left_err = self.revs_to_feet(self._left_motor.getPosition() - self._left_motor.getSetpoint())
        right_err = self.revs_to_feet(self._right_motor.getPosition() - self._right_motor.getSetpoint())
        return abs(left_err + right_err) / 2 < margin

    def is_manual_control_mode(self):
        return self._mode in (SmartDrivetrain.Mode.PercentVbus, SmartDrivetrain.Mode.Speed)

    def getDescription(self):
        return "SmartDrivetrain"

    def stopMotor(self):
        self._left_motor.stopMotor()
        self._right_motor.stopMotor()
        self.feed()
