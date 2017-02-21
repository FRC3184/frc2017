import time
import wpilib

import wpy.driverstation

_auto_time = 15  # Seconds in auto period
_teleop_time = 2 * 60 + 15  # Seconds in teleop period
_endgame_time = 30


def sleep(millis=None, seconds=None):
    if millis is None and seconds is None:
        raise ValueError("Provide millis or seconds")
    if millis is not None:
        time.sleep(millis / 1000)
    if seconds is not None:
        time.sleep(seconds)


def get_period_remaining_time():
    return wpy.driverstation.get_match_time()


def get_match_time():
    if wpilib.DriverStation.getInstance().isAutonomous():
        return _auto_time - get_period_remaining_time()
    elif wpilib.DriverStation.getInstance().isOperatorControl():
        return _teleop_time - get_period_remaining_time()
    elif wpilib.DriverStation.getInstance().isDisabled():
        return 0


def is_endgame():
    return wpilib.DriverStation.getInstance().isOperatorControl() and get_period_remaining_time() < _endgame_time
