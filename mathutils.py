import math


def sgn(x):
    return 0 if x == 0 else x/abs(x)


def normalize_angle(theta):
    """
    Normalize angle to +/- pi/2
    :param theta: The angle to normalize, in radians
    :return: The normalized angle
    """
    return math.atan2(math.sin(theta), math.cos(theta))


def angle_difference(theta1, theta2):
    return normalize_angle(theta1 - theta2)
