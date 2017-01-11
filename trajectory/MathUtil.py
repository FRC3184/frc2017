import math


def sgn(x):
    return abs(x)/x if x != 0 else 0


def almost_equal(a, b, epsilon=1e-6):
    return abs(a - b) < epsilon


def bound(x, lower, upper):
    if x < lower:
        return lower
    if x > upper:
        return upper
    return x


def normalize_angle_radians(angle):
    return angle % (2 * math.pi)


def normalize_half_angle_radians(angle):
    theta = normalize_angle_radians(angle)
    if theta > math.pi:
        theta -= 2 * math.pi
    return theta


def radian_angle_difference(angle_from, angle_to):
    return normalize_half_angle_radians(angle_to - angle_from)

