import math

import pose


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


def signed_power(x, pow):
    s = sgn(x)
    return s * abs(x)**pow


def deadband(x, deadband):
    if abs(x) < deadband:
        return 0
    return x


def clamp(x, vmin, vmax):
    return min(max(x, vmin), vmax)
  
  
class Vector2:
    """
    A 2D vector. x,y are in world coordinates
    """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def translated(self, pose):
        dx = self.x - pose.x
        dy = self.y - pose.y
        c = math.cos(pose.heading)
        s = math.sin(pose.heading)
        dxp = dx * c + dy * s
        dyp = dx * -s + dy * c
        return Vector2(dxp, dyp)

    def inv_translate(self, pose):
        c = math.cos(pose.heading)
        s = math.sin(pose.heading)
        dxp = self.x * c - self.y * s
        dyp = self.x * s + self.y * c
        return Vector2(dxp + pose.x, dyp + pose.y)

    def distance(self, point):
        return ((point.x - self.x)**2 + (point.y - self.y)**2)**0.5

    def normalized(self):
        magn = abs(self)
        return Vector2(self.x / magn, self.y / magn)

    def __mul__(self, other):
        if type(other) == Vector2:
            return self.x * other.x + self.y * other.y
        else:
            return Vector2(self.x * other, self.y * other)

    def __repr__(self):
        return "Vector({}, {})".format(self.x, self.y)

    def __sub__(self, other):
        assert type(other) == Vector2
        return Vector2(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        assert type(other) == Vector2 or type(other) == pose.Pose
        return Vector2(self.x + other.x, self.y + other.y)

    def __abs__(self):
        return self.distance(Vector2(0, 0))

    def __copy__(self):
        return Vector2(self.x, self.y)

    def __neg__(self):
        return self * -1
