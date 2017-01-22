import math


def _sgn(x):
    return 0 if x == 0 else x / abs(x)


class _Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def dist(self, other):
        return ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5

    def dx2(self, other):
        return (self.x - other.x)**2

    def dy2(self, other):
        return (self.y - other.y)**2


class Waypoint(_Point):
    def __init__(self, x, y, heading):
        super().__init__(x, y)
        self.heading = heading


def _calc(point1, knot1, knot2, point2):
    sgn_theta1 = -_sgn((knot2.y - knot1.y)/(knot2.x - knot1.x) * (point1.x - knot1.x) + knot1.y - point1.y)
    num = knot2.dx2(point1) + knot2.dy2(point1) - knot1.dx2(point1) - knot1.dy2(point1) - knot1.dx2(knot2) - \
          knot1.dy2(knot2)
    angle_theta1 = math.acos(num / (-2 * knot1.dist(knot2) * knot1.dist(point1)))
    theta1 = sgn_theta1 * angle_theta1

    sgn_theta2 = _sgn((knot2.y - knot1.y) / (knot2.x - knot1.x) * (point2.x - knot2.x) + knot2.y - point2.y)
    num = knot1.dx2(point2) + knot1.dy2(point2) - knot2.dx2(point2) - knot2.dy2(point2) - knot1.dx2(knot2) - \
          knot1.dy2(knot2)
    angle_theta2 = math.acos(num / (-2 * knot1.dist(knot2) * knot1.dist(point1)))
    theta2 = sgn_theta2 * angle_theta2

    d = knot1.dist(knot2)
    return theta1, d, theta2


def navigate(waypoint1, waypoint2):
    r_point1 = _Point(waypoint1.x + math.tan(waypoint1.heading), waypoint1.y + 1)
    r_point2 = _Point(waypoint2.x + math.tan(waypoint2.heading), waypoint2.y + 1)
    return _calc(r_point1, waypoint1, waypoint2, r_point2)


def straight_trajectory(dist, max_speed, acc):
    ramp_time = max_speed / acc
