import math

import mathutils


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

    def __str__(self):
        return "Point(x={}, y={})".format(self.x, self.y)


class Waypoint(_Point):
    def __init__(self, x, y, heading):
        super().__init__(x, y)
        self.heading = heading

    def __str__(self):
        return "Waypoint(x={}, y={}, theta={})".format(self.x, self.y, self.heading)


def _calc(point1, knot1, knot2, point2):
    # Chris made this in desmos. I don't know how it works. Don't ask me.
    # https://www.desmos.com/calculator/yri3mwz7kn
    sgn_theta1 = -mathutils.sgn((knot2.y - knot1.y)/(knot2.x - knot1.x) * (point1.x - knot1.x) + knot1.y - point1.y)
    num = knot2.dx2(point1) + knot2.dy2(point1) - knot1.dx2(point1) - knot1.dy2(point1) - knot1.dx2(knot2) - \
          knot1.dy2(knot2)
    angle_theta1 = math.acos(num / (-2 * knot1.dist(knot2) * knot1.dist(point1)))
    theta1 = sgn_theta1 * angle_theta1

    sgn_theta2 = -mathutils.sgn((knot2.y - knot1.y) / (knot2.x - knot1.x) * (point2.x - knot2.x) + knot2.y - point2.y)
    num = knot1.dx2(point2) + knot1.dy2(point2) - knot2.dx2(point2) - knot2.dy2(point2) - knot1.dx2(knot2) - \
          knot1.dy2(knot2)
    print(num / (-2 * knot1.dist(knot2) * knot1.dist(point1)))
    angle_theta2 = math.acos(num / (-2 * knot1.dist(knot2) * knot2.dist(point2))) - math.pi
    theta2 = sgn_theta2 * angle_theta2

    d = knot1.dist(knot2)
    print(mathutils.normalize_angle(theta1))
    print(mathutils.normalize_angle(theta2))
    return mathutils.normalize_angle(theta1), d, mathutils.normalize_angle(theta2)


def navigate(waypoint1, waypoint2):
    """
    Calculate a path between two points.
    :param waypoint1:
    :param waypoint2:
    :return: (Angle of first turn, distance, angle of second turn)
    """
    r_point1 = _Point(waypoint1.x + math.tan(waypoint1.heading), waypoint1.y + 1)
    r_point2 = _Point(waypoint2.x + math.tan(waypoint2.heading), waypoint2.y + 1)
    return _calc(r_point1, waypoint1, waypoint2, r_point2)


class TrajectoryPoint:
    def __init__(self, time, pos, vel, acc):
        self.time = time
        self.pos = pos
        self.vel = vel
        self.acc = acc

    def __str__(self):
        return "{}\t{}\t{}\t{}".format(self.time, self.pos, self.vel, self.acc)


def straight_trajectory(dist, cruise_speed, acc, frequency=100):
    # https://www.desmos.com/calculator/ponjr7cwze

    ramp_time = cruise_speed / acc
    ramp_dist = acc * ramp_time**2 / 2
    cruise_dist = dist - 2 * ramp_dist
    cruise_time = cruise_dist / cruise_speed

    if cruise_dist < 0:
        # All ramp, no cruise. Fix parameters to match
        cruise_time = 0
        cruise_dist = 0
        ramp_time = (dist / acc)**0.5
        ramp_dist = dist / 2

    time = cruise_time + 2 * ramp_time

    def get_pos(t):
        if t <= ramp_time:
            return acc * t**2 / 2
        elif t <= ramp_time + cruise_time:
            return ramp_dist + (t - ramp_time) * get_vel(ramp_time)
        else:
            tp = (t - ramp_time - cruise_time)
            return ramp_dist + cruise_dist + get_vel(ramp_time + cruise_time) * tp - acc * tp**2 / 2

    def get_vel(t):
        if t <= ramp_time:
            return get_acc(t) * t
        elif t <= ramp_time + cruise_time:
            return cruise_speed
        else:
            tp = (t - ramp_time - cruise_time)
            return get_vel(ramp_time + cruise_time) - acc * tp

    def get_acc(t):
        if t <= ramp_time:
            return acc
        elif t <= ramp_time + cruise_time:
            return 0
        else:
            return -acc

    points = []
    for i in range(int(time * frequency) + 1):
        t = i / frequency
        points.append(TrajectoryPoint(t, get_pos(t), get_vel(t), get_acc(t)))
    return points, time, 1 / frequency


def radius_trajectory(radius, angle, track_width, cruise_speed, acc, frequency=100):

    D = track_width / 2
    ratio = (radius - D) / (radius + D)

    outer_dist = (radius + D) * angle
    inner_dist = (radius - D) * angle

    outer_speed = cruise_speed
    inner_speed = cruise_speed * ratio

    outer_acc = acc
    inner_acc = acc * ratio

    outer_points, total_time, _ = straight_trajectory(outer_dist, outer_speed, outer_acc, frequency=frequency)
    inner_points, _, _ = straight_trajectory(inner_dist, inner_speed, inner_acc, frequency=frequency)
    return outer_points, inner_points, total_time, 1/frequency
