import trajectory.MathUtil as MathUtil
import math


class Waypoint:
    def __init__(self, *args):
        if len(args) == 1:
            src = args[0]
            self.x = src.x
            self.y = src.y
            self.theta = src.theta
        else:
            self.x = args[0]
            self.y = args[1]
            self.theta = args[2]


class WaypointSequence:
    def __init__(self, max_size):
        self.waypoints = [None] * max_size
        self.num_waypoints = 0

    def add_waypoint(self, w):
        if self.num_waypoints < len(self.waypoints):
            self.waypoints[self.num_waypoints] = w
            self.num_waypoints += 1

    def invert_y(self):
        inverted = WaypointSequence(len(self.waypoints))
        for waypoint in self.waypoints:
            cloned = Waypoint(waypoint)
            cloned.y *= -1
            cloned.theta = MathUtil.normalize_angle_radians(2 * math.pi - cloned.theta)
            inverted.add_waypoint(cloned)
