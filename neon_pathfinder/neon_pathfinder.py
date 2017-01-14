from ctypes import *
import time

cdll.LoadLibrary("libpathfinder_arm.so")  # TODO Make sure neon_pathfinder is in PATH

neon_pathfinder = CDLL("libpathfinder_arm.so")

TRAPEZOID = 0
SCURVE = 1


class Segment1D(Structure):
    _fields_ = [("distance", c_float),
                ("velocity", c_float),
                ("acceleration", c_float),
                ("jerk", c_float)]


class Segment2D_Ext(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("heading", c_float)]


class Segment:
    def __init__(self, seg, seg_ext):
        self.distance = seg.distance
        self.velocity = seg.velocity
        self.acceleration = seg.acceleration
        self.jerk = seg.jerk
        self.x = seg_ext.x
        self.y = seg_ext.y
        self.angle = seg_ext.angle


class Waypoint2D(Structure):
    _fields_ = [("x", c_float), ("y", c_float), ("angle", c_float)]


class Spline2D(Structure):
    _fields_ = [("a", c_float), ("b", c_float), ("c", c_float), ("d", c_float), ("e", c_float),
                ("x_offset", c_float), ("y_offset", c_float), ("angle_offset", c_float), ("knot_distance", c_float),
                ("arc_length", c_float)]


def trapezoid_linear(dt, dist, acc, vel):
    dt = c_float(dt)
    dist = c_float(dist)
    acc = c_float(acc)
    vel = c_float(vel)

    segments = (Segment1D * 8192)()
    # Only C works. ASM generates a segfault.
    neon_pathfinder.pf_trapezoid_generate_c(byref(segments), dt, dist, acc, vel)
    return segments


def scurve_linear(dt, dist, vel, acc, jerk):
    dt = c_float(dt)
    dist = c_float(dist)
    acc = c_float(acc)
    vel = c_float(vel)

    segments = (Segment1D * 8192)()
    neon_pathfinder.pf_scurve_generate(byref(segments), dt, dist, jerk, acc, vel)
    return segments


def trajectory(waypoints, dt, vel, acc, jerk, curve_type=TRAPEZOID, samples=10000):
    dt = c_float(dt)
    vel = c_float(vel)
    acc = c_float(acc)
    jerk = c_float(jerk)

    _waypoints = (Waypoint2D * len(waypoints))()
    for i in range(len(waypoints)):
        _waypoints[i] = waypoints[i]

    segments = (Segment1D * 8192)()
    segments_ext = (Segment2D_Ext * 8192)()

    neon_pathfinder.pf_generate_trajectory(segments, segments_ext, curve_type, _waypoints, len(waypoints), samples,
                                           dt, vel, acc, jerk, 0)  # 0 for C, 1 for ASM

    real_segments = []
    for i in range(len(waypoints)):
        real_segments += [Segment(segments[i], segments_ext[i])]
    return real_segments


if __name__ == '__main__':
    dt = 0.001
    dist = 10
    vel = 2
    acc = 2
    jerk = 10
    #scurve_linear(dt, dist, vel, acc, jerk)
    #print("Finished scurve")
    waypoints = [Waypoint2D(-4, -1, -3.1415/4), Waypoint2D(-1, 2, 0), Waypoint2D(2,4, 0)]
    trajectory(waypoints, dt, vel, acc, jerk)