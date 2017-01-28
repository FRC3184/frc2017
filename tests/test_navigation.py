import mathutils
import navigation
import math


def test_trajectory_long():
    # Test trapezoidal profile
    dist = 10
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-3
    assert max(map(lambda x: x.vel, points)) == cruise


def test_trajectory_triangle():
    dist = 4
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-3
    assert max(map(lambda x: x.vel, points)) == cruise


def test_trajectory_long2():
    dist = 20
    cruise = 2
    acc = 2
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-3
    assert max(map(lambda x: x.vel, points)) == cruise


def test_trajectory_short():
    # Test short triangular profile
    dist = 3
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-2  # Give some leeway for floating points


def test_heading_calc1():
    wp1 = navigation.Waypoint(0, 0, math.pi / 4)
    wp2 = navigation.Waypoint(5, 5, math.pi / 4)
    turn1, dist, turn2 = navigation.navigate(wp1, wp2)
    assert abs(dist - 50**0.5) < 1e-3
    assert abs(turn1) < 1e-3
    assert abs(turn2) < 1e-3


def test_heading_calc2():
    wp1 = navigation.Waypoint(0, 0, 0)
    wp2 = navigation.Waypoint(5, 5, math.pi / 4)
    turn1, dist, turn2 = navigation.navigate(wp1, wp2)
    assert abs(dist - 50**0.5) < 1e-3
    assert abs(mathutils.angle_difference(turn1, math.pi / 4)) < 1e-3
    assert abs(turn2) < 1e-3


def test_heading_calc3():
    wp1 = navigation.Waypoint(0, 0, 0)
    wp2 = navigation.Waypoint(5, 5, 0)
    turn1, dist, turn2 = navigation.navigate(wp1, wp2)
    assert abs(dist - 50**0.5) < 1e-3
    assert abs(mathutils.angle_difference(turn1, math.pi / 4)) < 1e-3
    assert abs(mathutils.angle_difference(turn2, -math.pi / 4)) < 1e-3
