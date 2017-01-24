import navigation


def test_trajectory_long():
    dist = 10
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-3
    assert max(map(lambda x: x.vel, points)) == cruise


def test_trajectory_short():
    dist = 3
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-3
