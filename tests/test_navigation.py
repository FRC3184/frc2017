import navigation


def test_trajectory_long():
    # Test trapezoidal profile
    dist = 10
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-3
    assert max(map(lambda x: x.vel, points)) == cruise


def test_trajectory_short():
    # Test triangular profile
    dist = 3
    cruise = 2
    acc = 1
    points = navigation.straight_trajectory(dist, cruise, acc)
    assert abs(points[-1].pos - dist) < 1e-3
    assert abs(points[-1].vel) < 1e-2  # Give some leeway for floating points
