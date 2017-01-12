import math

from trajectory import MathUtil
from trajectory.Spline import Spline, SplineException


def _do_spline_test(x0, y0, theta0, x1, y1, theta1, is_straight):
    for s_type in (Spline.Type.CubicHermite, Spline.Type.QuinticHermite):
        s = Spline(x0, y0, theta0, x1, y1, theta1, s_type)
        print(s)
        for t in range(0, 105, 5):
            t /= 100
            print("{}, {}, {}".format(t, s.value_at(t), s.angle_at(t)))
        print("Checking endpoint values...")
        assert MathUtil.almost_equal(s.value_at(0), y0)
        assert MathUtil.almost_equal(s.value_at(1), y1)
        print("Checking endpoint angles...")
        assert MathUtil.almost_equal(MathUtil.normalize_half_angle_radians(s.angle_at(0)),
                                     MathUtil.normalize_half_angle_radians(theta0))
        assert MathUtil.almost_equal(MathUtil.normalize_half_angle_radians(s.angle_at(1)),
                                     MathUtil.normalize_half_angle_radians(theta1))
        if is_straight:
            print("Spline is straight, verifying length...")
            expected = ((x1 - x0)**2 + (y1 - y0)**2)**0.5
            assert MathUtil.almost_equal(expected, s.calculate_length())

        if s_type == Spline.Type.QuinticHermite:
            print("Checking 2nd derivatives...")
            assert MathUtil.almost_equal(0, s.angle_change_at(0))
            assert MathUtil.almost_equal(0, s.angle_change_at(1))


def _do_spline_check_fail(x0, y0, theta0, x1, y1, theta1):
    result = False
    try:
        s = Spline(x0, y0, theta0, x1, y1, theta1, Spline.Type.CubicHermite)
    except SplineException:
        result = True
    assert result


def test_spline_unit_lines():
    _do_spline_test(0, 0, 0, 1, 0, 0, True)
    _do_spline_test(0, 0, math.pi / 2, 0, 1, math.pi / 2, True)
    _do_spline_test(0, 0, math.pi, -1, 0, math.pi, True)
    _do_spline_test(0, 0, -math.pi / 2, 0, -1, -math.pi / 2, True)
    _do_spline_test(0, 0, math.pi / 4, 2**0.5 / 2, 2**0.5 / 2, math.pi / 4, True)


def test_spline_non_unit_lines():
    _do_spline_test(0, 0, 0, 5, 0, 0, True)
    _do_spline_test(0, 0, math.pi / 4, 2**0.5, 2**0.5, math.pi / 4, True)


def test_translated_unit_lines():
    _do_spline_test(1, 1, 0, 2, 1, 0, True)
    _do_spline_test(1, 1, math.pi / 4, 2**0.5 / 2 + 1, 2**0.5 / 2 + 1, math.pi / 4,
                    True)


def test_translated_non_unit_lines():
    _do_spline_test(1, 1, 0, 6, 1, 0, True)
    _do_spline_test(1, 1, math.pi / 4, 2**0.5 + 1, 2**0.5 + 1, math.pi / 4,
                    True)


def test_unit_step():
    _do_spline_test(0, 0, 0, 2**0.5 / 2, 2**0.5 / 2, 0, False)


def test_non_unit_step():
    _do_spline_test(0, 0, 0, 10, 20, 0, False)


def test_translated_step():
    _do_spline_test(0, 5, 0, 10, 20, 0, False)


def test_rotated_step():
    _do_spline_test(0, 0, math.pi / 4, 0, 1, math.pi / 4, False)


def test_invalid_input():
    _do_spline_check_fail(0, 0, 0, 0, 0, 0)
    _do_spline_check_fail(0, 0, -math.pi / 2, 1, 0, 0)


def test_problematic_scurve():
    _do_spline_test(0, 0, 0, 3, 4, math.pi / 4, False)


def test_regression_quintic():
    _do_spline_test(0, 0, 0, 10, 5, 0, False)
    _do_spline_test(10, 5, 0, 30, -5, 0, False)
    _do_spline_test(30, -5, 0, 40, 0, 0, False)