import math
import trajectory.MathUtil as MathUtil


class SplineException(ValueError):
    pass


class Spline:
    class Type:
        QuinticHermite = 0
        CubicHermite = 1

    def __init__(self, *args):
        self.arc_length = None
        self.x_offset = None
        self.y_offset = None
        self.type = None
        self.knot_distance = None
        self.theta_offset = None
        self.a = None
        self.b = None
        self.c = None
        self.d = None
        self.e = None
        if len(args) == 3:
            self.reticulate_waypoints(*args)
        elif len(args) == 7:
            self.reticulate_raw(*args)
        else:
            raise ValueError("Invalid arguments given")

    def reticulate_waypoints(self, wp_begin, wp_end, type):
        self.reticulate_raw(wp_begin.x, wp_begin.y, wp_begin.theta,
                            wp_end.x, wp_end.y, wp_end.theta, type)

    def reticulate_raw(self, x0, y0, theta0, x1, y1, theta1, type):
        self.arc_length = None
        self.x_offset = x0
        self.y_offset = y0
        self.type = type
        dist = ((x1 - x0)**2 + (y1 - y0)**2)**0.5
        if dist == 0:
            raise SplineException("Zero length path")
        self.knot_distance = dist
        self.theta_offset = math.atan2(y1 - y0, x1 - x0)
        theta0_hat = MathUtil.radian_angle_difference(self.theta_offset, theta0)
        theta1_hat = MathUtil.radian_angle_difference(self.theta_offset, theta1)

        if MathUtil.almost_equal(abs(theta0_hat), math.pi / 2) or MathUtil.almost_equal(abs(theta1_hat), math.pi / 2):
            raise SplineException("Vertical slope")
        if abs(MathUtil.radian_angle_difference(theta0_hat, theta1_hat)) >= math.pi / 2:
            raise SplineException("Turn greater than 90*")
        yp0_hat = math.tan(theta0_hat)
        yp1_hat = math.tan(theta1_hat)
        if type == Spline.Type.CubicHermite:
            self.a = 0
            self.b = 0
            self.c = (yp1_hat + yp0_hat) / (dist**2)
            self.d = -(2 * yp0_hat + yp1_hat) / dist
            self.e = yp0_hat
        elif type == Spline.Type.QuinticHermite:
            self.a = -(3 * (yp0_hat + yp1_hat)) / (dist**4)
            self.b = (8 * yp0_hat + 7 * yp1_hat) / (dist**3)
            self.c = -(6 * yp0_hat + 4 * yp1_hat) / (dist**2)
            self.d = 0
            self.e = yp0_hat
        else:
            raise SplineException("Invalid type given")

    def calculate_length(self):
        if self.arc_length is not None:
            return self.arc_length
        k_num_samples = 10000
        last_integrand = ((1 + self.derivative_at(0)**2)**0.5) / k_num_samples
        arc_length = 0

        for i in range(1, k_num_samples+1):
            t = i / k_num_samples
            dydt = self.derivative_at(t)
            integrand = ((1 + dydt**2)**0.5) / k_num_samples
            arc_length += (integrand + last_integrand) / 2
            last_integrand = integrand
        self.arc_length = arc_length * self.knot_distance
        return self.arc_length

    def get_percentage_for_dist(self, distance):
        k_num_samples = 10000
        last_integrand = ((1 + self.derivative_at(0) ** 2) ** 0.5) / k_num_samples
        arc_length = 0
        last_arc_length = 0
        t = 0

        for i in range(1, k_num_samples + 1):
            t = i / k_num_samples
            dydt = self.derivative_at(t)
            integrand = ((1 + dydt ** 2) ** 0.5) / k_num_samples
            arc_length += (integrand + last_integrand) / 2
            if arc_length > distance:
                break
            last_integrand = integrand
            last_arc_length = arc_length

        interpolated = t
        if arc_length != last_arc_length:
            interpolated += ((distance - last_arc_length) / (arc_length - last_arc_length) - 1) / k_num_samples
        return interpolated

    def get_point(self, percentage):
        percentage = MathUtil.bound(percentage, 0, 1)
        x = percentage * self.knot_distance
        y = self.a * x**5 + self.b * x**4 + self.c * x**3 + self.d * x**2 + self.e * x
        cos_theta = math.cos(self.theta_offset)
        sin_theta = math.sin(self.theta_offset)

        res_x = x * cos_theta - y * sin_theta + self.x_offset
        res_y = x * sin_theta + y * cos_theta + self.y_offset
        return res_x, res_y

    def value_at(self, percentage):
        percentage = MathUtil.bound(percentage, 0, 1)
        x = percentage * self.knot_distance
        y = self.a * x ** 5 + self.b * x ** 4 + self.c * x ** 3 + self.d * x ** 2 + self.e * x
        cos_theta = math.cos(self.theta_offset)
        sin_theta = math.sin(self.theta_offset)

        res_y = x * sin_theta + y * cos_theta + self.y_offset
        return res_y

    def derivative_at(self, percentage):
        percentage = MathUtil.bound(percentage, 0, 1)
        x_hat = percentage * self.knot_distance
        return 5 * self.a * x_hat**4 + 4 * self.b * x_hat**3 + 3 * self.c * x_hat**2 + 2 * self.d * x_hat + self.e

    def second_derivative_at(self, percentage):
        percentage = MathUtil.bound(percentage, 0, 1)
        x = percentage * self.knot_distance
        return 20 * self.a * x**3 + 12 * self.b * x**2 + 6 * self.c * x + 2 * self.d

    def angle_at(self, percentage):
        return MathUtil.normalize_angle_radians(math.atan(self.derivative_at(percentage)) + self.theta_offset)

    def angle_change_at(self, percentage):
        return MathUtil.normalize_half_angle_radians(math.atan(self.second_derivative_at(percentage)))

    def __str__(self):
        return "{a}x^5 + {b}x^4 + {c}x^3 + {d}x^2 + {e}x".format(a=self.a,
                                                                 b=self.b,
                                                                 c=self.c,
                                                                 d=self.d,
                                                                 e=self.e)
