from trajectory import TrajectoryGenerator
from trajectory.Path import Path
from trajectory.Spline import Spline
from trajectory.Trajectory import Trajectory, TrajectoryPair
import math


def generate_from_path(path, config):
    if path.num_waypoints < 2:
        return None
    splines = []
    spline_lens = []
    total_distance = 0
    for i in range(path.num_waypoints - 1):
        spline = Spline(path.waypoints[i], path.waypoints[i + 1], Spline.Type.QuinticHermite)
        splines.append(spline)
        spline_len = spline.calculate_length()
        spline_lens.append(spline_len)
        total_distance += spline_len

    trajectory = TrajectoryGenerator.generate(config,
                                              TrajectoryGenerator.Strategy.SCurvesStrategy,
                                              0, path.waypoints[0].theta, total_distance, 0,
                                              path.waypoints[0].theta)
    cur_spline = 0
    cur_spline_start_pos = 0
    length_of_splines_finished = 0
    for i in range(trajectory.get_num_segments()):
        segment = trajectory.get_segment(i)
        cur_pos = segment.pos
        found_spline = False
        while not found_spline:
            cur_pos_relative = cur_pos - cur_spline_start_pos
            if cur_pos_relative <= spline_lens[cur_spline]:
                percentage = splines[cur_spline].get_percentage_for_dist(cur_pos_relative)
                segment.heading = splines[cur_spline].angle_at(percentage)
                x, y = splines[cur_spline].get_point(percentage)
                segment.x = x
                segment.y = y
                found_spline = True
            elif cur_spline < len(splines) - 1:
                length_of_splines_finished += spline_lens[cur_spline]
                cur_spline += 1
            else:
                segment.heading = splines[-1].angle_at(1)
                x, y = splines[-1].get_point(1)
                segment.x = x
                segment.y = y
                found_spline = True

    return trajectory


def make_left_and_right_trajectories(input_traj, wheelbase_width):
    left = input_traj.copy()
    right = input_traj.copy()
    
    for i in range(input_traj.get_num_segments()):
        current = input_traj.get_segment(i)
        cos_angle = math.cos(current.heading)
        sin_angle = math.sin(current.heading)
        
        s_left = left.get_segment(i)
        s_left.x = current.x - wheelbase_width / 2 * sin_angle
        s_left.y = current.y + wheelbase_width / 2 * cos_angle
        if i > 0:
            dist = ((s_left.x - left.get_segment(i - 1).x) ** 2 +
                    (s_left.y - left.get_segment(i - 1).y) ** 2) ** 0.5
            s_left.pos = left.get_segment(i - 1).pos + dist
            s_left.vel = dist / s_left.dt
            s_left.acc = (s_left.vel - left.getSegment(i - 1).vel) / s_left.dt
            s_left.jerk = (s_left.acc - left.getSegment(i - 1).acc) / s_left.dt

        s_right = right.get_segment(i)
        s_right.x = current.x + wheelbase_width / 2 * sin_angle
        s_right.y = current.y - wheelbase_width / 2 * cos_angle
        if i > 0:
            dist = ((s_right.x - right.get_segment(i - 1).x) ** 2 +
                    (s_right.y - right.get_segment(i - 1).y) ** 2) ** 0.5
            s_right.pos = right.get_segment(i - 1).pos + dist
            s_right.vel = dist / s_right.dt
            s_right.acc = (s_right.vel - right.getSegment(i - 1).vel) / s_right.dt
            s_right.jerk = (s_right.acc - right.getSegment(i - 1).acc) / s_right.dt
    return TrajectoryPair(left, right)


def generate_left_and_right_from_seq(waypoints, config, wheelbase_width):
    return make_left_and_right_trajectories(generate_from_path(waypoints, config), wheelbase_width)


def make_path(waypoints, config, wheelbase_width, name):
    return Path(name, generate_left_and_right_from_seq(waypoints, config, wheelbase_width))