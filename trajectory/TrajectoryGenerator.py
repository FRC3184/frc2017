import math

from trajectory.Trajectory import Trajectory, Segment


class Config:
    def __init__(self, dt=0, max_vel=0, max_acc=0, max_jerk=0):
        self.dt = dt
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_jerk = max_jerk


class Strategy:
    StepStrategy = 0
    TrapezoidalStrategy = 1
    SCurvesStrategy = 2
    AutomaticStrategy = 3


class IntegrationMethod:
    RectangularIntegration = 0
    TrapezoidalIntegration = 1


def choose_strategy(start_vel, goal_vel, max_vel):
    if start_vel == goal_vel and start_vel == max_vel:
        return Strategy.StepStrategy
    elif start_vel == goal_vel and start_vel == 0:
        return Strategy.SCurvesStrategy
    else:
        return Strategy.TrapezoidalStrategy


def second_order_filter(f1_len, f2_len, dt, start_vel, max_vel, total_impulse, length, integration):
    if length < 1:
        return None
    trajectory = Trajectory(length)
    last = Segment()
    last.pos = 0
    last.vel = start_vel
    last.acc = 0
    last.jerk = 0
    last.dt = dt

    f1 = [0] * length
    f1[0] = ((start_vel / max_vel) * f1_len)
    f2 = 0
    for i in range(length):
        current = trajectory.segments[i]
        input = min(total_impulse, 1)
        if input < 1:
            input -= 1
            total_impulse = 0
        else:
            total_impulse -= input
        f1_last = 0
        if i > 0:
            f1_last = f1[i - 1]
        else:
            f1_last = f1[0]
        f1[i] = max(0, min(f1_len, f1_last + input))

        f2 = 0
        for j in range(f2_len):
            if i > j:
                break
            f2 += f1[i - j]
        f2 /= f1_len

        current.vel = f2 / f2_len * max_vel
        if integration == IntegrationMethod.RectangularIntegration:
            current.pos = current.vel * dt + last.pos
        elif integration == IntegrationMethod.TrapezoidalIntegration:
            current.pos = (last.vel + current.vel) / 2 * dt + last.pos
        current.x = current.pos
        current.y = 0

        current.acc = (current.vel - last.vel) / dt
        current.jerk = (current.jerk - last.jerk) / dt
        current.dt = dt
        last = current
    return trajectory


def generate_step_strategy(config, start_vel, start_heading, goal_pos, goal_vel, goal_heading):
    impulse = (goal_pos / config.max_vel) / config.dt
    time = math.floor(impulse)
    return second_order_filter(1, 1, config.dt, config.max_vel, config.max_vel, impulse, time,
                               IntegrationMethod.TrapezoidalIntegration)


def generate_trapezoidal_strategy(config, start_vel, start_heading, goal_pos, goal_vel, goal_heading):
    start_discount = 0.5 * start_vel**2 / config.max_acc
    end_discount = 0.5 * goal_vel**2 / config.max_acc

    adj_max_vel = min(config.max_vel, (config.max_acc * goal_pos - start_discount - end_discount)**0.5)

    t_rampup = (adj_max_vel - start_vel) / config.max_acc
    x_rampup = start_vel * t_rampup + .5 * config.max_acc * t_rampup**2

    t_rampdown = (adj_max_vel - goal_vel) / config.max_acc
    x_rampdown = adj_max_vel * t_rampdown - .5 * config.max_acc * t_rampdown**2

    x_cruise = goal_pos - x_rampdown - x_rampup

    time = round((t_rampup + t_rampdown + x_cruise / adj_max_vel) / config.dt)
    f1_length = math.ceil((adj_max_vel / config.max_acc) / config.dt)
    impulse = (goal_pos / adj_max_vel) / config.dt - \
              start_vel / config.max_acc / config.dt + \
              start_discount + end_discount

    return second_order_filter(f1_length, 1, config.dt, start_vel, adj_max_vel, impulse, time,
                               IntegrationMethod.TrapezoidalIntegration)


def generate_scurves_strategy(config, start_vel, start_heading, goal_pos, goal_vel, goal_heading):
    adj_max_vel = min(config.max_vel,
                      (-(config.max_acc**2) +
                       (config.max_acc**4 +
                        4 * config.max_jerk**2 * config.max_acc * goal_pos)**0.5) /
                      (2 * config.max_jerk))
    f1_len = math.ceil((adj_max_vel / config.max_acc) / config.dt)
    f2_len = math.ceil((config.max_acc / config.max_jerk) / config.dt)
    impulse = (goal_pos / adj_max_vel) / config.dt
    time = math.ceil(f1_len + f2_len + impulse)
    return second_order_filter(f1_len, f2_len, config.dt, 0, adj_max_vel, impulse, time,
                               IntegrationMethod.TrapezoidalIntegration)


def assign_headings(start_heading, goal_heading, trajectory):
    total_heading_change = goal_heading - start_heading
    for segment in trajectory.segments:
        segment.heading = start_heading + total_heading_change * segment.pos / trajectory.segments[-1].pos


def generate(config, strategy, start_vel, start_heading, goal_pos, goal_vel, goal_heading):
    if strategy == Strategy.AutomaticStrategy:
        strategy = choose_strategy(start_vel, goal_vel, config.max_vel)
    if strategy == Strategy.SCurvesStrategy:
        trajectory = generate_scurves_strategy(config, start_vel, start_heading, goal_pos, goal_vel, goal_heading)
    elif strategy == Strategy.TrapezoidalStrategy:
        trajectory = generate_trapezoidal_strategy(config, start_vel, start_heading, goal_pos, goal_vel, goal_heading)
    elif strategy == Strategy.StepStrategy:
        trajectory = generate_step_strategy(config, start_vel, start_heading, goal_pos, goal_vel, goal_heading)
    else:
        return None
    assign_headings(start_heading, goal_heading, trajectory)
    return trajectory
