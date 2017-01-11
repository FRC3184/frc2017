from trajectory.Trajectory import Trajectory


class TrajectoryFollower:
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.kv = 0
        self.ka = 0
        self.last_error = 0
        self.current_heading = 0
        self.current_segment = 0
        self.profile = None

    def configure(self, kp, ki, kd, kv, ka):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kv = kv
        self.ka = ka

    def reset(self):
        self.last_error = 0
        self.current_segment = 0

    def set_trajectory(self, profile):
        """

        :param profile:
        :type profile: Trajectory
        :return:
        """
        self.profile = profile

    def calculate(self, dist_so_far):
        if self.current_segment < self.profile.get_num_segments():
            segment = self.profile.get_segment(self.current_segment)
            error = segment.pos - dist_so_far
            output = self.kp * error + \
                     self.kd * ((error - self.last_error) / segment.dt - segment.vel) + \
                     self.kv * segment.vel + \
                     self.ka * segment.acc
            self.last_error = error
            self.current_heading = segment.heading
            self.current_segment += 1
            return output
        return 0

    @property
    def get_heading(self):
        return self.current_heading

    def has_finished(self):
        return self.current_segment >= self.profile.get_num_segments()