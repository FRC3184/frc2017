

class Segment:
    def __init__(self, *args, **kwargs):
        if len(args) == 0:
            # ¯\_(ツ)_/¯
            self.pos = 0
            self.vel = 0
            self.acc = 0
            self.jerk = 0
            self.heading = 0
            self.dt = 0
            self.x = 0
            self.y = 0
        elif len(args) == 1:
            src = args[0]
            self.pos = src.pos
            self.vel = src.vel
            self.acc = src.acc
            self.jerk = src.jerk
            self.heading = src.heading
            self.dt = src.dt
            self.x = src.x
            self.y = src.y
        else:
            self.pos, self.vel, self.acc, self.jerk, self.heading, self.dt, self.x, self.y = args

    def __str__(self):
        return super().__str__()  # TODO


class TrajectoryPair:
    def __init__(self, left, right):
        self.left = left
        self.right = right


class Trajectory:
    def __init__(self, length):
        self.segments = []
        for i in range(length):
            self.segments.append(Segment())
        self.inverted_y = False

    def get_num_segments(self):
        return len(self.segments)

    def get_segment(self, index):
        if index < self.get_num_segments():
            if self.inverted_y:
                inverted_segment = Segment(self.segments[index])
                inverted_segment.y *= -1
                inverted_segment.heading *= -1
                return inverted_segment
            return self.segments[index]
        return Segment()

    def scale(self, factor):
        for segment in self.segments:
            segment.pos *= factor
            segment.vel *= factor
            segment.acc *= factor
            segment.jerk *= factor

    def append(self, other):
        self.segments += other.segments

    def copy(self):
        clone = Trajectory(self.get_num_segments())
        clone.segments = self.copy_segments()
        return clone

    def copy_segments(self):
        clones = []
        for segment in self.segments:
            clones.append(Segment(segment))
        return clones
