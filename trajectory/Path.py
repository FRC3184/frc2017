

class Path:
    def __init__(self, name, go_left_pair):
        self.name = name
        self.trajectory_pair = go_left_pair
        self.go_left = True

    def go_left(self):
        self.go_left = True
        self.trajectory_pair.left.inverted_y = False
        self.trajectory_pair.right.inverted_y = False

    def go_right(self):
        self.go_left = False
        self.trajectory_pair.left.inverted_y = True
        self.trajectory_pair.right.inverted_y = True

    def get_left_wheel_trajectory(self):
        return self.trajectory_pair.left if self.go_left else self.trajectory_pair.right

    def get_right_wheel_trajectory(self):
        return self.trajectory_pair.right if self.go_left else self.trajectory_pair.left

    def get_end_heading(self):
        return self.trajectory_pair.left.segments[-1].heading
