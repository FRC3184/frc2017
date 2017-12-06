from typing import List, Tuple, Optional, Union
from mathutils import LineSegment, Vector2
import pose

import copy


def curvature(pose: pose.Pose, path: List[Vector2], lookahead: float) -> float:
    # Get lookahead point
    path_lines = []
    # Build a path of segments
    for k in range(len(path) - 1):
        path_lines += [LineSegment(path[k], path[k + 1])]
    project_points = []
    # Project the robot's pose onto each line to find the closest line to the robot
    for line in path_lines:
        project = line.projected_point(pose)
        dist = project.distance(pose)
        project_points += [(project, dist, line)]
    # Choose the closest point
    print(project_points)
    closest_point, dist, line = sorted(project_points, key=lambda x: x[1])[0]

    # If the closest point is farther from the robot than `lookahead`, we need to instead use
    # the point on the path
    goal = closest_point
    if dist <= lookahead:
        # Calculate the forward intercept point of the lookahead circle
        t = line.invert(closest_point)
        d = (lookahead**2 - dist**2)**0.5
        if line.in_segment(t + d):
            goal = line.r(t + d)
    curv = -2 * goal.translated(pose).y / lookahead ** 2
    return curv
