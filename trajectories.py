import math
from typing import Tuple, List

import runner


class Landmark:
    def __init__(self, points=None):
        self.points = [] if points is None else points

    def add_point(self, point: Tuple[float, float], incoming_trajectory: float):
        self.points.append((point, incoming_trajectory))

    def within(self, candidate: Tuple[float, float], robot_diameter: float) -> bool:
        return any(runner.euclidean_distance(p, candidate) < robot_diameter for (p, _) in self.points)


class TrajectoryMap:
    def __init__(self, distance_tolerance: float=0.01, heading_tolerance: float = math.pi / 32,
                 start=None, collisions=None, prev=None, current=None, named_locations=None):
        self.distance_tolerance = distance_tolerance
        self.heading_tolerance = heading_tolerance
        self.start = start
        self.collisions = [] if collisions is None else collisions
        self.prev = prev
        self.current = current
        self.named_locations = {} if named_locations is None else named_locations

    def __repr__(self):
        return f"TrajectoryMap({self.distance_tolerance}, {self.heading_tolerance}, {self.start}, {self.collisions}, {self.prev}, {self.current}, {self.named_locations})"

    def assign_location_name(self, name: str):
        if self.current is not None:
            self.named_locations[name] = self.current

    def is_started(self) -> bool:
        return self.start is not None

    def has_trajectory(self) -> bool:
        return self.prev is not None

    def update(self, x: float, y: float):
        if self.is_started():
            if runner.euclidean_distance((x, y), self.current) > self.distance_tolerance:
                if self.has_trajectory():
                    px, py = self.prev
                    cx, cy = self.current
                    prev_trajectory = math.atan2(cy - py, cx - px)
                    current_trajectory = math.atan2(y - cy, x - cx)
                    if abs(runner.angle_diff(prev_trajectory, current_trajectory)) > self.heading_tolerance:
                        self.collisions.append(self.current)
                self.prev = self.current
                self.current = (x, y)
        else:
            self.start = self.current = (x, y)

    def all_points(self) -> List[Tuple[float, float]]:
        if self.is_started():
            result = [self.start] + self.collisions
            if self.has_trajectory():
                result.extend([self.prev, self.current])
            return result
        else:
            return []

    def landmarks(self, robot_diameter=0.15) -> List[Landmark]:
        result = []
        prev = self.start
        for p in self.collisions:
            added = False
            for mark in result:
                if mark.within(p, robot_diameter):
                    added = True
                    mark.add_point(p, math.atan2(p[1] - prev[1], p[0] - prev[0]))
            if not added:
                result.append(Landmark([p]))
            prev = p
        return result
