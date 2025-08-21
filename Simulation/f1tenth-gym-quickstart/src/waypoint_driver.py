import numpy as np

class PurePursuitDriver:
    def __init__(self, waypoints, lookahead=0.3):
        self.waypoints = waypoints
        self.lookahead = lookahead
        self.target_index = 0

    def process_lidar(self, scan, x, y):
        current_pos = np.array([x, y])

        # Find the next waypoint at lookahead distance
        while self.target_index < len(self.waypoints):
            target = np.array(self.waypoints[self.target_index])
            dist = np.linalg.norm(target - current_pos)
            if dist > self.lookahead:
                break
            self.target_index += 1

        if self.target_index >= len(self.waypoints):
            return 0.0, 0.0  # stop if finished

        goal = self.waypoints[self.target_index]
        dx = goal[0] - x
        dy = goal[1] - y
        steering_angle = np.arctan2(dy, dx)
        speed = 1.5  # Constant speed, can make smarter later

        return speed, steering_angle
