import math
import numpy as np

class TargetCourse:
    def __init__(self, cx, cy, k=0.1, Lfc=1):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None
        self.k = k  # look forward gain
        self.Lfc = Lfc  # [m] look-ahead distance

    def calc_distance(self, rear_x, rear_y, point_x, point_y):
        dx = rear_x - point_x
        dy = rear_y - point_y
        return math.hypot(dx, dy)

    def search_target_index(self, rear_x, rear_y, v):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [rear_x - icx for icx in self.cx]
            dy = [rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = self.calc_distance(rear_x, rear_y, self.cx[ind], self.cy[ind])
            while True:
                distance_next_index = self.calc_distance(rear_x, rear_y, self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = self.k * v + self.Lfc  # update look ahead distance
        #Lf = max(0.5, min(2.0, self.k * v + self.Lfc)) # 0.5 - 2
        # search look ahead target point index
        while Lf > self.calc_distance(rear_x, rear_y, self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

def pure_pursuit_steer_control(L, trajectory, pind, rear_x, rear_y, x_odo, y_odo, v_odo, theta_odo, delta_filtered):

    ind, Lf = trajectory.search_target_index(rear_x, rear_y, v_odo)
    #Lf = max(0.5, min(2.0, 0.8 * Lf + 0.2 * (0.2 * v_odo + 0.0650 /2)))

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]

    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    #cte = np.hypot(ty - rear_y, tx - rear_x)
    #print("-----------------------------------------", cte)

    alpha = math.atan2(ty - rear_y, tx - rear_x) - theta_odo
    #alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf,  1.0)
    delta_filtered = 0.1 * delta_filtered + 0.9 * delta  # Low-pass filter
    return delta_filtered, ind
