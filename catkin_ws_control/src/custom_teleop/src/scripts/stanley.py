import numpy as np

def angle_mod(x, zero_2_2pi=False, degree=False):
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    return angle_mod(angle)

def calc_target_index(cx, cy, x_odo, y_odo, theta_odo, L):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """

    # Calc front axle position
    fx = x_odo + L * np.cos(theta_odo)
    fy = y_odo + L * np.sin(theta_odo)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    #look_ahead_dist = L * 1.5  # Look ahead distance (~1.5x wheelbase)
    #while target_idx < len(cx) - 1 and d[target_idx] < look_ahead_dist:
    #    target_idx += 1  # Move to a future waypoint

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(theta_odo + np.pi / 2),
                      -np.sin(theta_odo + np.pi / 2)]

    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def stanley_control(L, cx, cy, cyaw, last_target_idx, k_stanley, theta_odo, v_odo, x_odo, y_odo):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """

    current_target_idx, error_front_axle = calc_target_index(cx, cy, x_odo, y_odo, theta_odo, L)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - theta_odo)

    # theta_d corrects the cross track error
    #k_dynamic = k_stanley + 1.0 / (1.0 + v_odo)  # Adjust adaptively

    theta_d = np.arctan2(k_stanley * error_front_axle, v_odo)

    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx

def rot_mat_2d(self, angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


