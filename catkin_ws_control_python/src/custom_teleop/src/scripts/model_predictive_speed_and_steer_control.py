import time
import cvxpy
import math
import numpy as np
np.set_printoptions(precision=5)
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
class StateMPC:
    """
    vehicle state class
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

class MPC:
    def __init__(self, DT, N_IND_SEARCH, T, NX, WB, MAX_STEER, MAX_DSTEER, MAX_SPEED, MIN_SPEED, MAX_ACCEL):

        self.NX = NX  # x = x, y, v, yaw
        self.NU = 2  # a = [accel, steer]
        self.T = T  # horizon length

        # mpc parameters
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0]) # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix

        # iterative paramter
        self.MAX_ITER = 1  # Max iteration
        self.DU_TH = 0.1  # iteration finish param

        self.N_IND_SEARCH = N_IND_SEARCH  # Search index number

        self.DT = DT # [s] time tick

        # Vehicle parameters
        self.WB = WB  # [m]

        self.MAX_STEER = MAX_STEER #0.58  # maximum steering angle [rad]
        self.MAX_DSTEER = MAX_DSTEER #np.deg2rad(30.0)  # maximum steering speed [rad/s]
        self.MAX_SPEED = MAX_SPEED #1.5  # maximum speed [m/s]
        self.MIN_SPEED = MIN_SPEED #-1.5  # minimum speed [m/s]
        self.MAX_ACCEL = MAX_ACCEL #1.0  # maximum accel [m/ss]

    def get_linear_model_matrix(self, v, phi, delta):

        A = np.zeros((self.NX, self.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.DT * math.cos(phi)
        A[0, 3] = - self.DT * v * math.sin(phi)
        A[1, 2] = self.DT * math.sin(phi)
        A[1, 3] = self.DT * v * math.cos(phi)
        A[3, 2] = self.DT * math.tan(delta) / self.WB

        B = np.zeros((self.NX, self.NU))
        B[2, 0] = self.DT
        B[3, 1] = self.DT * v / (self.WB * math.cos(delta) ** 2)

        C = np.zeros(self.NX)
        C[0] = self.DT * v * math.sin(phi) * phi
        C[1] = - self.DT * v * math.cos(phi) * phi
        C[3] = - self.DT * v * delta / (self.WB * math.cos(delta) ** 2)

        return A, B, C

    def update_state(self, stateMPC, a, delta):

        # input check
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        stateMPC.x = stateMPC.x + stateMPC.v * math.cos(stateMPC.yaw) * self.DT
        stateMPC.y = stateMPC.y + stateMPC.v * math.sin(stateMPC.yaw) * self.DT
        stateMPC.yaw = stateMPC.yaw + stateMPC.v / self.WB * math.tan(delta) * self.DT
        stateMPC.v = stateMPC.v + a * self.DT

        if stateMPC.v > self.MAX_SPEED:
            stateMPC.v = self.MAX_SPEED
        elif stateMPC.v < self.MIN_SPEED:
            stateMPC.v = self.MIN_SPEED

        return stateMPC


    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()

    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        stateMPC = StateMPC(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            stateMPC = self.update_state(stateMPC, ai, di)
            xbar[0, i] = stateMPC.x
            xbar[1, i] = stateMPC.y
            xbar[2, i] = stateMPC.v
            xbar[3, i] = stateMPC.yaw

        return xbar

    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC control with updating operational point iteratively
        """
        ox, oy, oyaw, ov = None, None, None, None

        for i in range(self.MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= self.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov

    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constraints = []

        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])

            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                self.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.MAX_SPEED]
        constraints += [x[2, :] >= self.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.CLARABEL, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            ov = self.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def control_signals(self, xref, dref, oa, odelta, x0):
        """
        Simulation

        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        ck: course curvature list
        sp: speed profile
        dl: course tick [m]

        """
        oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

        return oa, odelta

if __name__ == '__main__':

    from mpc_utils import get_straight_course3, calc_nearest_index, calc_ref_trajectory, check_goal, plot_car, calc_speed_profile, smooth_yaw
    import matplotlib.pyplot as plt

    class State:
        """
        vehicle state class
        """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.predelta = None

    show_animation = True
    dl = 0.1  # course tick
    cx, cy, cyaw, ck = get_straight_course3(dl)

    plt.figure()
    plt.plot(cyaw, "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [kmh]")

    plt.show()

    TARGET_SPEED = 1.0  # [m/s] target speed
    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
    state = State(x=0, y=0, yaw=0.0, v=0.0)
    N_IND_SEARCH = 10
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0, N_IND_SEARCH)

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    goal = [cx[-1], cy[-1]]

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]

    odelta, oa = None, None
    cyaw = smooth_yaw(cyaw)

    plt.figure()
    plt.plot(cyaw, "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [kmh]")

    plt.show()
    exit()
    DT = 0.02
    T = 5
    NX = 4 # number of states
    WB = 0.650
    MAX_STEER = 0.5 #self.max_steer_angle # maximum steering angle [rad]
    MAX_DSTEER = np.deg2rad(40.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 1.5 #self.max_linear_speed # maximum speed [m/s]
    MIN_SPEED = -1.5 #self.max_linear_speed # minimum speed [m/s]
    MAX_ACCEL = 1.0  # maximum accel [m/ss]

    mpc_controller = MPC(DT, N_IND_SEARCH, T, NX, WB, MAX_STEER, MAX_DSTEER, MAX_SPEED, MIN_SPEED, MAX_ACCEL)

    MAX_TIME = 2000  # max simulation time
    GOAL_DIS = 1.5  # goal distance
    STOP_SPEED = 0.125 # stop speed

    oa = [0.0] * T
    odelta = [0.0] * T


    ai = 0
    di = 0
    ct = 0
    while ct <= MAX_TIME:
        xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind, T, NX, N_IND_SEARCH, DT)
        x0 = [state.x, state.y, state.v, state.yaw]  # current state
        print(xref)
        print(target_ind)
        print(ct)
        ct += 1

        oa, odelta = mpc_controller.control_signals(xref, dref, oa, odelta, x0)

        ai = oa[0]
        di = odelta[0]

        state = mpc_controller.update_state(state, ai, di) #  this is actual model

        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)

        if check_goal(state, goal, target_ind, len(cx), GOAL_DIS, STOP_SPEED):
             print("Goal")
             break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            #if ox is not None:
            #    plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, WB, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()
