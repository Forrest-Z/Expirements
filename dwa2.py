import math
import numpy as np
import matplotlib.pyplot as plt

show_animation = True


class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_steer_angle = 0.785398
        self.min_steer_angle = -0.785398
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 1.0  # [m]
        
        

def motion(x, u, delta, dt):
    # motion model
    
    SPEED = 1.4
    LENGTH = 1.5    
    omega = SPEED/LENGTH * np.tan(delta)
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] +=  normalize(u[1] + (omega*dt)) 
    x[3] = u[0]
    x[4] = u[1]
    x[5] = delta
    return x


def normalize(theta):
    if theta < 0:    
        theta = theta + 2.0 * np.pi
        return theta
    if theta > 2*np.pi:    
        theta = theta - 2.0 * np.pi
        return theta
    else:
        return theta

def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate,
          config.min_steer_angle, config.max_steer_angle]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt,
          x[5] - config.max_steer_angle * config.dt,
          x[5] + config.max_steer_angle * config.dt]
    #  print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3]),
          max(Vs[4], Vd[4]), min(Vs[5], Vd[5])]
    #  print(dw)

    return dw


def calc_trajectory(xinit, delta, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], delta, config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    #  print(len(traj))
    return traj


def calc_final_input(x, u, delta, dw, config, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])
    delta_angle = delta_augment( x[5], 4, 0.0872665 )
    final_cost_list = []
    # evalucate all trajectory with sampled input in dynamic window
    for angle in len(delta_angle):
        for v in np.arange(dw[0], dw[1], config.v_reso):
            for y in np.arange(dw[2], dw[3], config.yawrate_reso):
                traj = calc_trajectory(xinit, delta_angle[angle], v, y, config)
    
                # calc cost
                to_goal_cost = calc_to_goal_cost(traj, goal, config)
                speed_cost = config.speed_cost_gain * \
                    (config.max_speed - traj[-1, 3])
                ob_cost = calc_obstacle_cost(traj, ob, config)
                #  print(ob_cost)                
                steer_cost = abs(delta_angle[angle]- x[5])
                final_cost = to_goal_cost + speed_cost + ob_cost + steer_cost
                final_cost_list.append(final_cost)

            # search minimum trajectory
            final_cost2 = min(final_cost_list)
            if min_cost >= final_cost2:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    #  print(min_u)
    #  input()

    return min_u, best_traj


def delta_augment(delta, n, off, config):

    delta_list = []
    delta_list.append(delta)
    delta_calc = delta
    for i in range(0 ,n):
        delta_calc += off   
        if delta_calc < config.max_steer_angle:
            delta_list.append(delta_calc)
        
    delta_calc = delta
    for i in range(0 ,n):
        delta_calc -= off
        if config.min_steer_angle < delta_calc:
            delta_list.append(delta_calc)
        
    return delta_list


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collisiton

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    dx = goal[0] - traj[-1, 0]
    dy = goal[1] - traj[-1, 1]
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def dwa_control(x, u, delta_angle, config, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, delta_angle, dw, config, goal, ob)

    return u, traj


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def main():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 10])
    # obstacles [x(m) y(m), ....]
    ob = np.matrix([[-1, -1],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ])

    u = np.array([0.0, 0.0])
    config = Config()
    traj = np.array(x)
    

    for i in range(1000):
                
        delta_angle = 0
        #delta_angle = delta_augment( x[5], 4, 0.0872665 )
        u, ltraj = dwa_control(x, u, delta_angle,config, goal, ob)
        x = motion(x, u, delta_angle, config.dt)
        traj = np.vstack((traj, x))  # store state history
        if show_animation:
            plt.plot(traj[:, 0], traj[:, 1], "-r")
            plt.show()
            print(x[5])
        '''
        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
'''
        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    


if __name__ == '__main__':
    main()
