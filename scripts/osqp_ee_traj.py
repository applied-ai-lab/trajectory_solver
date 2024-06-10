from time import process_time 

import numpy as np
from matplotlib import pyplot as plt

from trajectory_solver.EeTrajOpt import (EeTrajOpt, HandEnum, 
                                         EndPointPose, RelativeTimings)



def test():

    namespace = [HandEnum.LEFT, HandEnum.RIGHT]

    spline_N = 8

    dt = 0.1
    duration = 4.0

    ee_traj = EeTrajOpt(namespace, spline_N, True)
    ee_traj.create(verbose=False)

    ee_traj.initialise()

    poses = {HandEnum.LEFT: EndPointPose(), 
             HandEnum.RIGHT: EndPointPose()}

    relative_times = RelativeTimings(-np.pi / 2.0, duration, dt)

    t1_start = process_time()

    no_runs = 1000
    for k in range(no_runs):
        if k % 2 == 0:
            poses[HandEnum.RIGHT].init_pose.pos = 0.6 * np.ones(6)
            poses[HandEnum.LEFT].init_pose.pos = -0.6 * np.ones(6)

            poses[HandEnum.RIGHT].init_pose.vel = 0. * np.ones(6)
            poses[HandEnum.LEFT].init_pose.vel = 0. * np.ones(6)

            poses[HandEnum.RIGHT].mid_pose.pos = 0.15 * np.ones(6)
            poses[HandEnum.LEFT].mid_pose.pos = -0.15 * np.ones(6)

            poses[HandEnum.RIGHT].end_pose.pos = 0.0 * np.ones(6)
            poses[HandEnum.LEFT].end_pose.pos = 0.0 * np.ones(6)
        else:
            poses[HandEnum.RIGHT].init_pose.pos = 0.0 * np.ones(6)
            poses[HandEnum.LEFT].init_pose.pos = -0.0 * np.ones(6)

            poses[HandEnum.RIGHT].init_pose.vel = 0. * np.ones(6)
            poses[HandEnum.LEFT].init_pose.vel = 0. * np.ones(6)

            poses[HandEnum.RIGHT].mid_pose.pos = 0.15 * np.ones(6)
            poses[HandEnum.LEFT].mid_pose.pos = -0.15 * np.ones(6)

            poses[HandEnum.RIGHT].end_pose.pos = 0.6 * np.ones(6)
            poses[HandEnum.LEFT].end_pose.pos = -0.6 * np.ones(6)

        # Advance ee trajectory
        timed_splines_class = ee_traj.advance(poses, relative_times)

    # Stop the stopwatch / counter 
    t1_stop = process_time() 
    
    print("Elapsed time:", t1_start, t1_stop)  
    print("Elapsed time per update in seconds:", (t1_stop-t1_start) / no_runs)  

    # Get trajectories
    traj_dict = timed_splines_class.traj(0.0, duration + 2.0)


    pos_l = traj_dict[HandEnum.LEFT].pos
    vel_l = traj_dict[HandEnum.LEFT].vel
    acc_l = traj_dict[HandEnum.LEFT].acc
    
    pos_r = traj_dict[HandEnum.RIGHT].pos
    vel_r = traj_dict[HandEnum.RIGHT].vel
    acc_r = traj_dict[HandEnum.RIGHT].acc

    pos_pred = np.zeros(pos_l.shape[0])
    vel_pred = np.zeros(pos_l.shape[0])

    pos_pred[0] = pos_r[0, 0]
    vel_pred[0] = vel_r[0, 0]
    for k in range(1, pos_pred.shape[0], 1):
        pos_pred[k] = pos_pred[k - 1] + dt * vel_r[k - 1, 0]
        vel_pred[k] = vel_pred[k - 1] + dt * acc_r[k - 1, 0]

    _, axes = plt.subplots(3, 1)
    axes[0].plot(pos_l[:, 0])
    axes[0].plot(pos_r[:, 0])
    axes[0].plot(pos_pred, '*')

    axes[0].set_title("Position")

    axes[1].plot(vel_l[:, 0])
    axes[1].plot(vel_r[:, 0])
    axes[1].plot(vel_pred, '*')
    axes[1].set_title("Velcity")

    axes[2].plot(acc_l[:, 0])
    axes[2].plot(acc_r[:, 0])
    axes[2].set_title("Acceleration")

    plt.show()

    return 0


if __name__ == "__main__":
    test()
