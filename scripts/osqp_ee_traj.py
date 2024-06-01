import numpy as np

from matplotlib import pyplot as plt

from trajectory_solver.EeTrajOpt import (EeTrajOpt, HandEnum, 
                                         EndPointPose, SplineTimeEnum, 
                                         RelativeTimings)


def test():

    namespace = [HandEnum.LEFT, HandEnum.RIGHT]

    spline_N = 8
    spline_N1 = spline_N + 1

    ee_traj = EeTrajOpt(namespace, spline_N, True)
    ee_traj.create(verbose=False)

    ee_traj.initialise()

    poses = {HandEnum.LEFT: EndPointPose(), 
             HandEnum.RIGHT: EndPointPose()}


    poses[HandEnum.RIGHT].init_pose.pos = 0.4 * np.ones(6)
    poses[HandEnum.LEFT].init_pose.pos = -0.4 * np.ones(6)

    poses[HandEnum.RIGHT].init_pose.vel = 0. * np.ones(6)
    poses[HandEnum.LEFT].init_pose.vel = 0. * np.ones(6)

    poses[HandEnum.RIGHT].mid_pose.pos = 0.15 * np.ones(6)
    poses[HandEnum.LEFT].mid_pose.pos = -0.15 * np.ones(6)

    poses[HandEnum.RIGHT].end_pose.pos = 0.0 * np.ones(6)
    poses[HandEnum.LEFT].end_pose.pos = 0.0 * np.ones(6)

    relative_times = RelativeTimings(-np.pi / 2.0, 20, 0.02)

    timed_splines = ee_traj.advance(poses, relative_times)


    spline_params = timed_splines[SplineTimeEnum.SPLINES]
    times = timed_splines[SplineTimeEnum.TIMES]

    import pdb
    pdb.set_trace()


    pos_l = np.zeros(len(times[HandEnum.LEFT]))
    vel_l = np.zeros(len(times[HandEnum.LEFT]))
    acc_l = np.zeros(len(times[HandEnum.LEFT]))

    pos_r = np.zeros(len(times[HandEnum.RIGHT]))
    pos_pred = np.zeros(len(times[HandEnum.RIGHT]))
    vel_r = np.zeros(len(times[HandEnum.RIGHT]))
    acc_r = np.zeros(len(times[HandEnum.RIGHT]))

    for k in range(len(times[HandEnum.LEFT])):
        pos_l[k] = spline_params[HandEnum.LEFT]['LEFT_x'].pos(times[HandEnum.LEFT][k])
        vel_l[k] = spline_params[HandEnum.LEFT]['LEFT_x'].vel(times[HandEnum.LEFT][k])
        acc_l[k] = spline_params[HandEnum.LEFT]['LEFT_x'].acc(times[HandEnum.LEFT][k])
    
    for k in range(len(times[HandEnum.RIGHT])):
        pos_r[k] = spline_params[HandEnum.RIGHT]['RIGHT_x'].pos(times[HandEnum.RIGHT][k])
        vel_r[k] = spline_params[HandEnum.RIGHT]['RIGHT_x'].vel(times[HandEnum.RIGHT][k])
        acc_r[k] = spline_params[HandEnum.RIGHT]['RIGHT_x'].acc(times[HandEnum.RIGHT][k])

    pos_pred[0] = pos_r[0]
    for k in range(1, len(times[HandEnum.RIGHT]), 1):
        pos_pred[k] = pos_pred[k - 1] + relative_times.dt * vel_r[k - 1]

    _, axes = plt.subplots(3, 1)
    axes[0].plot(pos_l)
    axes[0].plot(pos_r)
    axes[0].plot(pos_pred)

    axes[0].set_title("Position")

    axes[1].plot(vel_l)
    axes[1].plot(vel_r)
    axes[1].set_title("Velcity")

    axes[2].plot(acc_l)
    axes[2].plot(acc_r)
    axes[2].set_title("Acceleration")

    plt.show()

    return 0


if __name__ == "__main__":
    test()
