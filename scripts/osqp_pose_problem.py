import numpy as np

from matplotlib import pyplot as plt 

from trajectory_solver.PoseProblem import PoseProblem, HandEnum, EndPointPose
from trajectory_solver.Splines import Splines


def test():

    namespace = [HandEnum.LEFT, HandEnum.RIGHT]

    spline_N = 8
    spline_N1 = spline_N + 1
    pose_problem = PoseProblem(namespace, spline_N, True)

    pose_problem.create(verbose=True)

    pose_problem.initialise()

    poses = {HandEnum.LEFT: EndPointPose(), 
             HandEnum.RIGHT: EndPointPose()}


    poses[HandEnum.RIGHT].init_pose.pos = 3.1 * np.ones(6)
    poses[HandEnum.LEFT].init_pose.pos = 3.1 * np.ones(6)

    poses[HandEnum.RIGHT].init_pose.vel = 0.15 * np.ones(6)
    poses[HandEnum.LEFT].init_pose.vel = 0.15 * np.ones(6)

    poses[HandEnum.RIGHT].mid_pose.pos = -0.15 * np.ones(6)
    poses[HandEnum.LEFT].mid_pose.pos = -0.15 * np.ones(6)

    poses[HandEnum.RIGHT].end_pose.pos = 1.1 * np.ones(6)
    poses[HandEnum.LEFT].end_pose.pos = 1.1 * np.ones(6)

    spline_params = pose_problem.advance(poses)

    # Setup splines
    spline_lst = []
    for k in range(len(namespace) * 2):
        spline_lst.append(Splines(N=spline_N, 
                                  coeffs=spline_params[spline_N1 * k: spline_N1 * k + spline_N1]))

    num_nodes = 100
    t = np.linspace(0.0, 1.0, num_nodes)

    pos = np.zeros(num_nodes)
    vel = np.zeros(num_nodes)
    acc = np.zeros(num_nodes)

    for k in range(num_nodes):
        pos[k] = spline_lst[0].pos(t[k])
        vel[k] = spline_lst[0].vel(t[k])
        acc[k] = spline_lst[0].acc(t[k])

    _, axes = plt.subplots(3, 1)
    axes[0].plot(pos)

    axes[0].set_title("Position")

    axes[1].plot(vel)
    axes[1].set_title("Velcity")

    axes[2].plot(acc)
    axes[2].set_title("Acceleration")

    plt.show()

    pose_problem.reset()

    return 0


if __name__ == "__main__":
    test()
