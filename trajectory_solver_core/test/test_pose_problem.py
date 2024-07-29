import numpy as np

from matplotlib import pyplot as plt 

from trajectory_solver.PoseProblem import PoseProblem, HandEnum, EndPointPose


def test():

    namespace = [HandEnum.LEFT, HandEnum.RIGHT]

    spline_N = 5
    pose_problem = PoseProblem(namespace, spline_N, True)
    pose_problem.create()

    # Initialise the problem
    pose_problem.initialise()

    # Update the pose problems
    poses = {HandEnum.LEFT: EndPointPose(), 
             HandEnum.RIGHT: EndPointPose()}
    
    poses[HandEnum.RIGHT].mid_pose.pos = 0.05 * np.ones(6)
    poses[HandEnum.LEFT].mid_pose.pos = 0.05 * np.ones(6)

    poses[HandEnum.RIGHT].end_pose.pos = 0.1 * np.ones(6)
    poses[HandEnum.LEFT].end_pose.pos = 0.1 * np.ones(6)

    # Advance the problem
    _ = pose_problem.advance(poses)
    
    # Reset
    pose_problem.reset()
    return 0


if __name__ == "__main__":
    test()
