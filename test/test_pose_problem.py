from trajectory_solver.PoseProblem import PoseProblem, HandEnum, EndPointPose


def test():

    namespace = [HandEnum.LEFT, HandEnum.RIGHT]

    pose_problem = PoseProblem(namespace, 5, True)

    pose_problem.create()

    pose_problem.initialise()

    poses = {HandEnum.LEFT: EndPointPose(), HandEnum.RIGHT: EndPointPose()}

    pose_problem.advance(poses)

    pose_problem.reset()

    return 0


if __name__ == "__main__":
    test()
