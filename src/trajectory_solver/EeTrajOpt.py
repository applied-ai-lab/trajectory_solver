from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation as R

from trajectory_solver.PoseTypes import (HandEnum, EndPointPose, RelativeTimings)
from trajectory_solver.PoseProblem import PoseProblem


class EeTrajOpt:
    def __init__(self, namespace=[HandEnum.LEFT, HandEnum.RIGHT], N=6, const_A=True) -> None:
        self._namespace = namespace
        self._pose_dict = self.create_pose_dict()

        self._coeffs = np.zeros(N + 1) 

        self._pose_problem = PoseProblem(namespace, N, const_A)

    def create(self, verbose=False):
        self._pose_problem.create(verbose)

    def advance(self, pose_targets: Dict[HandEnum, EndPointPose], timings: RelativeTimings):
        # Reframe trajectory into init_pose frame of pose_targets

        # Rescale timing
        
        # Solve optimisation
        self._coeffs = self._pose_problem.advance(self._pose_dict)

        # Find splines

    def initialise(self):
        return
    
    def reset(self):
        return

    # Helper member functions
    def create_pose_dict(self):
        pose_dict = dict()
        for name in self._namespace:
            pose_dict[name] = EndPointPose()
        return pose_dict