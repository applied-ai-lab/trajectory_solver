import copy
from typing import Dict

import numpy as np
# from scipy.spatial.transform import Rotation as R

from trajectory_solver.Splines import (Splines, SplineTimeEnum)
from trajectory_solver.PoseTypes import (HandEnum, EndPointPose, RelativeTimings)
from trajectory_solver.PoseProblem import PoseProblem


class TimeAndSplines:
    def __init__(self, spline_dict: Dict[HandEnum, Splines], timings: RelativeTimings) -> None:
        self._spline_dict = spline_dict
        self._timings = timings
        self._times = self.create_times()
        self._out = None

    def create_times(self):
        times_dict = dict()
        for name in self._spline_dict.keys():
            duration = copy.deepcopy(self._timings.duration)
            if name == HandEnum.LEFT:
                duration += (self._timings.phase_offset / np.pi) * self._timings.duration
            elif name == HandEnum.RIGHT:
                duration += (self._timings.phase_offset / np.pi) * self._timings.duration

            print(duration)
            times_dict[name] = np.linspace(0., 1., int(duration / self._timings.dt))
        return times_dict
    
    @property
    def out(self):
        return {SplineTimeEnum.SPLINES: self._spline_dict,
                SplineTimeEnum.TIMES: self._times}



class EeTrajOpt:
    def __init__(self, namespace=[HandEnum.LEFT, HandEnum.RIGHT], N=6, const_A=True) -> None:
        self._namespace = namespace
        self._N = N
        # self._pose_dict = self.create_pose_dict()
        self._pose_problem = PoseProblem(namespace, N, const_A)

        self._splines = self.create_splines()
        self._namespaced_splines = self.create_namespaced_splines(self._splines)

        self._coeffs = np.zeros(N + 1) 

    def create(self, verbose=False):
        self._pose_problem.create(verbose)

    def advance(self, pose_targets: Dict[HandEnum, EndPointPose], timings: RelativeTimings):
        # Solve optimisation
        self._coeffs = self._pose_problem.advance(pose_targets)

        # Update the spline coeffs
        self.update_splines(self._coeffs)

        # Update spline map (Should not be necessary)
        return TimeAndSplines(self._namespaced_splines, timings).out

    def initialise(self):
        return
    
    def reset(self):
        return

    # Helper member functions
    def create_splines(self):
        spline_dict = dict()
        for name in self._pose_problem.constraint_names:
            spline_dict[name] = Splines(self._N)
        return spline_dict

    def update_splines(self, coeffs: np.array):
        N1 = self._N + 1
        for k, name in enumerate(self._pose_problem.constraint_names):
            self._splines[name].coeffs = coeffs[k * N1: (k + 1) * N1]

    def create_namespaced_splines(self, splines: Dict[HandEnum, Splines]):
        namespaced_splines = dict()
        for ns, names in self._pose_problem.namespaced_names_dict.items():
            spline_dict = dict()
            for name in names:
                spline_dict[name] = splines[name]
            namespaced_splines[ns] = spline_dict
        return namespaced_splines
        

    # def create_pose_dict(self):
    #     pose_dict = dict()
    #     for name in self._namespace:
    #         pose_dict[name] = EndPointPose()
    #     return pose_dict
    

        