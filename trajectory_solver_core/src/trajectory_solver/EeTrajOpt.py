import copy
from typing import Dict

import numpy as np
# from scipy.spatial.transform import Rotation as R

from trajectory_solver.Splines import (Splines, SplineTimeEnum)
from trajectory_solver.PoseTypes import (HandEnum, EndPointPose, 
                                         RelativeTimings, PoseNames,
                                         JointAngles, JointNames,
                                         PoseVelAcc, TimingIndices)
from trajectory_solver.PoseProblem import PoseProblem
from trajectory_solver.JointSpaceProblem import JointSpaceProblem


class TimeAndSplines:
    def __init__(self, spline_dict: Dict[HandEnum, Splines], timings: RelativeTimings, pose_names=PoseNames.name_list) -> None:
        self._spline_dict = spline_dict
        self._timings = timings
        self._times = self.create_times()
        self._out = None
        self._dt = timings.dt

        self._pose_names = pose_names

    def create_times(self) -> Dict[HandEnum, np.array]:
        times_dict = dict()
        for name in self._spline_dict.keys():
            duration = copy.deepcopy(self._timings.duration)
            if name == HandEnum.LEFT and self._timings.phase_offset > 0:
                duration -= (0.5 * self._timings.phase_offset / np.pi) * self._timings.duration
            elif name == HandEnum.RIGHT and self._timings.phase_offset < 0:
                duration += (0.5 * self._timings.phase_offset / np.pi) * self._timings.duration

            times_dict[name] = TimingIndices()
            times_dict[name].indices =  np.arange(0.0, duration + self._timings.dt, self._timings.dt) / duration
            times_dict[name].duration = duration
        return times_dict
    
    @property
    def out(self):
        return {SplineTimeEnum.SPLINES: self._spline_dict,
                SplineTimeEnum.TIMES: self._times}

    @staticmethod
    def vel_scaling(duration):
        return 1.0  / duration
    
    @staticmethod
    def acc_scaling(duration):
        return (1.0 / duration) ** 2.0
    
    # Return pos, vel and acc 
    def traj(self, start_t: float, stop_t: float):
        start_index = int((start_t / self._timings.dt))
        stop_index  = int((stop_t / self._timings.dt))

        duration = stop_index - start_index + 1

        traj_dict = dict()
        for name, spline in self._spline_dict.items():

            pos = np.zeros([duration, len(self._pose_names)])
            vel = np.zeros([duration, len(self._pose_names)])
            acc = np.zeros([duration, len(self._pose_names)])

            for dim, pose_name in enumerate(self._pose_names):
                spline_name = name.value + "_" + pose_name
                for k in range(duration):
                    
                    if k < self._times[name].indices.shape[0]:
                        t = self._times[name].indices[start_index + k]
                        pos[k, dim] = spline[spline_name].pos(t)
                        vel[k, dim] = self.vel_scaling(self._times[name].duration) * spline[spline_name].vel(t)
                        acc[k, dim] = self.acc_scaling(self._times[name].duration) * spline[spline_name].acc(t)
                    else:
                        pos[k, dim] = pos[k - 1, dim]
                        vel[k, dim] = 0.0 * vel[k - 1, dim]
                        acc[k, dim] = 0.0 * acc[k - 1, dim]
                
                traj_dict[name] = PoseVelAcc(pos, vel, acc)
        
        return traj_dict
    

class EeTrajOpt:
    def __init__(self, namespace=[HandEnum.LEFT, HandEnum.RIGHT], N=6, const_A=True, problem_type=PoseProblem, pose_names=PoseNames.name_list) -> None:
        self._namespace = namespace
        self._N = N
        # self._pose_dict = self.create_pose_dict()
        self._pose_problem = problem_type(namespace, N, const_A)

        self._splines = self.create_splines()
        self._namespaced_splines = self.create_namespaced_splines(self._splines)

        self._coeffs = np.zeros(N + 1) 
        
        self._pose_names = pose_names

    def create(self, verbose=False):
        self._pose_problem.create(verbose)

    def advance(self, pose_targets: Dict[HandEnum, EndPointPose], timings: RelativeTimings):
        # Solve optimisation
        self._coeffs = self._pose_problem.advance(pose_targets)

        # Update the spline coeffs
        self.update_splines(self._coeffs)

        # Update spline map (Should not be necessary)
        return TimeAndSplines(self._namespaced_splines, timings, self._pose_names)

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
