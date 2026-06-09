from enum import Enum

import numpy as np


class HandEnum(Enum):
    LEFT =  "LEFT"
    RIGHT = "RIGHT"


class PoseVelAcc:
    def __init__(self, pos: np.array, vel: np.array, acc: np.array) -> None:
        self.pos = pos
        self.vel = vel
        self.acc = acc


class RelativeTimings:
    def __init__(self, phase_offset=0.0, duration=20, dt=0.02) -> None:
        self.phase_offset = phase_offset
        self.duration = duration
        self.dt = dt


class TimingIndices:
    indices = None
    duration = None


class EndPointPose:
    def __init__(self) -> None:
        self.init_pose = PoseVelAcc(np.zeros(6),
                                    np.zeros(6),
                                    np.zeros(6))
        self.mid_pose = PoseVelAcc(np.zeros(6),
                                   np.zeros(6),
                                   np.zeros(6))
        self.end_pose = PoseVelAcc(np.zeros(6),
                                   np.zeros(6),
                                   np.zeros(6))
        
class PoseNames:
    name_list = ["x", "y", "z", "th_x", "th_y", "th_z"]
    

class JointAngles:
    def __init__(self, dof=7) -> None:
        self.dof = dof
        self.init_q = PoseVelAcc(np.zeros(dof),
                                    np.zeros(dof),
                                    np.zeros(dof))
        self.mid_q = PoseVelAcc(np.zeros(dof),
                                   np.zeros(dof),
                                   np.zeros(dof))
        self.end_q = PoseVelAcc(np.zeros(dof),
                                   np.zeros(dof),
                                   np.zeros(dof))
        

class JointNames:
    def __init__(self, dof=7):
        self.name_list = list("q" + str(k) for k in range(dof))
