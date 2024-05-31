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
