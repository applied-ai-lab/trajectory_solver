import copy
from enum import Enum
from typing import List

import numpy as np
from scipy import sparse

from trajectory_solver.Splines import Splines
from trajectory_solver.QPcoeffs import QPcoeffs


class ConstraintType(Enum):
    POS = 0
    VEL = 1
    ACC = 2


class TemporalConstraint:
    def __init__(self, 
                 t: float,
                 u: float,
                 l: float, 
                 type: ConstraintType) -> None:
        self.t = t
        self.type = type
        self.u = u
        self.l = l


class SplineConstraints(Splines):
    def __init__(self, N: int, coeffs=None) -> None:
        super().__init__(N, coeffs)
        self.constraint_map = {ConstraintType.POS: self.pos_constraint,
                               ConstraintType.VEL: self.vel_constraint,
                               ConstraintType.ACC: self.acc_constraint}
    
    def pos_constraint(self, t:float):
        self._time_pos(t)
        return copy.deepcopy(self._t.reshape(1, -1))

    def vel_constraint(self, t:float):
        self._time_vel(t)
        return copy.deepcopy(self._dt.reshape(1, -1))
    
    def acc_constraint(self, t:float):
        self._time_acc(t)
        return copy.deepcopy(self._ddt.reshape(1, -1))
    
    def create_qp(self, constraints: List[TemporalConstraint]) -> QPcoeffs:
        # Create constraints
        l = np.zeros(len(constraints))
        u = np.zeros(len(constraints))
        A_np = np.zeros((len(constraints, self._N + 1)))
        # To do set the these elsewhere: create the costs
        P = sparse.eye(self._N + 1) 
        q = np.zeros(self._N + 1)

        for row, constraint in enumerate(constraints):
            A_np[row, :] = self.constraint_map[constraint.type](constraint.t)
            l[row] = constraint.l
            u[row] = constraint.u
        # Spline constraints
        A = sparse.csc_matrix(A_np)

        return QPcoeffs(P, q, A, l, u)

    def advance_qp(self, row: int, qp_coeffs: QPcoeffs, time_constraint: TemporalConstraint) -> None:
        qp_coeffs.A[row, :] = self.constraint_map[time_constraint.type](time_constraint.t)
        qp_coeffs.l[row] = time_constraint.l
        qp_coeffs.u[row] = time_constraint.u
        return 
