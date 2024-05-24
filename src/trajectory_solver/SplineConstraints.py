import copy

from trajectory_solver.Splines import Splines

class SplineConstraints(Splines):
    def __init__(self, N: int, coeffs=None) -> None:
        super().__init__(N, coeffs)
    
    def pos_constraint(self, t:float):
        self._time_pos(t)
        return copy.deepcopy(self._t.reshape(1, -1))

    def vel_constraint(self, t:float):
        self._time_vel(t)
        return copy.deepcopy(self._dt.reshape(1, -1))
    
    def acc_constraint(self, t:float):
        self._time_acc(t)
        return copy.deepcopy(self._ddt.reshape(1, -1))
