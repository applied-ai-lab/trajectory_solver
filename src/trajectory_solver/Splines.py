import numpy as np


class Splines:
    def __init__(self, N:int, coeffs=None) -> None:
        self._N = N
        self._coeffs = coeffs
        self._t   = np.zeros(self._N + 1)
        self._dt  = np.zeros(self._N + 1)
        self._ddt = np.zeros(self._N + 1)

    @property
    def coeffs(self):
        return self._coeffs
    
    @coeffs.setter
    def coeffs(self, values):
        self._coeffs = values

    def pos(self, t):
        self._time_pos(t)
        return np.dot(self._t, self._coeffs)
    
    def vel(self, t):
        self._time_vel(t)
        return np.dot(self._dt, self._coeffs)
    
    def acc(self, t):
        self._time_acc(t)
        return np.dot(self._ddt, self._coeffs)

    def _time_pos(self, t: float):
        self._t *= 0.0
        for k in range(self._N + 1):
            self._t[k] = t ** float(self._N - k)
    
    def _time_vel(self, t: float):
        self._dt *= 0.0
        for k in range(self._N):
            self._dt[k] = float(self._N - k) * t ** float(self._N - k - 1)
    
    def _time_acc(self, t: float):
        self._ddt *= 0.0
        for k in range(self._N - 1):
            self._ddt[k] = float(self._N - k) * float(self._N - k - 1) * t ** float(self._N - k - 2)
