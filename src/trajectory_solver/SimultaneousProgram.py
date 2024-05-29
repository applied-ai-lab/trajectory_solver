from typing import List, Dict

import numpy as np
from scipy import sparse

from trajectory_solver.QPcoeffs import QPcoeffs
from trajectory_solver.SplineConstraints import TemporalConstraint, SplineConstraints, ConstraintType


class NamedTemporalConstraint:
    def __init__(self, name: str, constraint_dict: Dict[str, TemporalConstraint]) -> None:
        self.name = name
        self.constraint_dict = constraint_dict


class ArrayIndex:
    def __init__(self, start=0, stop=0) -> None:
        self.start = start
        self.stop = stop

class MatrixIndex:
    def __init__(self, row_start=0, row_stop=0, col_start=0, col_stop=0) -> None:
        self.row = ArrayIndex(row_start, row_stop)
        self.col = ArrayIndex(col_start, col_stop)


class QPIndicies:
    def __init__(self) -> None:
        self.P = MatrixIndex()
        self.q = ArrayIndex()

        self.A = MatrixIndex()
        self.l = ArrayIndex()
        self.u = ArrayIndex()


class SimultaneousProgram(QPcoeffs):
    def __init__(self, N):
        super().__init__(None, None, None, None, None)

        self.constraint_map = dict()
        self.constraint_indices = dict() # This will contain QPindices

        # The spline order
        self.N = N

        self.no_constraints = 0
        self.no_decision_var = 0

    def create(self, named_constraints: List[NamedTemporalConstraint]):
        # create the spline constraints
        self.spline_constraint = SplineConstraints(self.N)
        # Reset the contraint map and indices
        self.constraint_indices = dict()
        self.constraint_map = dict()
        self.no_constraints = 0
        for constraint in named_constraints:
            self.constraint_map[constraint.name] = self.spline_constraint.create_qp(constraint.constraint_dict)
            self.no_constraints += len(constraint.constraint_dict.values())
        # Create the large constraint map
        self.no_decision_var = (self.N + 1) * len(named_constraints)
        
        # Create the init values
        self._P = sparse.eye(self.no_decision_var)
        self._q = np.zeros(self.no_decision_var)

        A_np = np.zeros([self.no_constraints, self.no_decision_var])
        self._l = np.zeros(self.no_constraints)
        self._u = np.zeros(self.no_constraints)

        row = 0
        col = 0
        for key, item in self.constraint_map.items():
            indices = QPIndicies()
            
            A_np[row: row + item.A.shape[0], col: col + item.A.shape[1]] = item.A.todense()
            self._l[row: row + item.l.shape[0]] = item.l
            self._u[row: row + item.u.shape[0]] = item.u

            # Update the indices
            indices.P.row.start = row
            indices.P.row.stop = row + item.P.shape[0]
            indices.P.col.start = col
            indices.P.col.stop = col + item.P.shape[1]

            indices.q.start = col
            indices.q.stop = col + item.q.shape[0]

            indices.A.row.start = row
            indices.A.row.stop = row + item.A.shape[0]
            indices.A.col.start = col
            indices.A.col.stop = col + item.A.shape[1]

            indices.l.start = row
            indices.l.stop = row + item.l.shape[0]

            indices.u.start = row
            indices.u.stop = row + item.u.shape[0]

            self.constraint_indices[key] = indices
            
            # Update the counters
            row += item.A.shape[0]
            col += item.A.shape[1]

        self._A = sparse.lil_matrix(A_np)
        return
    
    def initialise(self):
        return
    
    def advance(self, named_constraints: List[NamedTemporalConstraint]):
        for constraints in named_constraints:
            for _, constraint in constraints.constraint_dict.items():
                self.spline_constraint.advance_qp(constraint.row, self.constraint_map[constraints.name], constraint)
            # Update the global constraints
            indices = self.constraint_indices[constraints.name]
            self._A[indices.A.row.start: indices.A.row.stop, indices.A.col.start: indices.A.col.stop] = self.constraint_map[constraints.name].A
            self._l[indices.l.start: indices.l.stop] = self.constraint_map[constraints.name].l
            self._u[indices.u.start: indices.u.stop] = self.constraint_map[constraints.name].u
        return
    
    def reset(self):
        return