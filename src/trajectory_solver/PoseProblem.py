from typing import Dict, List
import copy
from enum import Enum

import numpy as np
import osqp

from trajectory_solver.SimultaneousProgram import (SimultaneousProgram, NamedTemporalConstraint, 
                                                   TemporalConstraint, ConstraintType)


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


class EndPointProblem:
    def __init__(self) -> None:
            
        self.constraint_dict = {
        "pos_00": TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.POS),
        "vel_00": TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.VEL),
        "acc_00": TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.ACC),
        # "pos_05": TemporalConstraint(t=0.4, u=0.5, l=0.5, type=ConstraintType.POS),
        "pos_10": TemporalConstraint(t=0.8, u=1.0, l=1.0, type=ConstraintType.POS),
        "vel_10": TemporalConstraint(t=1.0, u=0.0, l=0.0, type=ConstraintType.VEL),
        "acc_10": TemporalConstraint(t=1.0, u=0.0, l=0.0, type=ConstraintType.ACC)
    }
        

class DualArmNames:
    def __init__(self, namespaces: List[HandEnum], names: List[str]) -> None:
        self._namespaces = namespaces
        self._names = names

        self._namespace_dict = self.create_name_dict()
        self._all_names_lst = self.create_name_list()

    # Properties
    @property
    def namespace_dict(self):
        return self._namespace_dict

    @property
    def all_names_lst(self):
        return self._all_names_lst
    
    def create_name_dict(self):
        names_dict = dict()
        for ns in self._namespaces:
            names_dict[ns] = list(ns.value + "_" + name for name in self._names)
        return names_dict
    
    def create_name_list(self):
        name_lst = []
        for _, item in self._namespace_dict.items():
            name_lst += item
        return name_lst


class PoseProblem:
    def __init__(self, namespace:List[HandEnum], N=5, const_A=True) -> None:
        
        self.end_point_problem = EndPointProblem()

        pose_names = ["x", "y", "z", "th_x", "th_y", "th_z"]

        self.constraint_name_struct = DualArmNames(namespace, pose_names)

        # A list of all the constraint names
        self.constraint_names = self.constraint_name_struct.all_names_lst
        # A dictionary containing all the contraint names using the namespace as a key
        self.namespaced_names_dict = self.constraint_name_struct.namespace_dict
        # Dictionary of all the NamedTemporalConstraints with the constraint names as keys
        self.named_constraints = self.create_named_constraints()

        self.program = SimultaneousProgram(N, const_A=const_A)
        # Create an OSQP object
        self._prob = osqp.OSQP()
    
    # Base methods
    def create(self, verbose=False):
        self.program.create(self.named_constraints.values())
        self._prob.setup(P=self.program.P, 
                         q=self.program.q, 
                         A=self.program.A, 
                         l=self.program.l, 
                         u=self.program.u, 
                         verbose=verbose)
        self._res = self._prob.solve()
        return
    
    def initialise(self):
        return
    
    def reset(self):
        return

    def advance(self, end_point_poses: Dict[HandEnum, EndPointPose]):
        # Update the constraints
        for key, item in end_point_poses.items():
            for k in range(6):
                self.update_constraints(k, self.named_constraints[self.namespaced_names_dict[key][k]], item)
        # Advance the internal program matrices
        self.program.advance(self.named_constraints.values())
        
        # Solve the optimisation
        if self.program.const_A:
            self._prob.update(l=self.program.l, u=self.program.u)
        else:
            self._prob.update(A=self.program.A, l=self.program.l, u=self.program.u)
        self._res = self._prob.solve()
        return self._res.x
    
    # Helper methods
    def create_named_constraints(self):
        named_constraints = dict()
        for name in self.constraint_names:
            named_constraints[name] = (NamedTemporalConstraint(name, copy.deepcopy(self.end_point_problem.constraint_dict)))
        return named_constraints

    def update_constraints(self, index: int, constraint_dict: Dict[str, TemporalConstraint], end_point_pose: EndPointPose):
        # Init pose
        constraint_dict.constraint_dict["pos_00"].l = end_point_pose.init_pose.pos[index]
        constraint_dict.constraint_dict["pos_00"].u = constraint_dict.constraint_dict["pos_00"].l
        
        constraint_dict.constraint_dict["vel_00"].l = end_point_pose.init_pose.vel[index]
        constraint_dict.constraint_dict["vel_00"].u = constraint_dict.constraint_dict["vel_00"].l

        constraint_dict.constraint_dict["acc_00"].l = end_point_pose.init_pose.acc[index]
        constraint_dict.constraint_dict["acc_00"].u = constraint_dict.constraint_dict["acc_00"].l

        # Mid pose
        # constraint_dict.constraint_dict["pos_05"].l = end_point_pose.mid_pose.pos[index]
        # constraint_dict.constraint_dict["pos_05"].u = constraint_dict.constraint_dict["pos_05"].l

        # End pose
        constraint_dict.constraint_dict["pos_10"].l = end_point_pose.end_pose.pos[index]
        constraint_dict.constraint_dict["pos_10"].u = constraint_dict.constraint_dict["pos_10"].l
        
        constraint_dict.constraint_dict["vel_10"].l = end_point_pose.end_pose.vel[index]
        constraint_dict.constraint_dict["vel_10"].u = constraint_dict.constraint_dict["vel_10"].l

        constraint_dict.constraint_dict["acc_10"].l = end_point_pose.end_pose.acc[index]
        constraint_dict.constraint_dict["acc_10"].u = constraint_dict.constraint_dict["acc_10"].l
