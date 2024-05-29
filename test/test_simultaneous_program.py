import copy

import numpy as np

from trajectory_solver.SimultaneousProgram import (SimultaneousProgram, NamedTemporalConstraint, 
                                                   TemporalConstraint, ConstraintType)


def test():

    # Create the constraint list
    constraint_dict = {
        "pos_0": TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.POS),
        "vel_0": TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.VEL),
        "acc_0": TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.ACC),
        "pos_05": TemporalConstraint(t=0.0, u=0.5, l=0.5, type=ConstraintType.POS),
        "pos_1": TemporalConstraint(t=1.0, u=1.0, l=1.0, type=ConstraintType.POS),
        "vel_1": TemporalConstraint(t=1.0, u=0.0, l=0.0, type=ConstraintType.VEL),
        "acc_1": TemporalConstraint(t=1.0, u=0.0, l=0.0, type=ConstraintType.ACC)
    }

    left_x = NamedTemporalConstraint("left_x", copy.deepcopy(constraint_dict))
    right_x = NamedTemporalConstraint("right_x", copy.deepcopy(constraint_dict))

    named_constraints = [left_x, right_x]
    # Spline order
    N = 5

    # Spline Constraints
    spline_program = SimultaneousProgram(N)
    spline_program.create(named_constraints)

    # Update the constraints
    right_x.constraint_dict["pos_0"].l, right_x.constraint_dict["pos_0"].u = 1.0, 1.0
    right_x.constraint_dict["vel_0"].t = 1.0
    right_x.constraint_dict["acc_0"].t = 1.0

    right_x.constraint_dict["pos_05"].u, right_x.constraint_dict["pos_05"].l = 1.5, 1.5
    right_x.constraint_dict["pos_05"].t = 1.5

    right_x.constraint_dict["pos_1"].l, right_x.constraint_dict["pos_1"].u = 2.0, 2.0
    right_x.constraint_dict["vel_1"].t = 1.0
    right_x.constraint_dict["acc_1"].t = 1.0

    named_constraints = [right_x]
    spline_program.advance(named_constraints)

    import pdb
    pdb.set_trace()

    return 0


if __name__ == "__main__":
    test()
