import numpy as np

from trajectory_solver.SplineConstraints import (ConstraintType, TemporalConstraint, SplineConstraints)


def test():

    # Create the constraint list
    pos_0 = TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.POS)
    vel_0 = TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.VEL)
    acc_0 = TemporalConstraint(t=0.0, u=0.0, l=0.0, type=ConstraintType.ACC)

    pos_05 = TemporalConstraint(t=0.0, u=0.5, l=0.5, type=ConstraintType.POS)

    pos_1 = TemporalConstraint(t=1.0, u=1.0, l=1.0, type=ConstraintType.POS)
    vel_1 = TemporalConstraint(t=1.0, u=0.0, l=0.0, type=ConstraintType.VEL)
    acc_1 = TemporalConstraint(t=1.0, u=0.0, l=0.0, type=ConstraintType.ACC)

    # Constraint list
    constraint_lst = [pos_0, vel_0, acc_0, pos_05, pos_1, vel_1, acc_1]

    # Spline Constraint Handler
    N = 5 # Spline order
    spline_constraints = SplineConstraints(N)

    program_matrices = spline_constraints.create_qp(constraint_lst)

    # Write out the target matrices
    A_tar = np.array([[ 0.,  0.,  0.,  0.,  0.,  1.],
                      [ 0.,  0.,  0.,  0.,  1.,  0.],
                      [ 0.,  0.,  0.,  2.,  0.,  0.],
                      [ 0.,  0.,  0.,  0.,  0.,  1.],
                      [ 1.,  1.,  1.,  1.,  1.,  1.],
                      [ 5.,  4.,  3.,  2.,  1.,  0.],
                      [20., 12.,  6.,  2.,  0.,  0.]])
    l_tar = np.array([0., 0., 0., 0.5, 1., 0., 0.])
    u_tar = l_tar

    np.testing.assert_allclose(program_matrices.A.todense(), A_tar, rtol=1e-3, atol=0) 
    np.testing.assert_allclose(program_matrices.l, l_tar, rtol=1e-3, atol=0) 
    np.testing.assert_allclose(program_matrices.u, u_tar, rtol=1e-3, atol=0)

    # Update the constraints
    pos_0.u, pos_0.l = 1.0, 1.0
    pos_0.t = 1.0

    vel_0.t = 1.0
    acc_0.t = 1.0

    pos_05.u, pos_05.l = 1.5, 1.5
    pos_05.t = 1.5

    pos_1.u, pos_1.l = 2.0, 2.0
    vel_1.t = 2.0
    acc_1.t = 2.0

    # Constraint list
    constraint_lst = [pos_0, vel_0, acc_0, pos_05, pos_1, vel_1, acc_1]

    for constraint in constraint_lst:
        spline_constraints.advance_qp(constraint.row, program_matrices, constraint)

    # Write out the target matrices
    A_tar = np.array([[ 1.,  1.,  1.,  1.,  1.,  1.],
                      [ 5.,  4.,  3.,  2.,  1.,  0.],
                      [20., 12.,  6.,  2.,  0.,  0.],
                      [ 7.59375, 5.0625, 3.375, 2.25, 1.5, 1.],
                      [ 1.,  1.,  1.,  1.,  1.,  1.],
                      [ 80.,  32.,  12.,  4.,  1.,  0.],
                      [160., 48.,  12.,  2.,  0.,  0.]])
    l_tar = np.array([1., 0., 0., 1.5, 2., 0., 0.])
    u_tar = l_tar

    np.testing.assert_allclose(program_matrices.A.todense(), A_tar, rtol=1e-3, atol=0) 
    np.testing.assert_allclose(program_matrices.l, l_tar, rtol=1e-3, atol=0) 
    np.testing.assert_allclose(program_matrices.u, u_tar, rtol=1e-3, atol=0)

    return 0


if __name__ == "__main__":
    test()
