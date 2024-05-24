import copy

import osqp
import numpy as np
from scipy import sparse
from matplotlib import pyplot as plt 

import pdb

from trajectory_solver.Splines import Splines
from trajectory_solver.SplineConstraints import SplineConstraints


def main():

    # Quintic spline
    N = 5

    # No. constraints
    num_constra = 6

    splineConstraint = SplineConstraints(N)
    print(splineConstraint.pos_constraint(0.0))
    print(splineConstraint.pos_constraint(1.0))

    print(splineConstraint.vel_constraint(0.0))
    print(splineConstraint.vel_constraint(1.0))

    print(splineConstraint.acc_constraint(0.0))
    print(splineConstraint.acc_constraint(1.0))

    P = sparse.eye(N+1)
    q = np.zeros(N+1)

    l = np.zeros(num_constra)
    u = np.zeros(num_constra) 

    constraint_list = []
    # Init constraints
    constraint_list.append(splineConstraint.pos_constraint(0.0))
    constraint_list.append(splineConstraint.vel_constraint(0.0))
    constraint_list.append(splineConstraint.acc_constraint(0.0))
    # Mid position
    constraint_list.append(splineConstraint.pos_constraint(0.5))
    # End constraints
    constraint_list.append(splineConstraint.pos_constraint(1.0))
    constraint_list.append(splineConstraint.vel_constraint(1.0))

    A_np = np.vstack(constraint_list)
    A = sparse.csc_matrix(A_np)

    # Upper and Lower bounds
    l[0] = 0.0
    l[1] = 0.0
    l[2] = 0.0
    l[3] = 0.5
    l[4] = 1.0
    l[5] = 0.0
    # Set as equality constraints
    u = l

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l, u)
    
    res = prob.solve()

    print(res.x)

    num_nodes = 10
    t = np.linspace(0.0, 1.0, num_nodes)

    spline_impl = Splines(N, res.x)

    spline = np.zeros(num_nodes)
    pos = np.zeros(num_nodes * 2 - 1)
    vel = np.zeros(num_nodes * 2 - 1)
    acc = np.zeros(num_nodes * 2 - 1)

    for k in range(num_nodes):
        spline[k] = np.matmul(res.x.reshape(1, -1), np.array([t[k]**5,  t[k]**4, t[k]**3, t[k]**2, t[k]**1, 1.0]).reshape(-1, 1)) 
        pos[k] = spline_impl.pos(t[k])
        vel[k] = spline_impl.vel(t[k])
        acc[k] = spline_impl.acc(t[k])

    # Second spline
    A[3, :] = splineConstraint.pos_constraint(0.4)
    # Upper and Lower bounds
    l[0] = pos[num_nodes - 1]
    l[1] = vel[num_nodes - 1]
    l[2] = acc[num_nodes - 1]
    l[3] = 1.3
    l[4] = 2.0
    l[5] = 0.0
    # Set as equality constraints
    u = l

    # Update 
    prob.update(l=l, u=u)
    res = prob.solve()

    spline_impl.coeffs = res.x

    for k in range(num_nodes - 1):
        pos[k + num_nodes] = spline_impl.pos(t[k + 1])
        vel[k + num_nodes] = spline_impl.vel(t[k + 1])
        acc[k + num_nodes] = spline_impl.acc(t[k + 1])



    _, axes = plt.subplots(3, 1)
    axes[0].plot(spline)
    axes[0].plot(pos, "*")

    axes[0].set_title("Position")

    axes[1].plot(vel)
    axes[1].set_title("Velcity")

    axes[2].plot(acc)
    axes[2].set_title("Acceleration")

    plt.show()

    return 0


if __name__ == "__main__":
    main()

