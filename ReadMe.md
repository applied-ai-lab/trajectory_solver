# Trajectory Solver

Author [Alexander Mitchell](https://github.com/mitch722) (2024)

## 0. Overview

This is a python interface for a creating spline trajectories.

## 1. Usage

For dual arm manipulation you can setup and solve the optimisation as follows.


### Initialise the Solver and Create the Optimisation

The two lines below create the spline solver object:

```python3
namespace = [HandEnum.LEFT, HandEnum.RIGHT]
spline_N = 8

ee_traj = EeTrajOpt(namespace, spline_N, True)
ee_traj.create(verbose=False)
```


### Define the Start, Mid and End Poses

A single spline is parameterised by the initial pose, velocity, acceleration, a mid-pose, and end pose, velocity and acceleration.

These values are all set to zero on initialisation.


In src/trajectory_solver/PoseTypes.py:

```python3
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
```

Timings govern the duration and phase relationship of the spline:

```python3
class RelativeTimings:
    def __init__(self, phase_offset=0.0, duration=20, dt=0.02) -> None:
        self.phase_offset = phase_offset
        self.duration = duration
        self.dt = dt
```

### Optimise the Spline

The spline params are optimised by calling:

```python3
# Advance ee trajectory
timed_splines = ee_traj.advance(poses, relative_times)
```

### Decode the Spline

This method produces the pose, velocity and acceleration of the spline per hand:

```python3
# Get trajectories
traj_dict = timed_splines.traj(0.0, duration)


pos_l = traj_dict[HandEnum.LEFT].pos
vel_l = traj_dict[HandEnum.LEFT].vel
acc_l = traj_dict[HandEnum.LEFT].acc
    
pos_r = traj_dict[HandEnum.RIGHT].pos
vel_r = traj_dict[HandEnum.RIGHT].vel
acc_r = traj_dict[HandEnum.RIGHT].acc
```