import trajoptpy.math_utils as mu
import numpy as np


def traj_collisions(traj, robot, n=100):
    """
    Returns the set of collisions.
    manip = Manipulator or list of indices
    """
    traj_up = mu.interp2d(np.linspace(0, 1, n),
                          np.linspace(0, 1, len(traj)), traj)
    env = robot.GetEnv()
    col_times = []

    with robot:
        for (i, row) in enumerate(traj_up):
            robot.SetActiveDOFValues(row)
            col_env = env.CheckCollision(robot)
            col_self = robot.CheckSelfCollision()
            if col_env and col_self:
                col_times.append(i)
        return col_times


def traj_is_safe(traj, robot, n=100):
    return len(traj_collisions(traj, robot, n)) == 0
