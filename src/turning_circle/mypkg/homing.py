import numpy as np


def homing(position, orientation, target, p_distance=3.0, p_rot=2.0, max_speed=0.2, max_rot_speed=0.9, min_rot_speed=.5):

    v_target = target - position
    M_rot = np.matrix([[np.cos(orientation), -np.sin(orientation)],
                       [np.sin(orientation), np.cos(orientation)]])
    v_target = np.asarray(np.dot(M_rot.T, v_target).T)

    distance_to_target = np.linalg.norm(v_target)

    v_view = np.array([1, 0])

    # -1 -> bitte um 180 grad drehen. 1 -> perfekt, so bleiben
    rot_error = np.dot(v_view, v_target / distance_to_target)

    # ------------------Regelung----------------------
    lin_speed = p_distance*distance_to_target*rot_error
    velocity_adjustment_lin = np.clip(lin_speed, -max_speed, max_speed)

    rot_speed = p_rot * (-.5 * rot_error + .5)  # mapping [-1,1] to [1,0]
    rot_speed = np.clip(rot_speed, min_rot_speed, max_rot_speed)
    velocity_adjustment_ang = np.sign(-v_target[1])*rot_speed

    return (velocity_adjustment_lin, velocity_adjustment_ang)
