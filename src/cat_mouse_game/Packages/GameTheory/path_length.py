import numpy as np

# calculate r:
#r = v/omega


def pay_out_function(r, orientation, pos, target_pos):
    turning = how_strong_have_to_turn(r, orientation, pos, target_pos)
    path = get_path_length(r, orientation, pos, target_pos)

    return turning*path + 2*turning


def get_path_length(r, orientation, pos, target_pos):

    # calculate p_target, relative to robot:
    M_transform = np.array([[np.cos(orientation), -np.sin(orientation)],
                            [np.sin(orientation), np.cos(orientation)]])
    target = np.array(target_pos)-np.array(pos)
    p_target = np.asarray(np.dot(M_transform.T, target).T)

    # it doesnt metter if y is positive or negative
    p_target = np.array([p_target[0], np.abs(p_target[1])])

    c = np.linalg.norm(p_target-np.array([0, r]))

    robot_radius = 0.4
    secure_distance = 2*robot_radius + .1
    dist2target = np.linalg.norm(p_target)

    if dist2target < secure_distance:
        return dist2target - secure_distance

    if c < r:
        # target is inside the turning cycle
        return 2.0 * np.pi * r + (r-c)
        vr = p_target - np.array([0, r])
        vre = vr / np.linalg.norm(vr)
        p_target = np.array(vre*r+[0, r])
        c = r

    gamma = np.arccos(np.abs(p_target[0])/c)

    alpha = np.arccos(r/c)

    if p_target[0] > 0:
        if p_target[1] < r:
            betha = np.pi/2.0 - gamma - alpha
        else:
            betha = np.pi/2.0 + gamma - alpha
    else:
        if p_target[1] > r:
            betha = 3.0*np.pi/2.0 - gamma - alpha
        else:
            betha = 3.0*np.pi/2.0 + gamma - alpha

    #print("betha=", np.rad2deg(betha), "alpha=", np.rad2deg(alpha))

    u = betha*r

    d = np.sin(alpha)*c

    return (d + u)


def how_strong_have_to_turn(r, orientation, pos, target_pos):
    # calculate p_target, relative to robot:
    M_transform = np.array([[np.cos(orientation), -np.sin(orientation)],
                            [np.sin(orientation), np.cos(orientation)]])
    target = np.array(target_pos)-np.array(pos)
    p_target = np.asarray(np.dot(M_transform.T, target).T)

    robot_radius = 0.4
    secure_distance = 2*robot_radius + .1
    dist2target = np.linalg.norm(p_target)

    if dist2target < secure_distance:
        return 1.0

    view_vector = np.array([1, 0])
    e_target = p_target / dist2target
    return -(np.dot(view_vector, e_target)-1.0)/2.0
