import numpy as np

# calculate r:
#r = v/omega


def get_path_length(r, orientation, pos, target_pos):

    # calculate p_target, relative to robot:
    M_transform = np.array([[np.cos(orientation), -np.sin(orientation)],
                            [np.sin(orientation), np.cos(orientation)]])
    target = np.array(target_pos)-np.array(pos)
    p_target = np.asarray(np.dot(M_transform.T, target).T)

    # it doesnt metter if y is positive or negative
    p_target = np.array([p_target[0], np.abs(p_target[1])])

    c = np.linalg.norm(p_target-np.array([0, r]))

    if c < r:
        # target is inside the turning cycle
        return 2.0 * np.pi * r + (r-c)

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

    return d + u
