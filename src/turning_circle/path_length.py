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
        return 3.0/2.0 * np.pi * r + 2*r

    # gamma ist immer positiv
    gamma = np.arctan((p_target[1]-r) / p_target[0])

    alpha = np.arccos(r/c)

    betha = np.pi/2.0 - alpha + gamma

    u = betha*r

    d = np.sin(alpha)*c

    return d + u
