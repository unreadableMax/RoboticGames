import collections
import sys
import numpy as np

from directions import *
from game_theory import max_min_solution


from Packages.GameTheory.game_theory import r_cat
from Packages.GameTheory.game_theory import r_mouse

# Constants defined by the task
CAT_MAX_SPEED = 0.4
CAT_MAX_ANGLE = 0.8
MOUSE_MAX_SPEED = 0.35
MOUSE_MAX_ANGLE = 1.0


class Path_Predicter:

    def __init__(self, max_s, max_a, update_time=1.0):
        self.time_delta = update_time
        self.max_speed = max_s
        self.max_angle = max_a
        self.max_distance = self.max_speed * self.time_delta

    # predicts new position and orientation for certatin direction
    def predict_point(self, direction, position, orientation):
        xPart = yPart = new_z = 0

        # calculate the new orientation
        if (direction == Direction.FORWARD):
            new_z = orientation
        elif (direction == Direction.STRONG_LEFT):
            new_z = orientation + self.max_angle * self.time_delta
        elif (direction == Direction.STRONG_RIGHT):
            new_z = orientation - self.max_angle * self.time_delta
        elif (direction == Direction.LIGHT_LEFT):
            new_z = orientation + (self.max_angle / 2) * self.time_delta
        elif (direction == Direction.LIGHT_RIGHT):
            new_z = orientation - (self.max_angle / 2) * self.time_delta

        # calculate the x and y part of the distance the roboter is going to drive
        if (abs(new_z) <= HALF_PI):
            xPart = 1 - abs(new_z) / HALF_PI
            yPart = new_z / HALF_PI
        else:
            tmp_orientation = 2*HALF_PI - abs(new_z)
            tmp_orientation = -tmp_orientation if new_z < 0 else tmp_orientation
            xPart = (1 - (abs(tmp_orientation) / HALF_PI)) * -1
            yPart = tmp_orientation / HALF_PI

        new_x = position[0] + xPart * self.max_distance
        new_y = position[1] + yPart * self.max_distance
        return {"x": new_x, "y": new_y, "z": new_z}

    # return all 5 predicted points in one dictionary
    def predict_all(self, position, orientation):
        st_left = self.predict_point(
            Direction.STRONG_LEFT, position, orientation)
        li_left = self.predict_point(
            Direction.LIGHT_LEFT, position, orientation)
        forward = self.predict_point(Direction.FORWARD, position, orientation)
        st_right = self.predict_point(
            Direction.STRONG_RIGHT, position, orientation)
        li_right = self.predict_point(
            Direction.LIGHT_RIGHT, position, orientation)
        d = collections.OrderedDict()
        d['left'] = st_left
        d['half_left'] = li_left
        d['forward'] = forward
        d['half_right'] = li_right
        d['right'] = st_right
        return d

    # l = v*t
    def predict_point_exact(self, pos, rot, left, r, l):
        a = l/r
        v_r = np.array([1, 0])
        res_point = np.array([0.0, 0.0])
        if left:
            v_r = self.rotate_vector(v_r, rot+np.pi/2.0)
            v_r = v_r*r
            point2r = pos+v_r
            v_rot = -v_r
            v_rot = self.rotate_vector(v_rot, a)
            res_point = point2r + v_rot
            return {"x": res_point[0], "y": res_point[1], "z": rot+a}
        else:
            v_r = self.rotate_vector(v_r, rot-np.pi/2.0)
            v_r = v_r*r
            point2r = pos+v_r
            v_rot = -v_r
            v_rot = self.rotate_vector(v_rot, -a)
            res_point = point2r + v_rot
            return {"x": res_point[0], "y": res_point[1], "z": rot-a}

    # l = v*t        r = Katzenradius oder Mausradius
    def predict_all_exact(self, pos, rot, r, l):

        v_forward = np.array([1, 0])
        v_forward = self.rotate_vector(v_forward, rot)
        forward = np.array(pos) + v_forward*l
        forward = {"x": forward[0], "y": forward[1], "z": rot}

        st_left = self.predict_point_exact(pos, rot, True, r, l)
        li_left = self.predict_point_exact(pos, rot, True, 2.0*r, l)
        st_right = self.predict_point_exact(pos, rot, False, r, l)
        li_right = self.predict_point_exact(pos, rot, False, 2.0*r, l)

        d = collections.OrderedDict()
        d['left'] = st_left
        d['half_left'] = li_left
        d['forward'] = forward
        d['half_right'] = li_right
        d['right'] = st_right
        return d

    # positive winkel = linksdrehung
    def rotate_vector(self, v, a):
        ca = np.cos(a)
        sa = np.sin(a)
        R = np.array([[ca, -sa], [sa, ca]])

        return np.dot(R, v).T

# return the predicted angle for cat or mouse


def maxmin_solution_angle(pos_cat, z_cat, pos_mouse, z_mouse, mouse_or_cat, update_time=1.0):
    if not (mouse_or_cat is 'cat' or mouse_or_cat is 'mouse'):
        print('THE FUNCTION maxmin_solution_angle NEEDS THE INFORMATION cat OR mouse')
        sys.exit()

    # gather all predicted positions for cat
    cat_path = Path_Predicter(CAT_MAX_SPEED, CAT_MAX_ANGLE, update_time)
    #all_paths_cat = cat_path.predict_all(pos_cat, z_cat)
    all_paths_cat = cat_path.predict_all_exact(
        pos_cat, z_cat, r_cat, np.pi/3.0*r_cat)

    # gather all predicted positions for cat
    mouse_path = Path_Predicter(MOUSE_MAX_SPEED, MOUSE_MAX_ANGLE, update_time)
    #all_paths_mouse = mouse_path.predict_all(pos_mouse, z_mouse)
    all_paths_mouse = mouse_path.predict_all_exact(
        pos_mouse, z_mouse, r_mouse, np.pi/3.0*r_mouse)

    angles_cat_mouse = max_min_solution(
        all_paths_cat, all_paths_mouse, CAT_MAX_ANGLE, MOUSE_MAX_ANGLE)
    angle = angles_cat_mouse['c_angle'] if mouse_or_cat == 'cat'else angles_cat_mouse['m_angle']
    return angle

# returns the predicted angle for mouse in a range of [-1, 1]


def angle_change_mouse_scaled(pos_cat, z_cat, pos_mouse, z_mouse, update_time=1.0):
    angle = maxmin_solution_angle(
        pos_cat, z_cat, pos_mouse, z_mouse, 'mouse', update_time)
    flattened_angle = (2 / (2 * MOUSE_MAX_ANGLE)) * angle
    return flattened_angle

# returns the predicted angle for mouse in a range of [-1, 1]


def angle_change_cat_scaled(pos_cat, z_cat, pos_mouse, z_mouse, update_time=1.0):
    angle = maxmin_solution_angle(
        pos_cat, z_cat, pos_mouse, z_mouse, 'cat', update_time)
    flattened_angle = (2 / (2 * CAT_MAX_ANGLE)) * angle
    return flattened_angle


# Just some tests
if __name__ == "__main__":
    cat_z = 0
    startPointCat = [0, 0]
    cat_path = Path_Predicter(max_s=CAT_MAX_SPEED, max_a=CAT_MAX_ANGLE)
    all_paths_cat = cat_path.predict_all(startPointCat, cat_z)
    # for result in all_paths_cat.values():
    # print('new_x: ' + str(result['x']) + ' new_y: ' + str(result['y']) + ' new_z:' + str(result['z']))
    mouse_z = 0
    startPointMouse = [0, 0]
    mouse_path = Path_Predicter(max_s=MOUSE_MAX_SPEED, max_a=MOUSE_MAX_ANGLE)
    all_paths_mouse = mouse_path.predict_all(startPointMouse, mouse_z)
    # for result in all_paths_mouse.values():
    # print('new_x: ' + str(result['x']) + ' new_y: ' + str(result['y']) + ' new_z:' + str(result['z']))

    angles = max_min_solution(
        all_paths_cat, all_paths_mouse, CAT_MAX_ANGLE, MOUSE_MAX_ANGLE)
    print('mouse angle: ' + str(angles['m_angle']))
    print('cat angle: ' + str(angles['c_angle']))

    a_mouse = angle_change_mouse_scaled(
        startPointCat, cat_z, startPointMouse, mouse_z, 1.0)
    print('flattened angle mouse: ' + str(a_mouse))
    a_cat = angle_change_cat_scaled(
        startPointCat, cat_z, startPointMouse, mouse_z, 1.0)
    print('flattened angle mouse: ' + str(a_cat))
