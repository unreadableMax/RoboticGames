import collections
import sys

from directions import *
from game_theory import max_min_solution

# Constants defined by the task
CAT_MAX_SPEED = 0.7
CAT_MAX_ANGLE = 0.2
MOUSE_MAX_SPEED = 0.2
MOUSE_MAX_ANGLE = 0.9

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
        return { "x": new_x, "y": new_y, "z": new_z }
    
    # return all 5 predicted points in one dictionary
    def predict_all(self, position, orientation):
        st_left = self.predict_point(Direction.STRONG_LEFT, position, orientation)
        li_left = self.predict_point(Direction.LIGHT_LEFT, position, orientation)
        forward = self.predict_point(Direction.FORWARD, position, orientation)
        st_right = self.predict_point(Direction.STRONG_RIGHT, position, orientation)
        li_right = self.predict_point(Direction.LIGHT_RIGHT, position, orientation)
        d = collections.OrderedDict()
        d['left'] = st_left
        d['half_left'] = li_left
        d['forward'] = forward
        d['half_right'] = li_right
        d['right'] = st_right
        return d

# return the predicted angle for cat or mouse
def maxmin_solution_angle(pos_cat, z_cat, pos_mouse, z_mouse, mouse_or_cat, update_time=1.0):
    if not (mouse_or_cat is 'cat' or mouse_or_cat is 'mouse'):
        print('THE FUNCTION maxmin_solution_angle NEEDS THE INFORMATION cat OR mouse')
        sys.exit()
    
    # gather all predicted positions for cat
    cat_path = Path_Predicter(CAT_MAX_SPEED, CAT_MAX_ANGLE, update_time)
    all_paths_cat = cat_path.predict_all(pos_cat, z_cat)
    
    # gather all predicted positions for cat
    mouse_path = Path_Predicter(MOUSE_MAX_SPEED, MOUSE_MAX_ANGLE, update_time)
    all_paths_mouse = mouse_path.predict_all(pos_mouse, z_mouse)
    
    angles_cat_mouse = max_min_solution(all_paths_cat, all_paths_mouse, CAT_MAX_ANGLE, MOUSE_MAX_ANGLE)
    angle = angles_cat_mouse['c_angle'] if mouse_or_cat == 'cat'else angles_cat_mouse['m_angle']
    return angle




# Just some tests
if __name__ == "__main__":
    cat_z = 0
    startPointCat = [0, 0]
    cat_path = Path_Predicter(max_s=CAT_MAX_SPEED, max_a=CAT_MAX_ANGLE)
    all_paths_cat = cat_path.predict_all(startPointCat, cat_z)
    #for result in all_paths_cat.values():
        #print('new_x: ' + str(result['x']) + ' new_y: ' + str(result['y']) + ' new_z:' + str(result['z']))
    mouse_z = 0
    startPointMouse = [0, -7]
    mouse_path = Path_Predicter(max_s=MOUSE_MAX_SPEED, max_a=MOUSE_MAX_ANGLE)
    all_paths_mouse = mouse_path.predict_all(startPointMouse, mouse_z)
    #for result in all_paths_mouse.values():
        #print('new_x: ' + str(result['x']) + ' new_y: ' + str(result['y']) + ' new_z:' + str(result['z']))
        
    angles = max_min_solution(all_paths_cat, all_paths_mouse, CAT_MAX_ANGLE, MOUSE_MAX_ANGLE)
    print(angles['c_angle'])
