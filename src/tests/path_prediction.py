from directions import Direction

# TODO remove import later
from game_theory import whole_matrix

HALF_PI = 1.5708

class Path_Predicter:
    def __init__(self, update_time=1.0, max_s=0.7, max_a=0.2):
        self.time_delta = update_time
        self.max_speed = max_s
        self.max_angle = max_a
        self.max_distance = self.max_speed * self.time_delta

    def predict_point(self, direction, position, orientation):
        xPart = yPart = new_z = 0
        if (direction == Direction.FORWARD):
            new_z = orientation
        elif (direction == Direction.STRONG_LEFT):
            new_z = orientation + self.max_angle * self.time_delta
        elif (direction == Direction.STRONG_RIGHT):
            new_z = orientation - self.max_angle * self.time_delta
        elif (direction == Direction.LIGHT_LEFT):
            new_z = orientation + self.max_angle / 2 * self.time_delta
        elif (direction == Direction.LIGHT_RIGHT):
            new_z = orientation - self.max_angle /2 * self.time_delta
        
        if (abs(new_z) <= HALF_PI):
            xPart = 1 - abs(new_z) / HALF_PI
            yPart = new_z / HALF_PI
        else:
            tmp_orientation = new_z + HALF_PI if new_z < 0 else new_z - HALF_PI
            xPart = (1 - abs(tmp_orientation) / HALF_PI) * -1
            yPart = tmp_orientation / HALF_PI
        
        new_x = position[0] + xPart * self.max_distance
        new_y = position[1] + yPart * self.max_distance
        return { "x": new_x, "y": new_y, "z": new_z }
        
    def predict_all(self, position, orientation):
        st_left = self.predict_point(Direction.STRONG_LEFT, position, orientation)
        li_left = self.predict_point(Direction.LIGHT_LEFT, position, orientation)
        forward = self.predict_point(Direction.FORWARD, position, orientation)
        st_right = self.predict_point(Direction.STRONG_RIGHT, position, orientation)
        li_right = self.predict_point(Direction.LIGHT_RIGHT, position, orientation)
        return {'left': st_left, 'half_left': li_left,
            'forward': forward, 'half_right': li_right, 'right': st_right,}
    
if __name__ == "__main__":
    startOrient = 0
    startPoint = [0, 0]
    cat_path = Path_Predicter(max_s=0.7, max_a=0.2)
    allp_cat = cat_path.predict_all(startPoint, startOrient)
    #for result in allp_cat.values():
        #print('new_x: ' + str(result['x']) + ' new_y: ' + str(result['y']) + ' new_z:' + str(result['z']))
    mouse_path = Path_Predicter(max_s=0.2, max_a=0.9)
    allp_mouse = mouse_path.predict_all(startPoint, startOrient)
    #for result in allp_mouse.values():
        #print('new_x: ' + str(result['x']) + ' new_y: ' + str(result['y']) + ' new_z:' + str(result['z']))
        
    whole_matrix(allp_cat, allp_mouse)
