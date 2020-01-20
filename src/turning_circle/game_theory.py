import math
import numpy as np
from directions import *

from path_length import get_path_length

v_cat = 0.7
omega_cat = 0.2
r_cat = 2.7  # experimentel ermittelt

v_mouse = 0.2
omega_mouse = 0.9
r_mouse = 0.18  # experimentel ermittelt

# Constants
DISTANCE_DISTRIBUTION = 5 # distributes the distance from 0 to DISTANCE_DISTRIBUTION

'''
calculates the NormalMatrix based on the cost of distance from mouse to cat
each matrix entry holds two value
the first value is the payout of the cat and the second is the payout of the mouse
max_payout defines the distribution of 0 to max -> (0,...,max_payout)
'''
def payout_distance(cat_predictions, mouse_predictions, max_payout):
    # create an empty matrix
    m_dist = np.zeros((5, 5, 2))
    
    # fill the matrix with distance and store max distance
    max_dist = 0
    for row_idx, cat_coords in enumerate(cat_predictions.values()):
        for col_idx, mouse_coords in enumerate(mouse_predictions.values()):
            dist = math.sqrt( (cat_coords['x'] - mouse_coords['x'])**2 +
                              (cat_coords['y'] - mouse_coords['y'])**2 )
            max_dist = dist if dist > max_dist else max_dist
            m_dist[row_idx][col_idx][0] = dist
            m_dist[row_idx][col_idx][1] = dist

    # change distribution and flip the payout for cat because distance is bad for cat
    temp = m_dist.reshape(-1, 2)
    for tuple in temp:
        tuple[0] = - tuple[0] / max_dist * max_payout
        tuple[1] = tuple[1] / max_dist * max_payout
    return m_dist

# TODO not finished!
def payout_infront_or_behind(cat_predictions, mouse_predictions, max_payout):
    cat_angle = -HALF_PI - HALF_PI / 2
    if (abs(cat_angle) <= HALF_PI):
        xPart = 1 - abs(cat_angle) / HALF_PI
        yPart = abs(cat_angle) / HALF_PI
    else:
        tmp_angle = 2 * HALF_PI - abs(cat_angle)
        xPart = 1 - tmp_angle / HALF_PI
        yPart = tmp_angle / HALF_PI
    slope = yPart / xPart
    # get the signe of the slope by checking the quadrant of the radiant circle
    if (cat_angle > 0 and cat_angle < HALF_PI) or (cat_angle  < -HALF_PI):
        slope *= -1
    print(slope)


def payout_path_length(cat_predictions, mouse_predictions):
    # create an empty matrix
    m_path_length = np.zeros((5, 5, 2))
    
    for row_idx, cat_coords in enumerate(cat_predictions.values()):
        for col_idx, mouse_coords in enumerate(mouse_predictions.values()):
            cat_pos = [cat_coords['x'], cat_coords['y']]
            mouse_pos = [mouse_coords['x'], mouse_coords['y']]
            path_cat_to_mouse = get_path_length(
                r_cat, cat_coords['z'], cat_pos, mouse_pos)
            payout_mouse = path_cat_to_mouse
            payout_cat = -path_cat_to_mouse
            
            m_path_length[row_idx][col_idx][0] = payout_cat
            m_path_length[row_idx][col_idx][1] = payout_mouse
    
    return m_path_length


'''
adds all payout matrices together and returns the result
'''
def get_normalForm(cat_predictions, mouse_predictions):
    m_empty = np.zeros((5, 5, 2))
    # m_dist = payout_distance(cat_predictions, mouse_predictions, DISTANCE_DISTRIBUTION)
    m_path_length = payout_path_length(cat_predictions, mouse_predictions)
    m_whole = m_empty + m_path_length # here one can add other payouts
    return m_whole


'''
takes as input the matrix in normalform
calculates the best startegie by applying the MaxMinSolution
returns the angle to move for cat an mouse as dictionary
'''
def max_min_solution(cat_predictions, mouse_predictions, max_cat_angle, max_mouse_angle):
    m = get_normalForm(cat_predictions, mouse_predictions)
    
    # calculates best strategie for mouse
    max_pay_cat_per_col = None
    best_idx_mouse = None
    for col_idx in range(5):    # the columns are the strategies of the mouse
        max_pay = 0
        for row_idx in range(5):
            cat_payment = m[row_idx][col_idx][0]
            max_pay = cat_payment if cat_payment > max_pay else max_pay
        if max_pay_cat_per_col == None or max_pay_cat_per_col > max_pay:
            best_idx_mouse = col_idx
            max_pay_cat_per_col = max_pay
    mouse_angle = Direction.get_angle_from_index(best_idx_mouse, max_mouse_angle)
    
    # calculates best strategie for cat
    max_pay_mouse_per_row = None
    best_idx_cat = None
    for row_idx in range(5):   # the rows are the strategies of the cat
        max_pay = 0
        for col_idx in range(5):
            mouse_payment = m[row_idx][col_idx][1]
            max_pay = mouse_payment if mouse_payment > max_pay else max_pay
        if max_pay_mouse_per_row == None or max_pay_mouse_per_row > max_pay:
            best_idx_cat = row_idx
            max_pay_mouse_per_row = max_pay
    cat_angle = Direction.get_angle_from_index(best_idx_cat, max_cat_angle)
    
    return {'c_angle': cat_angle, 'm_angle': mouse_angle}
    


if __name__ == "__main__":
    payout_infront_or_behind(0, 0, 0)
