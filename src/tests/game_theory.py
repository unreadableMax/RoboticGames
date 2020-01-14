import math
import numpy as np
from directions import Direction

DISTANCE_DISTRIBUTION = 5


'''
calculates the NormalMatrix based on the cost of distance of mouse and cat
each matrix entry holds two value
the first value is the payout of the cat and the second is the payout of the mouse

max_payout defines the distribution of 0 to max -> (0, .... max_payout)
'''
def payout_distance(cat_predictions, mouse_predictions, max_payout):
    m_dist = np.zeros((5, 5, 2))
    max_dist = 0
    for row_idx, cat_coords in enumerate(cat_predictions.values()):
        for col_idx, mouse_coords in enumerate(mouse_predictions.values()):
            dist = math.sqrt( (cat_coords['x'] - mouse_coords['x'])**2 +
                              (cat_coords['y'] - mouse_coords['y'])**2 )
            max_dist = dist if dist > max_dist else max_dist
            m_dist[row_idx][col_idx][0] = dist
            m_dist[row_idx][col_idx][1] = dist
            #print('xcat: ' + str(cat_coords['x']) + ' ycat: ' + str(cat_coords['y']) + ' dist: ' + str(dist))
    temp = m_dist.reshape(-1, 2)
    # change distribution and flip the payout for cat because less distance is better
    for tuple in temp:
        tuple[0] = abs(tuple[0] / max_dist * max_payout - max_payout)
        tuple[1] = tuple[1] / max_dist * max_payout
    return m_dist


def get_normalForm(cat_predictions, mouse_predictions):
    m_empty = np.zeros((5, 5, 2))
    m_dist = payout_distance(cat_predictions, mouse_predictions, DISTANCE_DISTRIBUTION)
    m_whole = m_empty + m_dist # here one can add other payouts
    return m_whole


# takes the normalform matrix
# returns the solution angle for cat an mouse
def max_min_solution(cat_predictions, mouse_predictions, max_cat_angle, max_mouse_angle):
    m = get_normalForm(cat_predictions, mouse_predictions)
    
    # calculates best index for mouse
    max_pay_cat_per_col = None
    best_idx_mouse = None
    for col_idx in range(5):
        max_pay = 0
        for row_idx in range(5):
            cat_payment = m[row_idx][col_idx][0]
            max_pay = cat_payment if cat_payment > max_pay else max_pay
        if max_pay_cat_per_col == None or max_pay_cat_per_col > max_pay:
            best_idx_mouse = col_idx
            max_pay_cat_per_col = max_pay
    mouse_angle = Direction.get_angle_from_index(best_idx_mouse, max_mouse_angle)
    
    # calculates best index for cat
    max_pay_mouse_per_row = None
    best_idx_cat = None
    for row_idx in range(5):
        max_pay = 0
        for col_idx in range(5):
            mouse_payment = m[row_idx][col_idx][1]
            max_pay = mouse_payment if mouse_payment > max_pay else max_pay
        if max_pay_mouse_per_row == None or max_pay_mouse_per_row > max_pay:
            best_idx_cat = row_idx
            max_pay_mouse_per_row = max_pay
    cat_angle = Direction.get_angle_from_index(best_idx_cat, max_cat_angle)
    
    return {'c_angle': cat_angle, 'm_angle': mouse_angle}
