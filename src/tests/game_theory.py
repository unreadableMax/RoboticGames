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
    for col_idx, cat_coords in enumerate(cat_predictions.values()):
        for row_idx, mouse_coords in enumerate(mouse_predictions.values()):
            dist = math.sqrt( (cat_coords['x'] - mouse_coords['x'])**2 +
                              (cat_coords['y'] - mouse_coords['y'])**2 )
            max_dist = dist if dist > max_dist else max_dist
            m_dist[col_idx][row_idx][0] = dist
            m_dist[col_idx][row_idx][1] = dist
    temp = m_dist.reshape(-1, 2)
    for tuple in temp:
        tuple[0] = tuple[0] / max_dist * max_payout
        tuple[1] = tuple[1] / max_dist * max_payout
    return m_dist

def payout_

def whole_matrix(cat_predictions, mouse_predictions):
    m_empty = np.zeros((5, 5, 2))
    m_dist = payout_distance(cat_predictions, mouse_predictions, DISTANCE_DISTRIBUTION)
    m_whole = m_empty + m_dist
    print(m_whole)
    
