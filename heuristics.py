__author__ = 'dirkvangheel'

import numpy as np

def euclidean_distance(n1, n2):
    a, b = n1, n2
    if not isinstance(n1, (np.ndarray, np.generic)):
        a = np.array(n1)
    if not isinstance(n2, (np.ndarray, np.generic)):
        b = np.array(n2)
    return np.linalg.norm(a - b)

