from typing import Tuple

import numpy as np
import math


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    res = np.zeros(shape=shape, dtype="float32")
    # # these are random values
    # res[100:150, 100:150] = 1
    # res[300:, 200:] = 1
    # ---
    #

    height, width = shape
    half_width = width // 2
    w_margin = 100
    half_height = height // 2
    # strategy 1 (too aggresive)
    forward_dist = half_height -100
    # left half all positive
    # res = np.ones(shape=shape, dtype="float32")
    
    # res[forward_dist:, :half_width] = 1
    # # right half all negative
    # res[forward_dist:, half_width:] = -1

    # res[:, :w_margin] = 0
    # res[:, width - w_margin: width] = 0

    # res[0:forward_dist, :half_width] = -1
    # res[0:forward_dist, half_width:width] = 1
    
    # strategy 2 
    # use a triangle shape for both left half and right half
    # bottom of the image (large height value) is the longer side
    
    scaling = 3.5
    for i in range(half_height, height):
        num_filling =(half_width - (i-half_height)* scaling) 
        num_filling = np.clip(num_filling, 0, half_width).astype(int)
        res[i, num_filling:half_width] = 1
    
    for i in range(half_height, height):
        num_filling = (i-half_height) * scaling
        num_filling = np.clip(num_filling, 0, half_width).astype(int)
        res[i, half_width:half_width+num_filling] = -1
    
    # patch bottom center a square to avoid the robot stuck in the middle
    # patch_height = 200
    # patch_height = half_height
    # patch_width = 100
    # res[height-patch_height:, half_width-patch_width:half_width+patch_width] = -1
    
    # patch the top to let the robot move forward
    forward_dist = 200
    # patch_dist = 75
    patch_dist = half_width-100

    res[:forward_dist, :] = 1
    res[:forward_dist, :patch_dist] = -1
    # res[:forward_dist, width - patch_dist:] = 1
    # res[:forward_dist, width - patch_dist:] = 1
    

    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    res = np.zeros(shape=shape, dtype="float32")
    # # these are random values
    # res[100:150, 100:300] = -1
    # ---
    res = np.flip(get_motor_left_matrix(shape), axis=1)
    return res
