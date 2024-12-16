from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    # steer_matrix_left = np.random.rand(*shape)
    steer_matrix_left = np.zeros(shape)

    h, w = shape
    steer_matrix_left[:, int(w/3):w] = -1.2 * 0.5
    # ---
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    # steer_matrix_right = np.random.rand(*shape)
    steer_matrix_right = np.zeros(shape)

    h, w = shape
    steer_matrix_right[:, 0:int(w*2/3)] = 1.0 * 0.5
    # ---
    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    # TODO: implement your own solution here
    # mask_left_edge = np.random.rand(h, w)
    # mask_right_edge = np.random.rand(h, w)
    
    # convert image to different format
    # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
    imgrgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(imgrgb, cv2.COLOR_BGR2GRAY)
    
    # dummy ground mask
    mask_ground = np.ones(img.shape, dtype=np.uint8) 
    
      
    # apply GaussianBlur to reduce noise
    sigma = 4
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    
    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)
    
    # define masking range
    white_lower_hsv = np.array([0, 0, 150])       
    white_upper_hsv = np.array([179, 80, 255]) 
    yellow_lower_hsv = np.array([15, 100, 100])      
    yellow_upper_hsv = np.array([35, 255, 255])
    
    
    # color masking
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
    
    # thresholding
    threshold = 25
    mask_mag = (Gmag > threshold)
    
    # edge masking
    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    # print all shape
    # print(mask_ground.shape)
    # print(mask_left.shape)
    # print(mask_mag.shape)
    # print(mask_sobelx_neg.shape)
    # print(mask_sobely_neg.shape)
    # print(mask_yellow.shape)
    
    mask_left_edge = mask_ground * mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_ground * mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white

    return mask_left_edge, mask_right_edge
