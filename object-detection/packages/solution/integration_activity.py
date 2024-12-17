from typing import Tuple


def DT_TOKEN() -> str:
    # TODO: change this to your duckietown token
    # dt_token = "PUT_YOUR_TOKEN_HERE"
    dt_token = 'dt1-3nT7FDbT7NLPrXykNJmqrVWv9QLpdUo86SNkw6cb8ptXWG4-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfZriKvyLPBTh37P12J28wWiPBeW1etXEF3'
    return dt_token


def MODEL_NAME() -> str:
    # TODO: change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5n"
    return "yolov5n"


def NUMBER_FRAMES_SKIPPED() -> int:
    # TODO: change this number to drop more frames
    # (must be a positive integer)
    return 2


def filter_by_classes(pred_class: int) -> bool:
    """
    Remember the class IDs:

        | Object    | ID    |
        | ---       | ---   |
        | Duckie    | 0     |
        | Cone      | 1     |
        | Truck     | 2     |
        | Bus       | 3     |


    Args:
        pred_class: the class of a prediction
    """
    # Right now, this returns True for every object's class
    # TODO: Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    # return True
    
    if pred_class in [1, 2, 3]:
        return False
    else:
        return True


def filter_by_scores(score: float) -> bool:
    """
    Args:
        score: the confidence score of a prediction
    """
    # Right now, this returns True for every object's confidence
    # TODO: Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    print(f'score: {score}')
    if score > 0.5:
        return True
    else:
        return False
    # return True


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    # TODO: Like in the other cases, return False if the bbox should not be considered.
    # return True
    print(f"bbox: {bbox}")
    # standard duckietown camera resolution
    h_max = 480
    w_max = 640
    
    # bounding box
    l, t, r, b = bbox
    
    # filter out the bounding boxes that are too small
    if (r - l) * (b - t) < 200:
        return False
    
    # define the margins that we want to ignore
    top_margin = 0.25 * h_max
    lr_margin = 0.125 * w_max
    
    # filter out the bounding boxes outside the margins
    if t < top_margin or l < lr_margin or r > w_max - lr_margin or b > h_max:
        print(f"bbox: {bbox}")
        print('object outside the margins')
        return False
    else:
        return True
