import cv2
import math
import numpy as np
from typing import List, Tuple
from numpy import ndarray
from .pipeline import get_filtered_contours
from .target_scoring import check_target
from .rectangle import Rect


def get_distance_to_target(target_pitch: float, target_height: float, camera_angle: float) -> float:
    """
    Get the forward distance from the camera to the target.

    :param target_pitch: the observed vertical angle between the camera and the target in radians
    :param target_height: the height of the target relative to the camera
    :param camera_angle: the vertical angle of the camera relative to the ground
    :return: the distance to the target in the same units as target_height
    """
    return target_height / math.tan(target_pitch + camera_angle)


def calc_angle(value: float, center_value: float, focal_length: float) -> float:
    """
    Calculate angle from linear position on screen (in radians)

    :param value: the value to calculate the angle to
    :param center_value: the value corresponding to the center of the screen
    :param focal_length: the focal length of the camera

    :return: the angle in radians to the value
    """
    return math.atan((value - center_value) / focal_length)


def find_target_angles(config: dict, image: np.ndarray) -> Tuple[float, float]:
    """
    Find the yaw and pitch of the best target within an image.

    :param config: the configuration dictionary
    :param image: the image to search for targets within
    :return: a tuple containing the yaw and pitch in radians
    """
    # unpack field of view information from config
    camera_fov = config['camera_fov']
    fov_h = camera_fov['aspect_h']
    fov_v = camera_fov['aspect_v']
    fov_diag = camera_fov['diag_field_view']

    # calculate camera information
    aspect_diagonal = math.hypot(fov_h, fov_v)
    field_view_h = math.atan(math.tan(fov_diag / 2.0) * (fov_h / aspect_diagonal)) * 2.0
    field_view_v = math.atan(math.tan(fov_diag / 2.0) * (fov_v / aspect_diagonal)) * 2.0

    h_focal_length = image.size().width / (2.0 * math.tan(field_view_h / 2.0))
    v_focal_length = image.size().height / (2.0 * math.tan(field_view_v / 2.0))

    # find targets
    score_thresh = config['vision_config']['score_thresh']
    size_relative_thresh = config['vision_config']['score_thresh']
    image_size = max(image.shape)
    image_center_x = image.shape[0] / 2.0
    image_center_y = image.shape[1] / 2.0

    targets = find_targets(config, image, score_thresh, image_size, size_relative_thresh)
    highest_target_rect = find_highest_target(image_center_y, targets)

    # calculate angles
    yaw = calc_angle(highest_target_rect.center_x, image_center_x, h_focal_length)
    pitch = -calc_angle(highest_target_rect.center_y, image_center_y, v_focal_length)

    return yaw, pitch


def find_targets(config: dict,
                 image: np.ndarray,
                 score_thresh: float,
                 image_size: float,
                 size_relative_thresh: float) -> List[Tuple[np.ndarray, Rect]]:
    """
    Return a list of targets given an image and some scoring parameters.

    :param config: the configuration parameters for the filtering and camera
    :param image: the image to locate targets within
    :param score_thresh: the minimum score a target must exceed to be returned
    :param image_size: the size of the image as its larger dimension
    :param size_relative_thresh: a proportion of image_size. See target_scoring.target_size_score

    :return: a list of targets as tuples of their contour and rectangle
    """
    contours = get_filtered_contours(config, image)
    targets = []
    for contour in contours:
        rect = Rect(cv2.boxPoints(cv2.minAreaRect(contour)))
        if check_target(rect, contour, score_thresh, image_size, size_relative_thresh):
            targets.append((contour, rect))
    return targets


def find_highest_target(center_y: float, targets: List[Tuple[np.ndarray, Rect]]) -> Rect:
    """
    Get the highest target from a list of targets.

    :param center_y: the y value of the center of the image
    :param targets: a list of targets, each represented with a tuple containing its contour and bounding rect

    :return: a rectangle around the target containing the highest point (or None if no targets are passed)
    """
    absolute_highest = None
    highest_target = None
    for target_contour, target_rect in targets:
        if target_rect.center_y > center_y:  # targets must be above middle of image or else they are false positives
            highest_in_contour = max(target_contour, key=lambda point: point[1])
            if absolute_highest is None or highest_in_contour[1] > absolute_highest[1]:
                absolute_highest = highest_in_contour
                highest_target = target_rect
    return highest_target
