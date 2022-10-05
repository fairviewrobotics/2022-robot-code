import cv2
import math
import numpy as np

from .rectangle import Rect

# number of contours to consider (chosen by size)
num_contours = 8


# scoring exponents
class ScoringExps:
    COVERAGE = 1.5
    ASPECT = 0.8


def area_coverage_score(rect: Rect, contour: np.ndarray) -> float:
    """
    Return a score for a contour based on what percentage of rectangle's area the given contour covers. This score gives
    a measurement of how rectangular the contour is. Since targets are very rectangular, this score helps determine
    which contours are most likely to mark real targets.

    :param rect: the minimum bounding box around the contour of the target
    :param contour: an array of points that make up the contour of the target
    :return: the score of a target based on its area coverage by its bounding box
    """
    coverage = cv2.contourArea(contour) / rect.area
    return 1.0 - (1.0 - coverage) ** ScoringExps.COVERAGE


def aspect_ratio_score(rect: Rect) -> float:
    """
    Score a target based on its aspect ratio. Aspect ratios should be close to 5"/2" = 2.5. These measurements are
    based on the dimensions of the retro-reflective tape on the upper hub.

    :param rect: the minimum bounding box around the target
    :return: the score of a target based on its aspect ratio
    """
    w = rect.width
    h = rect.height
    aspect = min(w, h) / max(w, h)

    return max(1.0 - (abs(aspect - 2.6)) ** ScoringExps.ASPECT, 0.0)


def target_angle_score(rect: Rect) -> float:
    """
    Score a target rectangle based on its angle. Target angles are expected to be in [0, 40]U[140, 180] (i.e. within
    40 degrees of horizontal). This returns 1.0 if this is the case and 0.0 otherwise.
    """

    if (0.0 < (rect.angle + 90)< 40.0) or (140.0 < (rect.angle + 90) < 180):
        return 1.0
    return 0.0


def target_size_score(rect: Rect, image_size: float, relative_thresh: float) -> float:
    """
    Score a target based on its relative size in the image. If a target is too small based on the passed threshold,
    then it receives a score of 0.0; otherwise, it receives a score of 1.0.

    :param rect: the minimum bounding box around the target
    :param image_size: the size of the image as its larger dimension
    :param relative_thresh: a proportion of image_size for calculating the size threshold

    :return: the score of the target based on its size
    """
    thresh = image_size * relative_thresh
    if max(rect.width, rect.height) >= thresh:
        return 1.0
    else:
        return 0.0


def check_target(rect: Rect,
                 contour: np.ndarray,
                 score_thresh: float,
                 image_size: float,
                 size_relative_thresh: float) -> bool:
    """
    Check if a contour with a given bounding rectangle (rect) is a valid target based on a score threshold
    (score_thresh).

    :param rect: the minimum rotated rectangle around the contour
    :param contour: an array of points to be considered as a target
    :param score_thresh: the minimum scoring threshold for the contour to be considered a valid target
    :param image_size: the size of the image as its larger dimension
    :param size_relative_thresh: a proportion of image_size. See target_scoring.target_size_score

    :return: whether the potential target exceeds the given threshold
    """
    return (
        aspect_ratio_score(rect) +
        area_coverage_score(rect, contour) + 
        target_angle_score(rect) +
        2.0 * target_size_score(rect, image_size, size_relative_thresh)
    ) / 5.0 > score_thresh
