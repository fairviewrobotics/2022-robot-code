import cv2
import numpy as np
from math import tan, atan, hypot
from typing import List

# kernel for morphological transformations
kernel = np.ones((5, 5), np.uint8)


def get_filtered_contours(config: dict, source_image: np.ndarray) -> List[np.ndarray]:
    """
    Get a list of contours in an image after applying an HSV filter and morphological
    transformations.

    :param config: the configuration parameters for the filtering
    :param source_image: the image to find contours within
    :return: a list of arrays that describe contours as a collection of points on the image
    """

    # copy source image and apply transformations to the copy
    vision_config = config['vision_config']
    image = source_image.copy()

  #  image = apply_morph(vision_config, image)
    image = apply_hsv_filter(vision_config, image)

    return find_contours(image)


def apply_hsv_filter(config: dict, image: np.ndarray) -> np.ndarray:
    """
    Get a bit mask from the image containing points that pass an HSV filter.

    :param config: the configuration parameters for the filtering
    :param image: the image to filter
    :return: a new image containing the bit mask
    """
    # convert image to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # generate threshold mask
    hsv_low = (config['hsv_low_h'], config['hsv_low_s'], config['hsv_low_v'])
    hsv_high = (config['hsv_high_h'], config['hsv_high_s'], config['hsv_high_v'])

    return cv2.inRange(image, hsv_low, hsv_high)


def apply_morph(config: dict, image: np.ndarray) -> np.ndarray:
    """
    Apply morphological transformations to the image. This involves blurring the image and then sharpening it to fill
    holes and reduce noise. Depending on configuration settings, this may also do an additional blurring (dilation) of
    the image.

    :param config: the configuration parameters for the filtering
    :param image: the image to apply morphological transformations to
    :return: the new, transformed image
    """
    if config['do_dilate']:
        image = cv2.dilate(image, kernel)
    opened = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return opened#cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def find_contours(image: np.ndarray) -> List[np.ndarray]:
    """
    Find contours within a bitmask.

    :param image: the image to find contours within
    :return: a list of contours
    """
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # return contours sorted by size
    return sorted(contours, key=lambda c: cv2.contourArea(c), reverse=True)
