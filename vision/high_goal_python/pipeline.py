import cv2
import numpy as np
from math import tan, atan, hypot
from typing import List

# kernel for morphological transformations
kernel = np.ones((3, 3), np.uint8)


def get_filtered_contours(config: dict, source_image: np.ndarray) -> List[np.ndarray]:
    # copy source image and apply transformations to the copy
    vision_config = config['vision_config']
    image = source_image.copy()

    image = apply_hsv_filter(vision_config, image)
    image = apply_morph(vision_config, image)

    return find_contours(image)


def apply_hsv_filter(config: dict, image: np.ndarray) -> np.ndarray:
    # convert image to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # generate threshold mask
    hsv_low = (config['hsv_low_h'], config['hsv_low_s'], config['hsv_low_h'])
    hsv_high = (config['hsv_high_h'], config['hsv_high_s'], config['hsv_high_h'])

    return cv2.inRange(image, hsv_low, hsv_high)


def apply_morph(config: dict, image: np.ndarray) -> np.ndarray:
    if config['do_dilate']:
        image = cv2.dilate(image, kernel)
    opened = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def find_contours(image: np.ndarray) -> List[np.ndarray]:
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # return contours sorted by size
    return sorted(contours, key=lambda c: cv2.contourArea(c))
