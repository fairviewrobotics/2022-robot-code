import numpy as np


class Rect:
    def __init__(self, box_points: np.ndarray):
        """
        Represents a rotated rectangle based.

        :param box_points: a numpy array of clockwise-ordered points (bottom right, bottom left, top left, top right)
        """

        self.box_points = box_points

        # calculate center point
        # use built-in sum instead of numpy sum because numpy will add individual point data into a single *value*
        # while build-in sum will add together points as vectors to get a single *point* as the result
        self.center = sum(self.box_points) / 4  # boxes have 4 corners

        # calculate rectangle height, width, area, and rotation angle
        side = self.box_points[1:3]  # second and third points form a vertical side
        self.height = np.linalg.norm(side)

        side = self.box_points[:2]  # first two points form a horizontal side
        self.width = np.linalg.norm(side)

        self.area = self.height * self.width

        bottom_right = self.box_points[0]
        bottom_left = self.box_points[1]
        self.angle = np.angle(bottom_right - bottom_left, deg=True)

        # tall rectangles are considered rotated an additional 90 degrees
        if self.width < self.height:
            self.angle += 90.0

    @property
    def center_x(self) -> float:
        return self.center[0]

    @property
    def center_y(self) -> float:
        return self.center[1]
