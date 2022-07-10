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

        # for efficiency, these values are not stored until calculated
        self._area = None
        self._width = None
        self._height = None
        self._angle = None

    @property
    def center_x(self) -> float:
        return self.center[0]

    @property
    def center_y(self) -> float:
        return self.center[1]

    @property
    def area(self) -> float:
        if not (self._area is None):
            return self._area

        self._area = self.width * self.height
        return self._area

    @property
    def width(self) -> float:
        if not (self._width is None):
            return self._width

        side = self.box_points[:2]  # first two points form a horizontal side
        self._width = np.linalg.norm(side)
        return self._width

    @property
    def height(self) -> float:
        if not (self._height is None):
            return self._height

        side = self.box_points[1:3]  # second and third points form a vertical side
        self._height = np.linalg.norm(side)
        return self._height

    @property
    def angle(self) -> float:
        if not (self._angle is None):
            return self._angle

        bottom_right = self.box_points[0]
        bottom_left = self.box_points[1]
        self._angle = np.angle(bottom_right - bottom_left, deg=True)

        # tall rectangles are considered rotated an additional 90 degrees
        if self.width < self.height:
            self._angle += 90.0

        return self._angle
