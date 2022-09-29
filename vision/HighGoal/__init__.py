from common import VisionLayer, VisionInstance
from .high_goal import find_targets
from .pipeline import apply_hsv_filter, apply_morph
from .target_scoring import check_target
import cv2 as cv

highGoalConfig = {
  "team": 2036,

  "camera_fov": {
    "diag_field_view": 1.500983,
    "aspect_h": 16.0,
    "aspect_v": 9.0
  },

  "vision_config": {
    "hsv_low_h": 62,
    "hsv_low_s": 200.0,
    "hsv_low_v": 200.0,

    "hsv_high_h": 87.5,
    "hsv_high_s": 255.0,
    "hsv_high_v": 255.0,

    "open_iters": 1,
    "close_iters": 1,

    "do_dilate": False,
    "size_rel_thresh": 0.018,

    "score_thresh": 0.5
  },

  "vision_layout": {
    "camera_height": 1.0,
    "camera_angle": 0.0,
    "target_height": 2.6416,
    "target_radius": 0.677926
  }
}

class HighGoal(VisionLayer):
    def __init__(self, visionInstance: VisionInstance):
        super().__init__(visionInstance)
        self.output = None

    def run(self):
        super().run()
        frame = self.visionInstance.get("HighGoal")

        height, width, color = frame.shape
        foundTargets = high_goal.find_targets(highGoalConfig, frame, 0.5, width, height / width)

        for found in foundTargets:
            rect = found[1]
            x1, y1, x2, y2 = int(rect.box_points[0][0]), int(rect.box_points[0][1]), int(rect.box_points[2][0]), int(rect.box_points[2][1])
            frame = cv.rectangle(frame, (x1, y1), (x2,y2), (10,255,0), 4)

        self.output = frame

    def stream(self):
        super().stream()
        return ("HighGoal", self.output)