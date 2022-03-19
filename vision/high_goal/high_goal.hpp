#include <opencv2/core.hpp>
#include <optional>

namespace frc::robot::vision {
// Vision pipeline configuration
struct VisionConfig {
  // hsv thresholds
  double hsvLowH, hsvLowS, hsvLowV;
  double hsvHighH, hsvHighS, hsvHighV;
  // number of open / close iterations
  int open_iters, close_iters;
  // contour score threshold
  double score_thresh;
};

// Camera field of view and aspect information
struct CameraFOV {
  // diagonal field of view of camera (radians)
  double diag_field_view;
  // aspect ratio of camera
  double aspect_h, aspect_v;
};

// The angle of a target in camera space.
struct TargetAngle {
  // The horizontal angle of the target from the center in radians.
  // Angles to the right of the vertical are positive.
  double yaw;
  // The vertical angle of the target from the center in radians.
  // Angles above the horizontal are positive.
  double pitch;
};

/**
 * Search the given image for a vision target and return the angle it is located at (pitch and yaw).
 * @param image The source image possibly containing a vision target. If DRAW_DEBUG is defined, target locations will be drawn on image.
 * @param diagFieldView The diagonal field of view (in radians) of the camera.
 * @param aspectH The horizontal aspect of the camera. aspectH is relative to aspectV.
 * @param aspectV The vertical aspect of the camera. aspectV is relative to aspectH.
 * @return The location of the target if found.
 */
  std::optional<TargetAngle> find_target_angles(const VisionConfig& config, const CameraFOV &camera_fov, cv::Mat &image);

/**
 * Calculate the distance to the target.
 * @param target angles to the target
 * @param target_height height of the target relative to the height of the camera
 * @param camera_angle vertical angle of the camera above the horizontal
 * @return the distance to the target. This is the distance to the edge of target (the vision target), not the center.
 */
  double get_distance_to_target(const TargetAngle& target, double target_height, double camera_angle);

}