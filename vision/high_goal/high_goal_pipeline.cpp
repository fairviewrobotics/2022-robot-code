#include "high_goal.hpp"
#include "high_goal_pipeline.hpp"

#define PI 3.14159265358979323846

// diagonal field of view of camera (radians)
// TODO: determine
const double camera_diag_fov = 68.5 * PI / 180.0;
// aspect ratio of camera
// TODO: determine
const double camera_aspect_h = 16.0;
const double camera_aspect_v = 9.0;

// height of center of camera off ground (meters)
// TODO: determine
const double camera_height = 1.0;

// angle camera is pointed from the horizontal (radians)
// TODO: determine
const double camera_angle = 0.0;

// height of top of vision target off ground (meters)
const double target_height = 2.6416;
// radius from vision target to center of high goal (meters)
const double target_radius = 0.677926;

using namespace frc::robot::vision;

void HighGoalPipeline::Process(cv::Mat &mat) {
  auto target = find_target_angles(mat, camera_diag_fov, camera_aspect_h, camera_aspect_v);

  if(target) {
    found_target = true;
    yaw = target->yaw;
    pitch = target->pitch;

    center_distance = get_distance_to_target(*target, target_height - camera_height, camera_angle) + target_radius;
  } else {
    found_target = false;
  }
}

HighGoalPipeline::HighGoalPipeline(): found_target(false), pitch(0.0), yaw(0.0), center_distance(0.0) {}
