#include <vision/VisionPipeline.h>

/**
 * Pipeline to run high goal vision on an image and record results.
 */
class HighGoalPipeline : public frc::VisionPipeline {
public:
  // if a target was found
  bool found_target;
  // target angles in radians
  double pitch, yaw;
  // distance to center of target in meters
  double center_distance;

  HighGoalPipeline();

  void Process(cv::Mat& mat) override;
};