#include <opencv2/core.hpp>

namespace frc::robot::vision {
extern const cv::Mat camera_matrix;
extern const cv::Mat distortion_coefficients;

// undistort an image
void undistort(const cv::Mat &src, cv::Mat &dst);
}