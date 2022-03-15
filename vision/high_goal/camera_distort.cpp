#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/calib3d.hpp>

namespace frc::robot::vision {
double camera_matrix_data[3][3] = {
    {7.1591177416446146e+02, 0.0, 0.0},
    {0.0,
     7.1591177416446146e+02, 0.0},
    {3.1950000000000000e+02, 2.3950000000000000e+02, 1.0}
};
const cv::Mat camera_matrix(3, 3, CV_64F, camera_matrix_data);

double camera_distort_data[5][1] = {
    {-1.2428234909953326e+00},
    {4.0908066988722256e+00},
    {0.0},
    {0.0},
    {-7.8038722924918309e+00}};
const cv::Mat distortion_coefficients(5, 1, CV_64F, camera_distort_data);

void undistort(const cv::Mat &src, cv::Mat &dst) {
  cv::undistort(src, dst, camera_matrix, distortion_coefficients);
}

}