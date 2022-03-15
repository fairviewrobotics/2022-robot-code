#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "high_goal.hpp"

// test vision on image files or a webcam

#define PI 3.14159265358979323846

using namespace frc::robot::vision;

//#define WEBCAM

#ifndef WEBCAM
int main(int argc, char *argv[])
{
  if(argc != 2) {
    std::cout << "Usage: " << argv[0] << " image" << std::endl;
    return 1;
  }

  std::string image_path = cv::samples::findFile(argv[1]);
  cv::Mat img = imread(image_path, cv::IMREAD_COLOR);
  if(img.empty())
  {
    std::cout << "Could not read the image: " << image_path << std::endl;
    return 1;
  }

  cv::resize(img, img, cv::Size(640, 480));
  cv::Mat linear;
#ifdef UNDISTORT
  undistort(img, linear);
#else
  linear = img;
#endif

  auto target = find_target_angles(linear, 68.5 * PI / 180.0, 16.0, 9.0);
  if(target) {
    printf("yaw: %lf, pitch: %lf\n", target->yaw, target->pitch);
    printf("distance (m): %lf\n", get_distance_to_target(*target, 1.74625, 0.518134));
  } else {
    printf("no target found\n");
  }
  cv::imshow("Image", linear);
  while(cv::waitKey(0) != 'q'){}; // Wait for a keystroke in the window
  return 0;
}
#endif

#ifdef WEBCAM
int main() {

  cv::VideoCapture cap;

  cap.open(0);
  if(!cap.isOpened()) {
    std::cout << "Cannot open webcam" << std::endl;
  }

  cap.set(cv::CAP_PROP_EXPOSURE, 125);

  cv::Mat img;

  while(true) {
    cap.read(img);

    cv::resize(img, img, cv::Size(640, 480));
    cv::Mat linear;
#ifdef UNDISTORT
    undistort(img, linear);
#else
    linear = img;
#endif
    cv::Mat mask;
    double yaw, pitch;
    if(process(linear, mask, 68.5 * PI / 180.0, 16.0, 9.0, &yaw, &pitch)) {
    }
  }

}
#endif