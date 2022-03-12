#include "high_goal.hpp"
#include "camera_distort.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ntcore/src/main/native/include/ntcore.h>
#include <networktables/NetworkTable.h>

// define to draw debugging information on images (bounding rectangles on targets)
#define DRAW_DEBUG
// define to apply distortion correction on images
//#define UNDISTORT

//#define WEBCAM

namespace frc::robot::vision {

// calculate angle from linear position on screen ( in radians)
double calcAngle(long value, long centerVal, double focalLen) {
  return atan((double) (value - centerVal) / focalLen);
}

const cv::Scalar hsvLow{57.0, 160.0, 60.0};
const cv::Scalar hsvHigh{100.0, 255.0, 255.0};

const cv::Mat openKernel =
    getStructuringElement(cv::MORPH_RECT, cv::Size{3, 3}, cv::Point{1, 1});

const int closeIters = 2;
const int openIters = 2;

// number of contours to consider (by size)
const size_t numContours = 8;

// draw the outline of a rotated rectangle on image
void drawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect,
                     const cv::Scalar &color) {
  cv::Point2f vertices2f[4];
  rect.points(vertices2f);

  cv::Point points[4];
  for (int i = 0; i < 4; ++i) {
    points[i] = vertices2f[i];
  }

  // draw lines between points
  for (int i = 0; i < 4; i++) {
    cv::line(image, points[i], points[(i + 1) % 4], color);
  }
}

const double coverageExp = 1.5;
// score a contour based on what percentage of rectangle's area the given contour covers this score gives a measurement of how rectangular the contour is (since targets are very rectangular)
double areaCoverScore(const cv::RotatedRect &rect,
                      const std::vector<cv::Point> &contour) {
  auto coverage = cv::contourArea(contour) / abs(rect.size.area());
  return 1.0 - pow(1.0 - coverage, coverageExp);
}

// score a target on it's aspect ratio (should be close to 5"/2" = 2.5)
const double aspectExp = 4.0;
double aspectScore(const cv::RotatedRect &rect) {
  const double w = rect.size.width;
  const double h = rect.size.height;
  auto aspect = cv::max(w, h) / cv::min(w, h);

  return cv::max(1.0 - pow(abs(aspect - 2.5), aspectExp), 0.0);
}

double rotatedRectAngle(const cv::RotatedRect &rect) {
  return rect.size.width < rect.size.height ? rect.angle + 90.0 : rect.angle;
}

// score a target based on it's angle (should be near-ish horizontal)
double targetAngleScore(const cv::RotatedRect &rect) {
  double angle = rotatedRectAngle(rect);
  // expect angle to be in [0, 40] union [140, 180] (withing 40 degrees of horizontal)
  bool hit =
      (angle >= 0.0 && angle <= 40.0) || (angle >= 140.0 && angle <= 180.0);
  return hit ? 1.0 : 0.0;
}

// check if a contour with the given bounding rectangle is a valid target
const double scoreThresh = 0.5;
bool checkTarget(const cv::RotatedRect &rect,
                 const std::vector<cv::Point> &contour) {
  return (aspectScore(rect) + areaCoverScore(rect, contour) +
          targetAngleScore(rect)) /
             3.0 >
         scoreThresh;
}

void process(cv::Mat &image, cv::Mat &dst, double diagFieldView, double aspectH, double aspectV) {
  // calculate camera information
  double aspectDiag = hypot(aspectH, aspectV);

  double fieldViewH = atan(tan(diagFieldView / 2) * (aspectH / aspectDiag)) * 2.0;
  double fieldViewV = atan(tan(diagFieldView / 2) * (aspectV / aspectDiag)) * 2.0;

  double hFocalLen = image.size().width / (2.0 * tan(fieldViewH / 2.0));
  double vFocalLen = image.size().height / (2.0 * tan(fieldViewV / 2.0));

  // convert to hsv
  cv::cvtColor(image, dst, cv::COLOR_BGR2HSV);

  // generate hsv threshold mask
  cv::inRange(dst, hsvLow, hsvHigh, dst);

  // close + open mask to patch up small holes
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, openKernel, cv::Point(-1, -1),
                   closeIters);
  cv::morphologyEx(dst, dst, cv::MORPH_OPEN, openKernel, cv::Point(-1, -1),
                   openIters);

  // find all contours in image
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // order contours by size
  std::sort(contours.begin(), contours.end(),
            [](std::vector<cv::Point> &c1, std::vector<cv::Point> &c2) -> bool {
              return abs(cv::contourArea(c1)) < abs(cv::contourArea(c2));
            });

  // filter and draw bounding rects
  std::vector<cv::RotatedRect> targets;
  for (size_t i = 0; i < cv::min(numContours, contours.size()); i++) {
    auto &contour = contours[i];
    auto rect = cv::minAreaRect(contour);
    auto hit = checkTarget(rect, contour);

    if (hit) {
#ifdef DRAW_DEBUG
      drawRotatedRect(image, rect, cv::Scalar(255.0, 0.0, 0.0));
#endif
      targets.push_back(rect);
    } else {
#ifdef DRAW_DEBUG
      drawRotatedRect(image, rect, cv::Scalar(0.0, 0.0, 255.0));
#endif
    }
  }

  // pick out highest point on contours
  int center_x = 0;
  int center_y = image.size().height;
  cv::RotatedRect *high_target = nullptr;
  for (auto &target : targets) {
    if (target.boundingRect().y < center_y) {
      center_x = rotatedRectAngle(target) < 90
                     ? target.boundingRect().x
                     : target.boundingRect().x + target.boundingRect().width;
      center_y = target.boundingRect().y;
      high_target = &target;
    }
  }

  if (high_target != nullptr) {
#ifdef DRAW_DEBUG
    cv::circle(image, cv::Point(center_x, center_y), 5,
               cv::Scalar(0.0, 125.0, 255.0));
#endif

    // calculate pitch and yaw angles
    double yaw = calcAngle(center_x, image.size().width / 2.0, hFocalLen);
    double pitch = calcAngle(center_y, image.size().height / 2.0, vFocalLen);

    printf("yaw: %f, pitch: %f\n", yaw, pitch);
  }
}
}

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

  cv::Mat mask;
  process(linear, mask);
  imshow("Mask", mask);
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
    process(linear, mask);
    cv::imshow("Image", linear);
    cv::waitKey(1);
  }

}
#endif