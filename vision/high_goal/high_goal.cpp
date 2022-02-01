#include "high_goal.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;

const cv::Scalar hsvLow{57.0, 160.0, 60.0};
const cv::Scalar hsvHigh{100.0, 255.0, 255.0};

const cv::Mat openKernel = getStructuringElement(cv::MORPH_RECT, cv::Size{3, 3}, cv::Point{1, 1});

const int closeIters = 2;
const int openIters = 2;

// number of contours to consider (by size)
const size_t numContours = 8;

// draw the outline of a rotated rectangle on image
void drawRotatedRect(Mat &image, const RotatedRect &rect, const cv::Scalar& color) {
  cv::Point2f vertices2f[4];
  rect.points(vertices2f);

  cv::Point points[4];
  for(int i = 0; i < 4; ++i){
    points[i] = vertices2f[i];
  }

  // draw lines between points
  for(int i = 0; i < 4; i++) {
    cv::line(image, points[i], points[(i + 1) % 4], color);
  }
}


const double coverageExp = 1.5;
// score a contour based on what percentage of rectangle's area the given contour covers
// this score gives a measurement of how rectangular the contour is (since targets are very rectangular)
double areaCoverScore(const RotatedRect &rect, const std::vector<cv::Point>& contour) {
  auto coverage = cv::contourArea(contour) / abs(rect.size.area());
  return 1.0 - pow(1.0 - coverage, coverageExp);
}

// score a target on it's aspect ratio (should be close to 5"/2" = 2.5)
const double aspectExp = 4.0;
double aspectScore(const RotatedRect &rect) {
  const double w = rect.size.width;
  const double h = rect.size.height;
  auto aspect = max(w, h) / min(w, h);

  return max(1.0 - pow(abs(aspect - 2.5), aspectExp), 0.0);
}

// score a target based on it's angle (should be near-ish horizontal)
double targetAngleScore(const RotatedRect &rect) {
  double angle = rect.size.width < rect.size.height ? rect.angle + 90.0 : rect.angle;
  // expect angle to be in [0, 40] union [140, 180] (withing 40 degrees of horizontal)
  bool hit = (angle >= 0.0 && angle <= 40.0) || (angle >= 140.0 && angle <= 180.0);
  return hit ? 1.0 : 0.0;
}

// check if a contour with the given bounding rectangle is a valid target
const double scoreThresh = 0.5;
bool checkTarget(const RotatedRect &rect, const std::vector<cv::Point>& contour) {
  return (aspectScore(rect) + areaCoverScore(rect, contour) + targetAngleScore(rect)) / 3.0 > scoreThresh;
}

void process(Mat& image, Mat& dst) {
  // convert to hsv
  cv::cvtColor(image, dst, cv::COLOR_BGR2HSV);

  // generate hsv threshold mask
  cv::inRange(dst, hsvLow, hsvHigh, dst);

  // close + open mask to patch up small holes
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, openKernel, cv::Point(-1, -1), closeIters);
  cv::morphologyEx(dst, dst, cv::MORPH_OPEN, openKernel, cv::Point(-1, -1), openIters);

  // find all contours in image
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // order contours by size
  std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> &c1, std::vector<cv::Point> &c2) -> bool {
    return abs(cv::contourArea(c1)) < abs(cv::contourArea(c2));
  });

  // filter and draw bounding rects
  std::vector<RotatedRect> targets;
  for(size_t i = 0; i < min(numContours, contours.size()); i++) {
    auto& contour = contours[i];
    auto rect = cv::minAreaRect(contour);
    auto hit = checkTarget(rect, contour);

    if(hit) {
      drawRotatedRect(image, rect, cv::Scalar(255.0, 0.0, 0.0));

      targets.push_back(rect);
    } else {
      drawRotatedRect(image, rect, cv::Scalar(0.0, 0.0, 255.0));
    }
  }

  //

  //cv::drawContours(image, contours, -1, cv::Scalar(0.0, 0.0, 255.0), 2);
}

#ifndef WEBCAM
int main(int argc, char *argv[])
{
  if(argc != 2) {
    std::cout << "Usage: " << argv[0] << " image" << std::endl;
    return 1;
  }

  std::string image_path = samples::findFile(argv[1]);
  Mat img = imread(image_path, IMREAD_COLOR);
  if(img.empty())
  {
    std::cout << "Could not read the image: " << image_path << std::endl;
    return 1;
  }
  //cv::resize(img, img, cv::Size(640, 480));
  Mat mask;
  process(img, mask);
  imshow("Mask", mask);
  cv::imshow("Image", img);
  while(waitKey(0) != 'q'){}; // Wait for a keystroke in the window
  return 0;
}
#endif

#ifdef WEBCAM
int main() {
  cv::VideoCapture cap;

  cap.open(2);
  if(!cap.isOpened()) {
    std::cout << "Cannot open webcam" << std::endl;
  }

  cap.set(cv::CAP_PROP_EXPOSURE, -4);

  Mat img;

  while(true) {
    cap.read(img);

    cv::resize(img, img, cv::Size(640, 480));
    Mat mask;
    process(img, mask);
    cv::imshow("Image", img);
    cv::imshow("Masked Targets", mask);
    cv::waitKey(1);
  }

}
#endif