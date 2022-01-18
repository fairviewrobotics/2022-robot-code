#include "high_goal.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;

const cv::Size blurRadius{3, 3};
const cv::Scalar hsvLow{57.0, 220.0, 60.0};
const cv::Scalar hsvHigh{100.0, 255.0, 255.0};

const cv::Mat openKernel = getStructuringElement(cv::MORPH_RECT, cv::Size{3, 3}, cv::Point{1, 1});

const cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size{61, 61}, cv::Point(-1, -1));

const int dilateIters = 1;
const int closeIters = 4;
const int openIters = 1;

void process(Mat& image, Mat& dst) {

  // convert to hsv
  cv::cvtColor(image, dst, cv::COLOR_BGR2HSV);

  // generate hsv threshold mask
  cv::inRange(dst, hsvLow, hsvHigh, dst);

  // dilate to attempt to combine tape sections
  cv::erode(dst, dst, Mat{}, cv::Point(-1, -1), 3);
  cv::dilate(dst, dst, kernel, cv::Point(-1, -1), dilateIters);


  //cv::erode(dst, dst, kernel, cv::Point(-1, -1), dilateIters);

  // clone + open mask to patch up small holes
  /*cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, openKernel, cv::Point(-1, -1), closeIters);
  cv::morphologyEx(dst, dst, cv::MORPH_OPEN, openKernel, cv::Point(-1, -1), openIters);*/

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::drawContours(image, contours, -1, cv::Scalar(0.0, 0.0, 255.0), 2);
}

/*int main(int argc, char *argv[])
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
  cv::resize(img, img, cv::Size(640, 480));
  cv::blur(img, img, cv::Size{3, 3});
  imshow("Init", img);
  Mat mask;
  process(img, mask);
  imshow("Processed", mask);
  while(waitKey(0) != 'q'){}; // Wait for a keystroke in the window
  return 0;
}*/

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