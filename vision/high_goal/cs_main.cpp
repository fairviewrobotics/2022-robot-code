#include <iostream>
#include <string>
#include <networktables/NetworkTableInstance.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/core.hpp>
#include "high_goal.hpp"

// Camera field of view and aspect information
struct CameraFOV {
  // diagonal field of view of camera (radians)
  double diag_field_view;
  // aspect ratio of camera
  double aspect_h, aspect_v;
};

// Camera / target physical layout
struct VisionLayout {
  // height of center of camera off ground (meters)
  double camera_height;
  // angle camera is pointed from the horizontal (radians)
  double camera_angle;
  // height of top of vision target off ground (meters)
  double target_height;
  // radius from vision target to center of high goal (meters)
  double target_radius;
};

class Vision {
  int team;
  CameraFOV camera_fov{};
  VisionLayout layout{};

  nt::NetworkTableInstance ntinst;
  nt::NetworkTableEntry target_found;
  nt::NetworkTableEntry yaw;
  nt::NetworkTableEntry pitch;
  nt::NetworkTableEntry center_distance;
  frc::CameraServer *cs;
  cs::UsbCamera camera;
  cs::CvSink cvSink;
  cs::CvSource cvSource;
  cv::Mat camera_mat;

public:
  Vision(int team, const std::string &video_path, CameraFOV camera_fov, VisionLayout layout) {
    this->team = team;
    this->camera_fov = camera_fov;
    this->layout = layout;

    // start network tables client
    std::cout << "Connecting to Network Tables\n";
    ntinst = nt::NetworkTableInstance::GetDefault();
    ntinst.StartClientTeam(team);
    ntinst.StartDSClient();
    std::cout << "Network Tables connected\n";

    // get network table entries we will be setting
    auto table = ntinst.GetTable("high_goal");
    target_found = table->GetEntry("target_found");
    yaw = table->GetEntry("yaw");
    pitch = table->GetEntry("pitch");
    center_distance = table->GetEntry("center_distance");

    // start camera server
    std::cout << "Starting CameraServer";
    cs = frc::CameraServer::GetInstance();

    // open camera and set appropriate mode
    std::cout << "Opening camera " << video_path << "\n";
    camera = cs->StartAutomaticCapture("High Goal Vision", video_path);
    if(!camera.SetResolution(640, 480)) {
      std::cerr << "Failed to set camera resolution to 640 x 480\n";
    } else {
      std::cout << "Set camera resolution to 640x480\n";
    }
    if(!camera.SetVideoMode(cs::VideoMode::PixelFormat::kYUYV, 640, 480, 30)) {
      std::cerr << "Failed to set camera mode to YUYV 640x480 30fps\n";
    } else {
      std::cout << "Set camera mode to YUYV\n";
    }

    // get cv sink for camera (so that we can get cv::Mat from camera)
    cvSink = cs->GetVideo(camera);
    cvSource = cs->PutVideo("High Goal Output", 640, 480);
  }

  // Get an image from the camera, process it, and write vision results to network tables
  void run() {
    if(!cvSink.GrabFrame(camera_mat)) {
      std::cerr << "Couldn't get frame from camera\n";
      return;
    }

    auto target = frc::robot::vision::find_target_angles(camera_mat, camera_fov.diag_field_view, camera_fov.aspect_h, camera_fov.aspect_v);
    if(target) {
      target_found.SetBoolean(true);
      yaw.SetDouble(target->yaw);
      pitch.SetDouble(target->pitch);

      auto distance = frc::robot::vision::get_distance_to_target(*target, layout.target_height - layout.camera_height, layout.camera_angle) + layout.target_radius;
      center_distance.SetDouble(distance);
    } else {
      target_found.SetBoolean(false);
    }
    cvSource.PutFrame(camera_mat);
  }
};

int main(int argc, char **argv) {
  if(argc != 3) {
    std::cerr << "Usage: " << argv[0] << " team video_dev\n";
    return 1;
  }

  CameraFOV camera_fov{
      68.5 * 3.14159 / 180.0,
      16.0,
      9.0
  };
  VisionLayout layout{
      /* TODO: Get camera height */ 1.0,
      /* TODO: get camera angle */ 0.0,
      2.6416,
      0.677926
  };

  Vision vision{static_cast<int>(strtol(argv[1], nullptr, 10)), std::string(argv[2]), camera_fov, layout};

  while(true) {
    vision.run();
  }
}