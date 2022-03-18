#include <iostream>
#include <string>
#include <networktables/NetworkTableInstance.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/core.hpp>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include "high_goal.hpp"

#define PI 3.14159265358979323846

/*
   JSON format:
   {
       "team": <team number>,
       "camera": {
           "path": <path, e.g. "/dev/video0">
           "pixel format": <"MJPEG", "YUYV", etc>   // optional
           "width": <video mode width>              // optional
           "height": <video mode height>            // optional
           "fps": <video mode fps>                  // optional
           "brightness": <percentage brightness>    // optional
           "white balance": <"auto", "hold", value> // optional
           "exposure": <"auto", "hold", value>      // optional
           "properties": [                          // optional
               {
                   "name": <property name>
                   "value": <property value>
               }
           ],
           "stream": {                              // optional
               "properties": [
                   {
                       "name": <stream property name>
                       "value": <stream property value>
                   }
               ]
           }
       },
       "camera_fov": {
          "diag_field_view": <camera's diagonal field of view, in radians>
          "aspect_h": <horizontal component of aspect>
          "aspect_v": <vertical component of aspect>
       },
       "vision_layout": {
          "camera_height": <height of camera off ground in meters>
          "camera_angle": <angle of camera from horizontal in radians>
          "target_height": <height of vision target off ground in meters>
          "target_radius": <distance from target to high goal center in meters>
       },
       "vision_config": {
          "hsv_[low/high]_[h/s/v]": <HSV threshold value>
          "[open/close]_iters": <morphological open close iterations>
          "score_thresh": <contour score threshold to be counted as target>
       }
   }
 */

using namespace frc::robot::vision;

static const char *config_file = "/boot/frc_high_goal.json";

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

// Camera config information
struct CameraConfig {
  std::string path;
  wpi::json config;
  wpi::json stream_config;
};

wpi::raw_ostream& config_error() {
  return wpi::errs() << "Error parsing config file " << config_file << ": ";
}

// Read vision layout from the config
std::optional<VisionLayout> read_config_vision_layout(const wpi::json& config) {
  VisionLayout layout;

  try {
    layout.camera_height = config.at("camera_height").get<double>();
    layout.camera_angle = config.at("camera_angle").get<double>();
    layout.target_height = config.at("target_height").get<double>();
    layout.target_radius = config.at("target_radius").get<double>();
  } catch(const wpi::json::exception& e) {
    config_error() << "could not parse vision layout: " << e.what() << "\n";
    return {};
  }

  return layout;
}

// Read CameraFOV from the config
std::optional<CameraFOV> read_config_camera_fov(const wpi::json& config) {
  CameraFOV fov;

  try {
    fov.diag_field_view = config.at("diag_field_view").get<double>();
    fov.aspect_h = config.at("aspect_h").get<double>();
    fov.aspect_v = config.at("aspect_v").get<double>();
  } catch (const wpi::json::exception &e) {
    config_error() << "could not parse camera fov: " << e.what() << "\n";
    return {};
  }

  return fov;
}

// Read VisionConfig from the config
std::optional<VisionConfig> read_config_vision_config(const wpi::json& config) {
  VisionConfig vs;

  try {
    vs.hsvLowH = config.at("hsv_low_h").get<double>();
    vs.hsvLowS = config.at("hsv_low_s").get<double>();
    vs.hsvLowV = config.at("hsv_low_v").get<double>();

    vs.hsvHighH = config.at("hsv_high_h").get<double>();
    vs.hsvHighS = config.at("hsv_high_s").get<double>();
    vs.hsvHighV = config.at("hsv_high_v").get<double>();

    vs.open_iters = config.at("open_iters").get<int>();
    vs.close_iters = config.at("close_iters").get<int>();

    vs.score_thresh = config.at("score_thresh").get<double>();
  } catch (const wpi::json::exception &e) {
    config_error() << "could not parse camera fov: " << e.what() << "\n";
    return {};
  }



  return vs;
}

class Vision {
  int team;
  CameraFOV camera_fov{};
  VisionLayout layout{};
  VisionConfig vision_config{};

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
  
  // Read and parse the config file from disk
  std::optional<CameraConfig> read_config() {
    std::error_code ec;
    wpi::raw_fd_istream is(config_file, ec);
    if(ec) {
      wpi::errs() << "could not open " << config_file << ": " << ec.message() << "\n";
      return {};
    }
    
    // parse json
    wpi::json j;
    try {
      j = wpi::json::parse(is);
    } catch (const wpi::json::parse_error& e) {
      config_error() << "byte " << e.byte << ": " << e.what() << '\n';
      return {};
    }
    
    if(!j.is_object()) {
      config_error() << "expected config top level to be json object\n";
      return {};
    }
    
    // parse team number
    try {
      team = j.at("team").get<int>();
    } catch (const wpi::json::exception& e) {
      config_error() << "could not read team number: " << e.what() << "\n";
      return {};
    }
    
    // read fov, vision layout, and vision config
    auto fov = read_config_camera_fov(j.at("camera_fov"));
    if(fov) {
      camera_fov = *fov;
    } else {
      return {};
    }
    
    auto vl = read_config_vision_layout(j.at("vision_layout"));
    if(vl) {
      layout = *vl;
    } else {
      return {};
    }
    
    auto vc = read_config_vision_config(j.at("vision_config"));
    if(vc) {
      vision_config = *vc;
    } else {
      return {};
    }
    
    CameraConfig cam_config;
    auto cam_json = j.at("camera");
    try {
      cam_config.path = cam_json.at("path").get<std::string>();
    } catch (const wpi::json::exception& e) {
      config_error() << "expected cam_json path: " << e.what() << "\n";
      return {};
    }
    
    if(cam_json.count("stream") != 0) {
      cam_config.stream_config = cam_json.at("stream");
    }
    cam_config.config = cam_json;
    
    return cam_config;
  }

public:
  Vision() {
    team = 2036;
    
    auto cam_config = read_config();
    if(!cam_config) {
      wpi::errs() << "Error reading config, exiting\n";
      exit(1);
    }

    // start network tables client
    wpi::outs() << "Connecting to Network Tables\n";
    ntinst = nt::NetworkTableInstance::GetDefault();
    ntinst.StartClientTeam(team);
    ntinst.StartDSClient();
    wpi::outs() << "Network Tables connected\n";

    // get network table entries we will be setting
    auto table = ntinst.GetTable("high_goal");
    target_found = table->GetEntry("target_found");
    yaw = table->GetEntry("yaw");
    pitch = table->GetEntry("pitch");
    center_distance = table->GetEntry("center_distance");

    // start camera server
    wpi::outs() << "Starting CameraServer";
    cs = frc::CameraServer::GetInstance();

    // open camera and set appropriate mode
    wpi::outs() << "Opening camera " << cam_config->path << "\n";
    camera = cs->StartAutomaticCapture("High Goal Vision", cam_config->path);
    camera.SetConfigJson(cam_config->config);

    // get cv sink for camera (so that we can get cv::Mat from camera)
    cvSink = cs->GetVideo(camera);
    cvSource = cs->PutVideo("High Goal Output", 480, 640);
  }

  // Get an image from the camera, process it, and write vision results to network tables
  void run() {
    if(!cvSink.GrabFrame(camera_mat)) {
      wpi::errs() << "Couldn't get frame from camera\n";
      return;
    }
    cv::Mat rotated;
    cv::rotate(camera_mat, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);

    auto target = find_target_angles(vision_config, camera_fov, rotated);
    if(target) {
      target_found.SetBoolean(true);
      yaw.SetDouble(target->yaw);
      pitch.SetDouble(target->pitch);

      auto distance = get_distance_to_target(*target, layout.target_height - layout.camera_height, layout.camera_angle) + layout.target_radius;
      center_distance.SetDouble(distance);
    } else {
      target_found.SetBoolean(false);
    }
    cvSource.PutFrame(rotated);
  }
};

int main(int argc, char **argv) {
  wpi::outs() << "High Goal Vision: starting...\n";
  Vision vision{};
  while(true) {
    vision.run();
  }
}
