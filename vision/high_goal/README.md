# High Goal Vision
High goal vision identifies the location of the high goal target in the 2022 FRC game.
It expects a low exposure image of the target illuminated with green light.

## Building
High goal vision can be built in multiple ways.

### Building Raspberry Pi Executable
To build the executable for the raspberry pi, run `make`. This requires the raspberry pi cross compiler to be installed.
See https://github.com/wpilibsuite/raspbian-toolchain/releases for details.

The executable for the pi is called `high_goal_vision`. You need to send this to the pi, and change the `/home/pi/runCamera` shell script to invoke it. You also need to setup a `/boot/frc_high_goal.json` config file with camera information. The `frc_high_goal.json` file contains an example of the format.

### Building Local Tests
The cmake project contains a `vision_test` program that runs vision on a provided image file.
The `sample_images/` folder contains FIRST provided vision samples to use. 
The program is configured with values for camera used for the `Terminal*.png`, `NearLaunchpad*.pnd`, and `FarLaunchpad*.png` images, and should calculate accurate distances.

The cmake project also contains a `camera_calibration` program.
This is opencv's program for generating camera distortion coefficients, and expects a chessboard in the frame.
See https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html for details.
