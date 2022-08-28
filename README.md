Welcome to the 2036 code repo!

# Development values
--- 
- *High-Octane*: Good code is fast to onboard to, fast to add to, and fast to build and deploy.
- *Modular*: Not just good code, but good software is extendable and configurable.
- *Robust*: Good code stands up to any obstacle it faces.

# robot/
---
This directory contains all of the WPILib code that runs on the roboRIO. It is a standard Kotlin project, and can be built and deployed using Gradle.

# vision/
---

## JSON configuration
The configuration for cameras is done through JSON. Do not modify the config.robot.json; this is the config that is used on the RaspberryPi on the robot for production code.
Instead, write your own config file (perhaps named config.development.json) and pass it into the script.

Here is the format that VisionInstance expects:

```
{
    "team": This must be the team number (2036). Right now this is not needed but may be used later down the line for network verification/identification.
    "cameras": [
        This is the array of camera instances.
        {
            "name": Give the camera a name. THIS IS HOW YOU'LL IDENTIFY AND GET A CAMERA'S FRAMES IN CODE!
            "id": The id of the camera. Can either be identified through an index (for windows) or through a device path. (for linux, macos(?), and robotpi)
            "props": [
                This is an array of properties that will be set when the VisionInstance first runs.
                {
                    "key": What property you want to set. You can find the available ones to tune [here](https://docs.opencv.org/4.6.0/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d).
                    "value": What value you want to set the property to.
                }
            ]
        }
    ] 
}
```