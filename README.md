Welcome to the 2036 code repo!

# Development values
- *High-Octane*: Good code is fast to onboard to, fast to add to, and fast to build and deploy.
- *Modular*: Not just good code, but good software is extendable and configurable.
- *Robust*: Good code stands up to any obstacle it faces.

# Poetry
Poetry is a dependency management tool for Python. What makes this a great alternative to pip is that it almost ensures fully reproducible builds and development environments on Windows, MacOS, Linux, and the robot Raspberry Pi. Follow the installation instructions in its [documentation](https://python-poetry.org/docs/). Then, for any project that uses poetry, run `poetry install` to create a virtualenv, and run `poetry env` to boot into that virtualenv. Then run whatever python command you normally use to run the project!

# robot/
This directory contains all of the WPILib code that runs on the roboRIO. It is a standard Kotlin project, and can be built and deployed using Gradle.

# vision/

## tflite-runtime, Poetry, and you
BallVision needs a specific library called tflite-runtime which a) does not work with Poetry and b) only works on Linux. When you need to use this system, install `tflite-runtime` by running `python3 -m pip install tflite-runtime`. I've added nessecary code that only adds this layer if its running on a linux system.

(sidenote: do not let whoever tooled ballvision write any code for the team again)

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