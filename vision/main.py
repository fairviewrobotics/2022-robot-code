import json
import sys
import numpy as np

from cscore import CameraServer

class VisionInstance():
    cameraSinks = {}
    cameraFrames = {}

    def log(self, str: str):
        print(f'[LOG] {str}')

    def error(self, str: str):
        print(f'[ERROR] {str}')
        sys.exit(-1)

    def __init__(self, config_string: str):
        self.log("Beginning vision instance initalization...")
        self.log("Getting CameraServer instance.")
        try:
            self.cs = CameraServer.getInstance()
        except:
            self.error("Failed to get CameraServer Instance.")

        config_json = json.loads(config_string)

        if not ("cameras" in config_json):
            self.error("Failed to find cameras array in config json.")
            

        for camera in config_json.cameras:
            try:
                cameraName = camera["name"]
                camera = self.cs.startAutomaticCapture(name=cameraName,path=camera["path"])
                WIDTH, HEIGHT = camera["width"], camera["height"]
                camera.setResolution(WIDTH, HEIGHT)
                self.cameraSinks[cameraName] = self.cs.getVideo(cameraName)
                self.cameraFrames[cameraName] = np.zeroes(shape=(HEIGHT,WIDTH,3), dtype=np.unit8)
            except:
                self.error(f"Failed to load camera.")

if __name__ == "__main__":
    fileName = ""
    try:
        fileName = sys.argv[1]
    except:
        print("[ERROR] Need file name.")
        sys.exit(-1)

    data = ""
    with open(fileName, 'r') as file:
        data = file.read().replace('\n', '')

    instance = VisionInstance(data)