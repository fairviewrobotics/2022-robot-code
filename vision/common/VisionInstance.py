import sys
import cv2 as cv
import numpy as np
import json
class VisionInstance():
    cameraSources = {}

    def log(self, str: str):
        """Print a logging message"""
        print(f'[LOG] {str}')

    def error(self, str: str, e: Exception = None):
        """Print an error message and exit. Accepts an optional Exception that can also be printed out."""
        print(f'[ERROR] {str}')
        if not (e is None):
            print(f'[ERROR] Here is the exception message: {e.with_traceback()}')
        sys.exit(-1)
    
    def __init__(self, config_string: str):
        """Initializes a VisionInstance."""
        self.log("Beginning vision instance initalization...")
        
        config_json = json.loads(config_string)

        if not ("cameras" in config_json):
            self.error("Failed to find cameras array in config json.")
            
        for cameraJson in config_json["cameras"]:
            try:
                cameraName = cameraJson["name"]
                cameraId = cameraJson["id"]
                camera = cv.VideoCapture(cameraId)

                if not camera.isOpened():
                    self.error(f"Failed to open camera {cameraName} with id {cameraId}.")

                if "props" in cameraJson:
                    for prop in cameraJson["props"]:
                        key = prop["key"]
                        value = prop["value"]
                        try:
                            camera.set(eval("cv." + key), value)
                            self.log(f"Set {cameraName} property cv.{key} to {value}")
                        except Exception as e:
                            self.error(f"Failed to set {cameraName} property {key} to {value}")
                self.cameraSources[cameraName] = camera
            except Exception as e:
                self.error(f"Failed to load camera:\n{json.dumps(cameraJson, indent=4)}", e)

        self.log(f'Success! VisionInstance was intialized.')

    def get(self, camera: str) -> np.ndarray:
        """Returns a camera's capture frame. The frame will be a numpy array."""
        if camera in self.cameraSources:
            ret, frame = self.cameraSources[camera].read()
            if not ret:
                self.error(f"Cannot recieve frame of camera {camera}. Stream end?")
            else:
                return frame
        else:
            self.error(f"Tried to get frame of {camera}, which does not exist.")

    def cleanup(self):
        self.log("Beginning cleanup...")
        for _, camera in self.cameraSources.items():
            camera.release()
