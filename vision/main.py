import argparse
import json
import sys
import numpy as np
import cv2 as cv

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

class VisionLayer():    
    def __init__(self, visionInstance: VisionInstance):
        self.visionInstance = visionInstance

    def run(self):
        "Run every frame."
        pass
    
    def debug(self):
        "If debug is enabled, then this also runs every frame."
        pass

    def end(self):
        "Run when vision process is terminated."
        pass

class ShowGrayscale(VisionLayer):
    def __init__(self, visionInstance: VisionInstance):
        super().__init__(visionInstance)
    
    def run(self):
        super().run()
        self.frame = self.visionInstance.get("BigWanda")

    def debug(self):
        super().debug()
        cv.imshow("frame", cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY))

    def end(self):
        super().end()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='2036 vision process.')
    parser.add_argument('config', metavar='config', type=str, help='file to parse for config')
    parser.add_argument('--debug',action='store_true', help='run debug functions for layers in addition to standard run')
    parser.set_defaults(debug=False)

    args = parser.parse_args()
    
    fileName = args.config

    data = ""
    with open(fileName, 'r') as file:
        data = file.read().replace('\n', '')

    instance = VisionInstance(data)
    # MARK: Layers are added here
    layers = [
        ShowGrayscale(instance)
    ]

    while True:
        for layer in layers:
            layer.run()
            if args.debug:
                layer.debug()

        if cv.waitKey(1) == ord('q'):
            break

    instance.cleanup()
    cv.destroyAllWindows()