from .VisionInstance import VisionInstance

class VisionLayer():    
    def __init__(self, visionInstance: VisionInstance):
        self.visionInstance = visionInstance

    def log(self, str: str):
        self.visionInstance.log(f"{type(self).__name__} | {str}")

    def run(self):
        "Run every frame."
        pass
    
    def debug(self):
        "If debug is enabled, then this also runs every frame."
        pass

    def stream(self):
        "If stream is enabled, then this runs every frame and returns a frame to display on the web client."
        pass

    def end(self):
        "Run when vision process is terminated."
        pass