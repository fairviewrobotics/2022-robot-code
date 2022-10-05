import collections
from socket import socket
import cv2
import math
import numpy as np
import tensorflow as tf
import json
from networktables import NetworkTables
import sys
import imagezmq
# sys.path.append("..")
# from common import VisionLayer

SCORE_THRESHOLD = 0.575

class BBox(collections.namedtuple('BBox', ['xmin', 'ymin', 'xmax', 'ymax'])):
    """Bounding box.
    Represents a rectangle which sides are either vertical or horizontal, parallel
    to the x or y axis.
    """
    __slots__ = ()

    def scale(self, sx, sy):
        """Returns scaled bounding box."""
        return BBox(xmin=sx * self.xmin,
                    ymin=sy * self.ymin,
                    xmax=sx * self.xmax,
                    ymax=sy * self.ymax)


class BallVision():
    def __init__(self, camera_name, camera_id):

        #Image ZMQ stuff
        self.sender = imagezmq.ImageSender(connect_to='tcp://10.20.36.100:5555')
        self.name = socket.gethostname()


        self.log("Loading tflite model for ball vision...")
        self.cameraSource
        try:
            cameraName = camera_name
            cameraId = camera_id
            camera = cv.VideoCapture(cameraId)

            if not camera.isOpened():
                self.error(f"Failed to open camera {cameraName} with id {cameraId}.")

            self.cameraSource = camera
        except Exception as e:
            self.error(f"Failed to load camera:\n{cameraName}", e)

        self.log(f'Success! VisionInstance was intialized.')
        try:
            model_path = "BallVision/four/output_tflite_graph_edgetpu.tflite"
            self.interpreter = tf.lite.Interpreter(model_path, experimental_delegates=[tf.lite.load_delegate('libedgetpu.so.1')])
            self.hardware_type = "Coral Edge TPU"
        except Exception as e:
            self.log("Failed to create Coral interpreter, switching to unoptimized for ball vision")
            model_path = "BallVision/four/output_tflite_graph.tflite"
            self.interpreter = tf.lite.Interpreter(model_path)
            self.hardware_type = "Unoptimized"

        self.interpreter.allocate_tensors()

        self.labels = ["Red", "Blue"]
        self.temp_entry = []

        # set up network tables
        self.log("Getting networktables instance for ball vision")
        NetworkTables.initialize(server="roborio-2036-frc.local")
        self.table = NetworkTables.getTable("ML")

        self.ball_data_entry = self.table.getEntry("ballData")

        # camera_to_ball_distance_vert = distance between the camera and ball (units_unknown) measured using tape measurer 
        self.camera_to_ball_distance_vert = 0

        # camera_angle = angle btw (grav vector rotated 90deg) and camera boresight (measured with iphone) (radians)
        self.camera_angle = 0

    def check_box(self, box):
        width = box[2] - box[0]
        height = box[3] - box[1]
        return height != 0 and abs(1 - width / height) <= 0.3

    def run(self):

        frame = None
        camera = "BallVision"

        ret, this_frame = self.cameraSource.read()
        if not ret:
            self.error(f"Cannot recieve frame of camera {camera}. Stream end?")
        else:
            frame = this_frame
                
        scale = self.set_input(frame)
        self.interpreter.invoke()
        boxes, class_ids, scores, _, _ = self.get_output(scale)
        width, height = self.input_size()
        resized = cv2.resize(frame, (width, height))

        ballJSON = [] #json containing data about the balls found
        self.log(f"{boxes} | {class_ids} | {scores}")
        for i, box in enumerate(boxes):
            if scores[i] > SCORE_THRESHOLD and self.check_box(box):
                class_id = class_ids[i]

                oneBallData = {} #contains data for one individual ball found

                alpha = 80 # deg, half the camera's field of view
                focalLen = 1.0 / (2.0 * math.tan(math.radians(alpha) / 2.0)) # constant, focal length of camera

                center = (box[2] + box[0]) / 2 # screen center x as a proportion
                offset = (center - 0.5) # units in percentage of image, distance to center
                target_yaw = math.atan(offset / focalLen) # plate scale , theta_y

                center = (box[3] + box[1]) / 2 # screen center y as a proportion
                offset = (center - 0.5) # units in percentage of image, distance to center
                focalLen = 1.0 / (2.0 * math.tan(math.radians(alpha) / 2.0)) # constant, focal length of camera
                target_pitch = math.atan(offset / focalLen) # plate scale
                distance = self.get_distance_to_target(target_pitch)

                oneBallData['target_yaw'] = target_yaw #adds yaw, distance, pitch to the dictionary containing info for one ball
                oneBallData['distance'] = distance
                oneBallData['pitch'] = math.degrees(target_pitch)
                oneBallData['score'] = float(scores[i])
                oneBallData['color'] = self.labels[int(class_id)]

                ballJSON.append(oneBallData) #add data for one individual ball to the json
                ballJSONString = json.dumps(ballJSON) #convert json to string
                self.log(f"{ballJSONString}")
                self.ball_data_entry.setString(ballJSONString)

                if np.isnan(class_id):
                    continue

                class_id = int(class_id)
                if class_id not in range(len(self.labels)):
                    continue
                
                color = (0, 0, 255)

                resized = self.label_frame(resized, self.labels[class_id], box, scores[i], width, height, color)

        # self.output.putFrame(resized)
        self.output = resized

    def stream(self):
        # imagezmq logic
        self.sender.send_image(self.name, self.output)

    def input_size(self):
        """Returns input image size as (width, height) tuple."""
        _, height, width, _ = self.interpreter.get_input_details()[0]['shape']
        return width, height


# double get_distance_to_target(const TargetAngle& target, double target_height, double camera_angle) {
#   return target_height / tan(target.pitch + camera_angle);
# }
    def get_distance_to_target(self, target_pitch):
        # y-axis = gravity vector 
        # x-axis = vector normal to gravity vector 
        # target_pitch = theta_x
        return self.camera_to_ball_distance_vert / math.tan(target_pitch + self.camera_angle)

    def set_input(self, frame):
        """Copies a resized and properly zero-padded image to the input tensor.
        Args:
          frame: image
        Returns:
          Actual resize ratio, which should be passed to `get_output` function.
        """
        width, height = self.input_size()
        h, w, _ = frame.shape
        new_img = np.reshape(cv2.resize(frame, (width, height)), (1, width, height, 3))
        self.interpreter.set_tensor(self.interpreter.get_input_details()[0]['index'], np.copy(new_img))
        return width / w, height / h

    
    def output_tensor(self, i):
        tensor = self.interpreter.get_tensor(self.interpreter.get_output_details()[i]['index'])
        return np.squeeze(tensor)


    def get_output(self, scale):
        boxes = self.output_tensor(1)
        class_ids = self.output_tensor(3)
        scores = self.output_tensor(0)

        width, height = self.input_size()
        image_scale_x, image_scale_y = scale
        x_scale, y_scale = width / image_scale_x, height / image_scale_y
        return boxes, class_ids, scores, x_scale, y_scale

    
    def label_frame(self, frame, object_name, box, score, x_scale, y_scale, color):
        ymin, xmin, ymax, xmax = box
        score = float(score)
        bbox = BBox(xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax).scale(x_scale, y_scale)

        height, width, channels = frame.shape
        # check bbox validity
        if not 0 <= ymin < ymax <= height or not 0 <= xmin < xmax <= width:
            return frame

        ymin, xmin, ymax, xmax = int(bbox.ymin), int(bbox.xmin), int(bbox.ymax), int(bbox.xmax)
        self.temp_entry.append({"label": object_name, "box": {"ymin": ymin, "xmin": xmin, "ymax": ymax, "xmax": xmax},
                                "confidence": score})

        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 4)

        # Draw label
        # Look up object name from "labels" array using class index
        label = '%s: %d%%' % (object_name, score * 100)  # Example: 'person: 72%'
        label_size, base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
        label_ymin = max(ymin, label_size[1] + 10)  # Make sure not to draw label too close to top of window
        cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10), (xmin + label_size[0], label_ymin + base - 10),
                      color, cv2.FILLED)
        # Draw label text
        cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        return frame
    
if __name__ == "main":
  cameraID = sys.argv[1]
  ballVision = BallVision("Ball Vision", cameraID)
  while True:
    ballVision.run()