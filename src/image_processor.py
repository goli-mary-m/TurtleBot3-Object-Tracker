#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results

# ROS
import rospy
from sensor_msgs.msg import Image
from turtlebot3_object_tracker.srv import Detection, DetectionResponse


class ImageProcessor:

    def __init__(self) -> None:
        
        self.image_msg = Image() # Image message
        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # Subscribe on robot's camera topic
        self.camera_subscriber = rospy.Subscriber("/follower/camera/image", Image, callback=self.camera_listener)

        # Instantiate YOLO object detector/classifier model
        self.model: YOLO = YOLO('../yolo/yolov5nu.pt')
        self.results: Results = self.model(self.image_np)

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # Setup human detection service
        self.human_detection_server = rospy.Service('detection', Detection, self.human_detection)
        self.bounding_boxes = []

        self.update_view()


    def human_detection(self, req):

        self.bounding_boxes = []
        res = DetectionResponse()

        for result in self.results:
            boxes = result.boxes
            for box in boxes:

                cls = box.cls.item()
                x1, y1, x2, y2 = box.xyxy[0]
                self.bounding_boxes.append([cls, x1, y1, x2, y2])
                
                # Response of service
                label_req = req.label
                label_box = self.model.names[cls]
                if(label_box == label_req):
                    res.box_x = x1
                    res.box_y = y1
                    res.box_width  = x2 - x1
                    res.box_height = y2 - y1
                    res.image_width = self.image_res[1]
                    res.image_height = self.image_res[0]
                    res.in_sight_of_robot = True
                else:
                    res.in_sight_of_robot = False
        return res


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)


    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                # Update results
                self.results = self.model(self.image_np)

                # Draw object bounding boxes on frame
                frame = copy.deepcopy(self.image_np)
                annotator = Annotator(frame)

                for bbox in self.bounding_boxes:
                    cls, x1, y1, x2, y2 = bbox
                    color = (128,128,128) # Gray color
                    annotator.box_label([x1, y1, x2, y2], label=self.model.names[cls], color=color)  

                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass



if __name__ == "__main__":

    rospy.init_node("image_processor", anonymous=True)
    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()


