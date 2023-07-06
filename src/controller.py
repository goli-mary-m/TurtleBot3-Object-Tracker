#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist

from turtlebot3_object_tracker.srv import *
from math import sqrt


class Controller:

    def __init__(self) -> None:

        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        self.cmd_publisher = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=10)

        self.angular_error = 0

        # The "p" parameter for p-controller
        self.angular_vel_coef = 0.001

        # Create a service proxy for human detection service
        self.detection = rospy.ServiceProxy('detection', Detection)


    def find_angular_error(self, response) -> None:

        box_x = response.box_x
        box_y = response.box_y
        box_width  = response.box_height
        box_height = response.box_height
        image_width  = response.image_width
        image_height = response.image_height

        box_center_x = box_x + (box_width/2)
        box_center_y = box_y + (box_height/2)
        img_center_x = image_width/2
        img_center_y = image_height/2

        if(box_center_x > img_center_x):
            direction = -1
        else:   
            direction = 1 

        self.angular_error = sqrt((img_center_x-box_center_x)**2 + (img_center_y-box_center_y)**2)
        self.angular_error = direction * self.angular_error

    
    def run(self) -> None:

        try:
            while not rospy.is_shutdown():


                # Call human detection service
                rospy.wait_for_service('detection')
                label = "person"
                response = self.detection(label)

                # When target is in sight of robot -> follow 
                if(response.in_sight_of_robot == True):

                    # Update angular velocity of robot using p-controller
                    self.find_angular_error(response)
                    self.move.angular.z = self.angular_vel_coef * self.angular_error
                    self.cmd_publisher.publish(self.move) 

                # When target is not in sight of robot -> freeze
                else:
                    self.cmd_publisher.publish(self.freeze) 

        except rospy.exceptions.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

