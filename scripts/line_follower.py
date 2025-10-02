#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class RobustLineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('/turtle_camera/image_raw', Image, self.camera_callback)
        self.bridge = CvBridge()
        self.state = "FOLLOWING"
        self.consecutive_high_errors = 0
        self.search_start_time = None
        self.turn_start_time = None
        self.last_error = 0
        
        self.error_threshold = 15
        self.required_consecutive = 3
        self.max_search_time = 3.0
        self.normal_linear_speed = 0.3
        self.search_linear_speed = 0.1
        self.turn_duration = 1.2
        self.turn_angular_speed = 0.6
        self.Kp = 0.003
        self.Kd = 0.001
        self.image_center_x = 320
        self.rate = rospy.Rate(20)

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            error, processed_image = self.process_image(cv_image)
            self.process_state_machine(error)
            cv2.imshow("Line Detection", processed_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Camera callback error: {e}")

    def process_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)
        h, w = binary.shape
        roi = binary[int(h*0.5):h, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        error = 0
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                centroid_x = int(M['m10'] / M['m00'])
                error = centroid_x - self.image_center_x
                cv2.circle(roi, (centroid_x, int(M['m01'] / M['m00'])), 5, (255, 255, 255), -1)
        
        vis_image = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        cv2.putText(vis_image, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Error: {error}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Consecutive: {self.consecutive_high_errors}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return error, vis_image

    def process_state_machine(self, error):
        if self.state == "FOLLOWING":
            self.handle_following_state(error)
        elif self.state == "JUNCTION_DETECTED":
            self.handle_junction_detected()
        elif self.state == "TURNING":
            self.handle_turning_state()
        elif self.state == "SEARCHING":
            self.handle_searching_state(error)
        elif self.state == "FOLLOWING_NEW_LINE":
            self.handle_new_line_following(error)

    def handle_following_state(self, error):
        if abs(error) > self.error_threshold:
            self.consecutive_high_errors += 1
            rospy.loginfo(f"High error detected: {error}, consecutive: {self.consecutive_high_errors}")
        else:
            self.consecutive_high_errors = 0
        
        if self.consecutive_high_errors >= self.required_consecutive:
            rospy.loginfo("JUNCTION DETECTED! Initiating left turn sequence")
            self.state = "JUNCTION_DETECTED"
            return
        
        derivative = error - self.last_error
        angular_vel = -(self.Kp * error + self.Kd * derivative)
        angular_vel = max(-1.0, min(1.0, angular_vel))
        self.publish_velocity(self.normal_linear_speed, angular_vel)
        self.last_error = error

    def handle_junction_detected(self):
        rospy.loginfo("Junction confirmed. Stopping before turn...")
        self.publish_velocity(0.0, 0.0)
        rospy.sleep(0.5)
        self.state = "TURNING"
        self.turn_start_time = rospy.Time.now()
        rospy.loginfo("Starting LEFT TURN...")

    def handle_turning_state(self):
        elapsed = (rospy.Time.now() - self.turn_start_time).to_sec()
        if elapsed < self.turn_duration:
            self.publish_velocity(0.0, self.turn_angular_speed)
            rospy.loginfo(f"Turning left... {elapsed:.1f}/{self.turn_duration:.1f}s")
        else:
            self.publish_velocity(0.0, 0.0)
            rospy.sleep(0.3)
            self.state = "SEARCHING"
            self.search_start_time = rospy.Time.now()
            rospy.loginfo("Turn completed. Searching for left branch line...")

    def handle_searching_state(self, error):
        search_elapsed = (rospy.Time.now() - self.search_start_time).to_sec()
        if abs(error) < 10 and error != 0:
            rospy.loginfo(f"Found left branch line! Error: {error}")
            self.state = "FOLLOWING_NEW_LINE"
            self.consecutive_high_errors = 0
            return
        if search_elapsed > self.max_search_time:
            rospy.logwarn("Search timeout. Returning to normal following mode.")
            self.state = "FOLLOWING"
            self.consecutive_high_errors = 0
            return
        self.publish_velocity(self.search_linear_speed, 0.0)

    def handle_new_line_following(self, error):
        derivative = error - self.last_error
        angular_vel = -(self.Kp * error + self.Kd * derivative)
        angular_vel = max(-1.0, min(1.0, angular_vel))
        self.publish_velocity(self.normal_linear_speed, angular_vel)
        self.last_error = error

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.loginfo("Line follower started. Press Ctrl+C to stop.")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down line follower...")
            self.publish_velocity(0.0, 0.0)
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        follower = RobustLineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

