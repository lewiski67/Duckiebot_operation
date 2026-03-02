#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, Joy, Range
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from color_detector import ColorDetector
from vpa_robot_interface.msg import WheelsEncoder

class LineFollower:
    def __init__(self):
        rospy.init_node('duckierace_line_follower_node')
        self.bridge     = CvBridge()
        self.detector   = ColorDetector()
        self.lost_detect = False
        self.speed = 0.25
        self.speed_step = 0.02
        self.max_speed = 0.35
        self.min_speed = 0.2

        self.kp = 5
        self.kd = rospy.get_param('~kd', 2.5)
        self.last_error = 0.0
        self.last_time = rospy.Time.now()

        self.default_color = 'red'
        self.alt_color = 'yellow'
        self.target_color = self.default_color
        self.red_mode_active = False
        self.red_line_last_seen = rospy.Time.now()

        self.h_row_ratio = 0.6

        self.joy_sub    = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.image_sub  = rospy.Subscriber('robot_cam/image_raw', Image, self.image_callback)
        self.cmd_pub    = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.last_joy_time = 0

        self.robot_name = rospy.get_param('~robot_name', 'henry')
        self.brake_pub = rospy.Publisher(f"/{self.robot_name}/local_brake", Bool, queue_size=1)
        self.last_brake_sent = False

        self.power_level            = 75.0  # start at 100%
        self.power_depletion_rate   = 3    # base depletion factor
        self.low_power_threshold    = 10.0   # percent

        self.in_fuel_zone           = False
        self.charging = False
        self.last_charge_time       = time.time()
        self.recharge_rate          = 2  # percent per second
        self.real_spd = 0.0  # Real speed from encoders     
        self.sub_wheel_enc = rospy.Subscriber("wheel_omega",WheelsEncoder,self.wheel_omega_cb,queue_size=1)

        power_topic = f"/{self.robot_name}/power_level"
        self.power_pub = rospy.Publisher(power_topic, Float32, queue_size=1)
        self.spd_pub = rospy.Publisher("speed_percent", Float32, queue_size=1)  
        fuel_topic = "in_fuel_zone"
        self.fuel_sub = rospy.Subscriber(fuel_topic, Bool, self.fuel_callback)
        self.stop_in_fuel = False
        self.power_pub.publish(data=self.power_level)
        self.fuel_timer = rospy.Timer(rospy.Duration(1.0), self.fuel_timer_callback)
        rospy.loginfo("Line follower node initialized")

    def fuel_timer_callback(self, event):
        if self.charging:
            print(f"[{self.robot_name}]: Charging in fuel zone")
            self.power_level = min(100.0, self.power_level + self.recharge_rate)
        self.power_pub.publish(data=self.power_level)

    def wheel_omega_cb(self,msg:WheelsEncoder) -> None:
        # Update real speed based on wheel encoders
        self.real_spd = (msg.omega_left + msg.omega_right) / 2.0

    def fuel_callback(self, msg):
        self.in_fuel_zone = msg.data
        print(f"[{self.robot_name}]:Fuel zone status: {self.in_fuel_zone}")

    def joy_callback(self, msg):
        now = time.time()
        if now - self.last_joy_time < 0.3:
            return

        if msg.buttons[4]:  # L1 to increase speed
            self.speed = min(self.speed + self.speed_step, self.max_speed)
            rospy.loginfo(f"Speed increased to: {self.speed:.2f}")
            self.last_joy_time = now
            self.power_depletion_rate += 0.5

        if msg.buttons[5]:  # R1 to decrease speed
            self.speed = max(self.speed - self.speed_step, self.min_speed)
            rospy.loginfo(f"Speed decreased to: {self.speed:.2f}")
            self.last_joy_time = now
            self.power_depletion_rate -= 0.5

        self.spd_pub.publish(Float32(data=(self.speed)))
        # Manual brake release (X = button[2]) yes is x
        if msg.buttons[2] and not self.last_brake_sent:
            rospy.loginfo(f"Sending manual brake unlock to /{self.robot_name}/local_brake")
            self.brake_pub.publish(Bool(data=False))
            self.last_brake_sent = True
            
        # elif not msg.buttons[2] and self.last_brake_sent:
        #     rospy.loginfo(f"Sending manual brake lock to /{self.robot_name}/local_brake")
        #     self.brake_pub.publish(Bool(data=False))
        #     self.last_brake_sent = True

        if msg.buttons[3] and self.in_fuel_zone: # Y
            self.charging = True
        else:
            self.charging = False

        # Button B (index 1): prefer yellow lines, naming has legacy problem, keep for now
        if not self.in_fuel_zone:
            # red mode here actually means alternative color, as in yellow in the setup
            # this command allows changing to yellow line when not in fuel zone, and will automatically come back to red if release buttons
            self.red_mode_active = bool(msg.buttons[1]) 
            self.stop_in_fuel = False  # Reset stop flag when toggling color mode outside fuel zone
        elif not bool(msg.buttons[1]): # when let go button B in fuel zone, force stop to prevent going anywhere
            self.stop_in_fuel = True
        else:
            self.stop_in_fuel = False


    def image_callback(self, msg):
        if self.stop_in_fuel:
            self.publish_twist(0.0, 0.0)  # Ensure we stay stopped in fuel zone
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")
            return

        # Update target color logic
        if self.red_mode_active:
            red_mask = self.detector.get_mask(bgr, self.alt_color)
            if np.sum(red_mask) > 2000:
                self.target_color = self.alt_color
                self.red_line_last_seen = rospy.Time.now()
            else:
                time_since_red = (rospy.Time.now() - self.red_line_last_seen).to_sec()
                if time_since_red > 1.0:
                    self.target_color = self.default_color
        else:
            self.target_color = self.default_color

        # Detect line of target color
        mask = self.detector.get_mask(bgr, self.target_color)
        h, w = mask.shape
        y = int(h * self.h_row_ratio)
        line_row = mask[y, :]

        indices = np.where(line_row > 0)[0]
        if len(indices) == 0:
            if not self.lost_detect:
                rospy.logwarn("No line detected!")
            self.lost_detect = True
            if self.in_fuel_zone:
                self.publish_twist(0.0, 0.0)  # Stop in fuel zone if line is lost

            return
        self.lost_detect = False
        avg_x = int(np.mean(indices))
        center_x = w // 2
        error = (avg_x - center_x) / center_x

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        derror = (error - self.last_error) / dt if dt > 0 else 0.0

        angular_speed = -self.kp * error - self.kd * derror
        angular_speed = np.clip(angular_speed, -2, 2)

        self.last_error = error
        self.last_time = now
        # If power is too low, force minimum speed
        if self.power_level < self.low_power_threshold:
            self.speed = self.min_speed
        self.publish_twist(self.speed, angular_speed)

        # Deplete virtual power: faster speed = faster depletion
        if self.real_spd > 0.1:
            depletion = self.power_depletion_rate * self.speed * dt
            self.power_level = max(0.0, self.power_level - depletion)
            # self.power_pub.publish(data=self.power_level)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LineFollower()
    node.run()
