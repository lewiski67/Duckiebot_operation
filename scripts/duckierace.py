#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, Joy, Range
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
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
        self.lost_since = None
        self.search_delay = 0.2
        self.search_period = 0.7
        self.search_linear_speed = 0.1
        self.search_angular_speed = 0.25
        self.default_speed = 0.25
        self.default_power_level = rospy.get_param('~initial_power', 75.0)
        self.speed = self.default_speed
        self.speed_step = 0.02
        self.max_speed = 0.35
        self.min_speed = 0.15

        self.kp = 3.0
        self.kd = rospy.get_param('~kd', 0.4)
        self.last_error = 0.0
        self.last_angular_speed = 0.0
        self.lost_linear_speed = 0.10
        self.lost_angular_decay = 0.6
        self.lost_angular_limit = 0.4
        self.last_time = rospy.Time.now()

        self.default_color = 'red'
        self.alt_color = 'yellow'
        self.target_color = self.default_color
        self.red_mode_active = False
        self.red_line_last_seen = rospy.Time.now()
        self.button_x = 0
        self.button_b = 2
        self.button_y = 3
        self.button_l1 = 4
        self.button_r1 = 5

        self.h_row_ratio = 0.65
        self.stop_in_fuel = False
        
        
        self.z_min = 0
        self.z_max = 1.5
        self.tof_status = 0 # For start
        self.tof_emergency_stop_enabled = rospy.get_param('~tof_emergency_stop', False)
        self.tof_stop_distance = rospy.get_param('~tof_stop_distance', 0.20)
        self.tof_release_distance = rospy.get_param('~tof_release_distance', 0.25)
        self.safety_recovery_duration = rospy.get_param('~tof_recovery_duration', 1.0)
        self.tof_mount_offset = rospy.get_param('~tof_mount_offset', 0.0)
        self.tof_obstacle_stop = False
        self.apriltag_stop_active = False
        self.safety_recovery_start = None
        self.tof_near_count = 0
        self.tof_stop_confirm_samples = 2
        self.tof_status_sub = rospy.Subscriber("front_range_status", Int32, self.status_tof_callback, queue_size=1)

        self.tof_sub = rospy.Subscriber("front_range", Range, self.tof_callback, queue_size=1)
        self.apriltag_stop_sub = rospy.Subscriber(
            "apriltag_stop/active", Bool, self.apriltag_stop_callback, queue_size=1
        )

        self.tof_warn = False


        self.joy_sub    = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.image_sub  = rospy.Subscriber('robot_cam/image_raw', Image, self.image_callback)
        self.cmd_pub    = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.last_joy_time = 0

        self.robot_name = rospy.get_param('~robot_name', 'henry')
        self.brake_pub = rospy.Publisher(f"/{self.robot_name}/local_brake", Bool, queue_size=1)
        self.last_brake_sent = False

        self.power_level            = self.default_power_level  # start at 100%
        self.power_depletion_rate   = 3    # base depletion factor
        self.low_power_threshold    = 10.0   # percent
        
        self.in_fuel_zone           = False
        self.charging = False
        self.last_charge_time       = time.time()
        self.recharge_rate          = 5  # percent per second
        self.real_spd = 0.0  # Real speed from encoders     
        self.sub_wheel_enc = rospy.Subscriber("wheel_omega",WheelsEncoder,self.wheel_omega_cb,queue_size=1)

        power_topic = f"/{self.robot_name}/power_level"
        self.power_pub = rospy.Publisher(power_topic, Float32, queue_size=1)
        self.spd_pub = rospy.Publisher("speed_percent", Float32, queue_size=1)  
        fuel_topic = "in_fuel_zone"
        self.fuel_sub = rospy.Subscriber(fuel_topic, Bool, self.fuel_callback)
        
        self.power_pub.publish(data=self.power_level)
        self.reset_power_speed_sub = rospy.Subscriber("reset_power_speed", Bool, self.reset_power_speed_callback)
        self.fuel_timer = rospy.Timer(rospy.Duration(1.0), self.fuel_timer_callback)
        rospy.loginfo("Line follower node initialized")
    
    def status_tof_callback(self, msg: Int32):
        self.tof_status = msg.data    

    def tof_callback(self, msg: Range):
        valid_reading = (
            self.tof_status == 9
            and np.isfinite(msg.range)
            and msg.range >= msg.min_range
        )
        if valid_reading:
            corrected_distance = np.clip(msg.range - self.tof_mount_offset, self.z_min, self.z_max)
            if corrected_distance <= self.tof_stop_distance + 1e-6:
                self.tof_near_count += 1
                if self.tof_near_count >= self.tof_stop_confirm_samples:
                    self.set_tof_obstacle_stop(True, "obstacle too close")
            elif corrected_distance >= self.tof_release_distance - 1e-6:
                self.tof_near_count = 0
                self.set_tof_obstacle_stop(False, "safe distance restored")
            else:
                self.tof_near_count = 0
        else:
            self.tof_near_count = 0
            self.set_tof_obstacle_stop(False, "no obstacle detected")

    def set_tof_obstacle_stop(self, stop, reason):
        if not self.tof_emergency_stop_enabled or self.tof_obstacle_stop == stop:
            return
        self.tof_obstacle_stop = stop
        if stop:
            self.safety_recovery_start = None
            rospy.logwarn("ToF emergency stop: %s", reason)
        else:
            if self.apriltag_stop_active:
                rospy.loginfo("ToF stop released, but AprilTag stop remains active: %s", reason)
            else:
                self.safety_recovery_start = rospy.Time.now()
                rospy.loginfo("ToF emergency stop released: %s", reason)

    def apriltag_stop_callback(self, msg: Bool):
        active = bool(msg.data)
        if self.apriltag_stop_active == active:
            return

        self.apriltag_stop_active = active
        if active:
            self.safety_recovery_start = None
            rospy.logwarn("AprilTag STOP active")
        elif self.tof_emergency_stop_enabled and self.tof_obstacle_stop:
            rospy.loginfo("AprilTag STOP released, but ToF stop remains active")
        else:
            self.safety_recovery_start = rospy.Time.now()
            rospy.loginfo("AprilTag STOP released; starting speed recovery")

    def safety_stop_active(self):
        return self.apriltag_stop_active or (
            self.tof_emergency_stop_enabled and self.tof_obstacle_stop
        )

        
    def fuel_timer_callback(self, event):
        if self.charging:
            print(f"[{self.robot_name}]: Charging in fuel zone")
            self.power_level = min(100.0, self.power_level + self.recharge_rate)
        self.power_pub.publish(data=self.power_level)

    def reset_power_speed_callback(self, msg):
        if not msg.data:
            return
        self.speed = self.default_speed
        self.power_level = self.default_power_level
        self.charging = False
        self.last_time = rospy.Time.now()
        self.spd_pub.publish(Float32(data=self.speed))
        self.power_pub.publish(Float32(data=self.power_level))
        rospy.loginfo(f"Reset power and speed for {self.robot_name}")

    def wheel_omega_cb(self,msg:WheelsEncoder) -> None:
        # Update real speed based on wheel encoders
        self.real_spd = (msg.omega_left + msg.omega_right) / 2.0

    def fuel_callback(self, msg):
        self.in_fuel_zone = msg.data
        print(f"[{self.robot_name}]:Fuel zone status: {self.in_fuel_zone}")

    def joy_callback(self, msg):
        now = time.time()
        if now - self.last_joy_time < 0.1:
            return
        
        self.last_joy_time = now

        if msg.buttons[self.button_l1]:  # L1 to increase speed
            self.speed = min(self.speed + self.speed_step, self.max_speed)
            rospy.loginfo(f"Speed increased to: {self.speed:.2f}")

        if msg.buttons[self.button_r1]:  # R1 to decrease speed
            self.speed = max(self.speed - self.speed_step, self.min_speed)
            rospy.loginfo(f"Speed decreased to: {self.speed:.2f}")

        self.spd_pub.publish(Float32(data=(self.speed)))
        # Manual brake release (Logitech Cordless RumblePad 2 mapping)
        if msg.buttons[self.button_x] and not self.last_brake_sent:
            if not self.in_fuel_zone:
                # you may unlock brake outside fuel zone without problem
                rospy.loginfo(f"Sending manual brake unlock to /{self.robot_name}/local_brake")
                self.brake_pub.publish(Bool(data=False))
                self.last_brake_sent = True
            else:
                # you must holding B button to allow brake release in fuel zone, otherwise it will keep brake lock to stop you
                if msg.buttons[self.button_b]: # B button for fuel zone brake release
                    rospy.loginfo(f"Sending manual brake unlock to /{self.robot_name}/local_brake in fuel zone")
                    self.brake_pub.publish(Bool(data=False))
                    self.last_brake_sent = True
        elif  msg.buttons[self.button_x] and self.last_brake_sent:
            rospy.loginfo(f"Sending manual brake lock to /{self.robot_name}/local_brake")
            self.brake_pub.publish(Bool(data=True))
            self.last_brake_sent = False 
            
        # elif not msg.buttons[self.button_x] and self.last_brake_sent and self.in_fuel_zone: # when release brake button in fuel zone, send brake lock command to ensure stop
        #     rospy.loginfo(f"Sending manual brake lock to /{self.robot_name}/local_brake")
        #     self.brake_pub.publish(Bool(data=True))
        #     self.last_brake_sent = False

        if msg.buttons[self.button_y] and self.in_fuel_zone: # Y
            self.charging = True
        else:
            self.charging = False

        # Button B: prefer yellow lines, naming has legacy problem, keep for now
        self.red_mode_active = bool(msg.buttons[self.button_b]) 
        # if not self.in_fuel_zone:
        #     # red mode here actually means alternative color, as in yellow in the setup
        #     # this command allows changing to yellow line when not in fuel zone, and will automatically come back to red if release buttons
        #     self.red_mode_active = bool(msg.buttons[1]) 
        #     self.stop_in_fuel = False  # Reset stop flag when toggling color mode outside fuel zone
        # elif not bool(msg.buttons[1]): # when let go button B in fuel zone, force stop to prevent going anywhere
        #     self.stop_in_fuel = True
        #     print(f"[{self.robot_name}]: Stopping in fuel zone due to button release")
        # else:
        #     print(f"[{self.robot_name}]: Button B pressed in fuel zone, but stop_in_fuel is {self.stop_in_fuel}")
        #     self.stop_in_fuel = False


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
            alt_mask = self.detector.get_mask(bgr, self.alt_color)
            tracking_y = int(alt_mask.shape[0] * self.h_row_ratio)
            if np.count_nonzero(alt_mask[tracking_y, :]) >= 8:
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
        if len(indices) > 0:
            segments = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
            valid_segments = [segment for segment in segments if len(segment) >= 8]
            indices = (
                np.concatenate(valid_segments)
                if valid_segments
                else np.array([], dtype=indices.dtype)
            )

        if len(indices) == 0:
            if not self.lost_detect:
                rospy.logwarn("No line detected!")
            self.lost_detect = True
            if self.in_fuel_zone:
                self.publish_twist(0.0, 0.0)  # Stop in fuel zone if line is lost
            else:
                self.last_angular_speed *= self.lost_angular_decay
                self.publish_twist(self.speed, self.last_angular_speed)
            return
        self.lost_detect = False
        self.lost_since = None
        center_x = w // 2
        avg_x = np.mean(indices)
        error = (avg_x - center_x) / center_x

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        derror = (error - self.last_error) / dt if dt > 0 else 0.0

        angular_speed = -self.kp * error - self.kd * derror
        angular_speed = np.clip(angular_speed, -1.5, 1.5)

        self.last_error = error
        self.last_angular_speed = angular_speed
        self.last_time = now
        # If power is too low, force minimum speed
        if self.power_level < self.low_power_threshold:
            self.speed = self.min_speed
            self.spd_pub.publish(Float32(data=self.speed))
        self.publish_twist(self.speed, angular_speed)

        # Deplete virtual power: faster speed = faster depletion
        if self.real_spd > 0.1:
            depletion = self.power_depletion_rate * self.speed * dt
            self.power_level = max(0.0, self.power_level - depletion)
            # self.power_pub.publish(data=self.power_level)

    def publish_twist(self, linear, angular):
        if self.safety_stop_active():
            linear = 0.0
            angular = 0.0
        elif self.safety_recovery_start is not None:
            elapsed = (rospy.Time.now() - self.safety_recovery_start).to_sec()
            if self.safety_recovery_duration <= 0.0 or elapsed >= self.safety_recovery_duration:
                self.safety_recovery_start = None
            else:
                recovery_scale = max(0.0, elapsed / self.safety_recovery_duration)
                linear *= recovery_scale
                angular *= recovery_scale
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LineFollower()
    node.run()
