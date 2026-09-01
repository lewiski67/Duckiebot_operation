#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int32, String


class ControlAuthorityMux:
    AUTO = "auto"
    JOYSTICK = "joystick"

    def __init__(self):
        rospy.init_node("control_authority_mux")

        self.mode = self.AUTO
        self.auto_cmd = Twist()
        self.manual_cmd = Twist()
        self.last_auto_cmd = None
        self.last_manual_cmd = None

        self.auto_timeout = float(rospy.get_param("~auto_timeout", 0.5))
        self.manual_timeout = float(rospy.get_param("~manual_timeout", 0.3))
        self.switch_hold = float(rospy.get_param("~switch_hold", 0.25))
        self.center_linear = float(rospy.get_param("~center_linear", 0.02))
        self.center_angular = float(rospy.get_param("~center_angular", 0.05))
        self.recovery_duration = float(rospy.get_param("~recovery_duration", 1.0))

        self.tof_enabled = bool(rospy.get_param("~tof_emergency_stop", False))
        self.tof_stop_distance = float(rospy.get_param("~tof_stop_distance", 0.20))
        self.tof_release_distance = float(rospy.get_param("~tof_release_distance", 0.25))
        self.tof_mount_offset = float(rospy.get_param("~tof_mount_offset", 0.04))
        self.tof_status = 0
        self.tof_stop = False
        self.tof_near_count = 0
        self.tof_confirm_samples = 2
        self.tag_stop = False

        self.global_brake = True
        self.local_brake = True
        self.switch_hold_until = rospy.Time.now()
        self.recovery_start = None
        self.last_status = None
        self.shutting_down = False

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.mode_pub = rospy.Publisher("control_mode", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("control_mode_status", String, queue_size=1, latch=True)
        self.joystick_ready_pub = rospy.Publisher("joystick_ready", Bool, queue_size=1, latch=True)

        rospy.Subscriber("cmd_vel_auto", Twist, self.auto_callback, queue_size=1)
        rospy.Subscriber("cmd_vel_manual", Twist, self.manual_callback, queue_size=1)
        rospy.Subscriber("set_control_mode", String, self.mode_callback, queue_size=1)
        rospy.Subscriber("apriltag_stop/active", Bool, self.tag_callback, queue_size=1)
        rospy.Subscriber("front_range_status", Int32, self.tof_status_callback, queue_size=1)
        rospy.Subscriber("front_range", Range, self.tof_callback, queue_size=1)
        rospy.Subscriber("/global_brake", Bool, self.global_brake_callback, queue_size=1)
        rospy.Subscriber("local_brake", Bool, self.local_brake_callback, queue_size=1)

        self.mode_pub.publish(String(data=self.mode))
        self.set_status("auto: waiting for command")
        rospy.on_shutdown(self.shutdown)
        self.control_timer_handle = rospy.Timer(rospy.Duration(0.02), self.control_timer)

    @staticmethod
    def zero_cmd():
        return Twist()

    @staticmethod
    def copy_cmd(source):
        result = Twist()
        result.linear.x = source.linear.x
        result.linear.y = source.linear.y
        result.linear.z = source.linear.z
        result.angular.x = source.angular.x
        result.angular.y = source.angular.y
        result.angular.z = source.angular.z
        return result

    def set_status(self, status):
        if status == self.last_status:
            return
        self.last_status = status
        self.status_pub.publish(String(data=status))
        rospy.loginfo("Control authority: %s", status)

    def shutdown(self):
        self.shutting_down = True
        if hasattr(self, "control_timer_handle"):
            self.control_timer_handle.shutdown()

    def publish_cmd(self, cmd):
        if self.shutting_down:
            return
        try:
            self.cmd_pub.publish(cmd)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def auto_callback(self, msg):
        self.auto_cmd = self.copy_cmd(msg)
        self.last_auto_cmd = rospy.Time.now()

    def manual_callback(self, msg):
        self.manual_cmd = self.copy_cmd(msg)
        self.last_manual_cmd = rospy.Time.now()

    def command_fresh(self, stamp, timeout, now=None):
        if stamp is None:
            return False
        now = now or rospy.Time.now()
        return (now - stamp).to_sec() <= timeout

    def manual_centered(self):
        return (
            abs(self.manual_cmd.linear.x) <= self.center_linear
            and abs(self.manual_cmd.angular.z) <= self.center_angular
        )

    def mode_callback(self, msg):
        requested = msg.data.strip().lower()
        if requested not in (self.AUTO, self.JOYSTICK):
            self.set_status("rejected: mode must be auto or joystick")
            return
        if requested == self.mode:
            self.set_status("%s: already active" % self.mode)
            return
        if requested == self.JOYSTICK:
            if not self.command_fresh(self.last_manual_cmd, self.manual_timeout):
                self.set_status("rejected: joystick is not publishing")
                return
            if not self.manual_centered():
                self.set_status("rejected: center joystick before switching")
                return

        self.mode = requested
        self.switch_hold_until = rospy.Time.now() + rospy.Duration(self.switch_hold)
        self.mode_pub.publish(String(data=self.mode))
        self.set_status("%s: switching" % self.mode)

    def global_brake_callback(self, msg):
        self.global_brake = bool(msg.data)

    def local_brake_callback(self, msg):
        self.local_brake = bool(msg.data)

    def tag_callback(self, msg):
        active = bool(msg.data)
        if active == self.tag_stop:
            return
        self.tag_stop = active
        if active:
            self.recovery_start = None
            rospy.logwarn("AprilTag STOP active in control mux")
        elif not self.tof_stop:
            self.recovery_start = rospy.Time.now()
            rospy.loginfo("AprilTag STOP released in control mux")

    def tof_status_callback(self, msg):
        self.tof_status = msg.data

    def tof_callback(self, msg):
        valid = (
            self.tof_status == 9
            and math.isfinite(msg.range)
            and msg.range >= msg.min_range
        )
        if valid:
            corrected = max(0.0, msg.range - self.tof_mount_offset)
            if corrected <= self.tof_stop_distance + 1e-6:
                self.tof_near_count += 1
                if self.tof_near_count >= self.tof_confirm_samples:
                    self.set_tof_stop(True, "obstacle too close")
            elif corrected >= self.tof_release_distance - 1e-6:
                self.tof_near_count = 0
                self.set_tof_stop(False, "safe distance restored")
            else:
                self.tof_near_count = 0
        else:
            self.tof_near_count = 0
            self.set_tof_stop(False, "no obstacle detected")

    def set_tof_stop(self, active, reason):
        if not self.tof_enabled or active == self.tof_stop:
            return
        self.tof_stop = active
        if active:
            self.recovery_start = None
            rospy.logwarn("ToF emergency stop in control mux: %s", reason)
        elif not self.tag_stop:
            self.recovery_start = rospy.Time.now()
            rospy.loginfo("ToF emergency stop released in control mux: %s", reason)

    def safety_stop_active(self):
        return self.tag_stop or (self.tof_enabled and self.tof_stop)

    def apply_recovery(self, cmd, now):
        if self.recovery_start is None:
            return cmd
        elapsed = (now - self.recovery_start).to_sec()
        if self.recovery_duration <= 0.0 or elapsed >= self.recovery_duration:
            self.recovery_start = None
            return cmd
        scale = max(0.0, elapsed / self.recovery_duration)
        cmd.linear.x *= scale
        cmd.angular.z *= scale
        return cmd

    def control_timer(self, _event):
        if self.shutting_down:
            return
        now = rospy.Time.now()
        manual_fresh = self.command_fresh(self.last_manual_cmd, self.manual_timeout, now)
        self.joystick_ready_pub.publish(Bool(data=manual_fresh))

        if self.global_brake:
            self.publish_cmd(self.zero_cmd())
            self.set_status("braked: global")
            return
        if self.local_brake:
            self.publish_cmd(self.zero_cmd())
            self.set_status("braked: local")
            return
        if self.safety_stop_active():
            self.publish_cmd(self.zero_cmd())
            self.set_status("braked: safety sensor")
            return
        if now < self.switch_hold_until:
            self.publish_cmd(self.zero_cmd())
            self.set_status("%s: switching" % self.mode)
            return

        if self.mode == self.AUTO:
            if not self.command_fresh(self.last_auto_cmd, self.auto_timeout, now):
                self.publish_cmd(self.zero_cmd())
                self.set_status("auto: command timeout")
                return
            cmd = self.copy_cmd(self.auto_cmd)
            self.set_status("auto: active")
        else:
            if not manual_fresh:
                self.publish_cmd(self.zero_cmd())
                self.set_status("joystick: command timeout")
                return
            cmd = self.copy_cmd(self.manual_cmd)
            self.set_status("joystick: active")

        self.publish_cmd(self.apply_recovery(cmd, now))


if __name__ == "__main__":
    try:
        ControlAuthorityMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
