#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class ManualJoyCommand:
    def __init__(self):
        rospy.init_node("manual_joy_command")
        self.linear_axis = int(rospy.get_param("~linear_axis", 1))
        self.angular_axis = int(rospy.get_param("~angular_axis", 3))
        self.linear_scale = float(rospy.get_param("~linear_scale", 0.25))
        self.reverse_scale = float(rospy.get_param("~reverse_scale", 0.6))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.5))
        self.deadzone = float(rospy.get_param("~deadzone", 0.08))

        self.cmd_pub = rospy.Publisher("cmd_vel_manual", Twist, queue_size=1)
        rospy.Subscriber("joy_raw", Joy, self.joy_callback, queue_size=1)

    def apply_deadzone(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def joy_callback(self, msg):
        required_axes = max(self.linear_axis, self.angular_axis) + 1
        if len(msg.axes) < required_axes:
            rospy.logwarn_throttle(
                2.0,
                "Joystick has %d axes; at least %d are required",
                len(msg.axes),
                required_axes,
            )
            self.cmd_pub.publish(Twist())
            return

        linear_axis = self.apply_deadzone(msg.axes[self.linear_axis])
        angular_axis = self.apply_deadzone(msg.axes[self.angular_axis])

        linear = linear_axis * self.linear_scale
        if linear < 0.0:
            linear *= self.reverse_scale

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular_axis * self.angular_scale
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        ManualJoyCommand()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
