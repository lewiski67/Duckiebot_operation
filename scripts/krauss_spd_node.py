#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Krauss-style speed setpoint controller (ROS1, Python3)
- Inputs:
    * perception/lead_car_distance : std_msgs/Float32 (meters). d > z_max => no leader (free flow).
    * wheel omega                 :encoder-estimated ego speed.
- Outputs:
    * cmd_vel_acc         : geometry_msgs/Twist (linear.x = speed setpoint in m/s, angular.z = 0)
    * v_setpoint         : std_msgs/Float32 (m/s), same as cmd_vel.linear.x for easy logging.
- Behavior:
    * Free-flow when no leader: ramp toward v_max with acceleration cap a.
    * When leader valid: estimate leader speed from gap derivative and compute Krauss safe speed.
    * Apply acceleration cap and an optional comfortable deceleration cap (b) to avoid harsh drops.
    * No extra artificial noise: physical system noise is sufficient.
"""

import math
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from vpa_robot_interface.msg import WheelEncoder

class KraussSpeedController(object):
    def __init__(self):
        # --- Parameters ---
        self.rate_hz    = rospy.get_param("~rate_hz", 20.0)     # control rate (Hz)
        self.v_max      = rospy.get_param("~v_max", 0.3)        # free-flow speed (m/s)
        self.accel_a    = rospy.get_param("~a", 0.6)            # accel cap (m/s^2)
        self.decel_b    = rospy.get_param("~b", 0.5)            # comfortable decel bound (m/s^2)
        self.min_gap    = rospy.get_param("~min_gap", 0.10)     # bumper clearance (m)
        self.stop_gap   = rospy.get_param("~z_stop", 0.10)      # hard stop threshold (m)
        # must match ACCLeadNode so sentinel semantics align:
        self.z_min      = rospy.get_param("~z_min", 0.04)       # min valid distance (m)
        self.z_max      = rospy.get_param("~z_max", 1.2)        # max valid distance (m)
        self.vlead_alpha= rospy.get_param("~vlead_alpha", 0.25) # LPF for inferred leader speed (0..1)

        # --- State ---
        self.v_meas   = 0.0
        self.lead_d   = None
        self.lead_valid = False
        self.prev_lead_valid = False
        self.g_prev   = None
        self.v_lead   = 0.0
        self.v_last   = 0.0
        self.t_prev   = None

        self.radius = 0.0318

        # --- ROS I/O ---
        rospy.Subscriber("wheel_omega", WheelEncoder, self.cb_speed, queue_size=10)
        rospy.Subscriber("perception/lead_car_distance", Float32, self.cb_lead_dist, queue_size=10)
        self.pub_twist = rospy.Publisher("cmd_vel_acc", Twist, queue_size=10)
        self.pub_vset  = rospy.Publisher("v_setpoint", Float32, queue_size=10)

        rospy.loginfo("[krauss_speed_controller] Initialized with v_max=%.3f a=%.3f b=%.3f z_min=%.2f z_max=%.2f",
                      self.v_max, self.accel_a, self.decel_b, self.z_min, self.z_max)

    # --- Callbacks ---
    def cb_speed(self, msg):
        self.v_meas = (msg.left_wheel_omega + msg.right_wheel_omega) * 0.5 * self.radius

    def cb_lead_dist(self, msg):
        self.lead_d = float(msg.data)
        self.prev_lead_valid, self.lead_valid = self.lead_valid, (self.lead_d <= self.z_max)

    # --- Helpers ---
    def _publish_speed(self, v):
        tw = Twist()
        tw.linear.x = max(0.0, float(v))
        tw.angular.z = 0.0
        self.pub_twist.publish(tw)
        self.pub_vset.publish(Float32(data=tw.linear.x))

    # --- Main step ---
    def step(self):
        now = time.time()
        if self.t_prev is None:
            self.t_prev = now
            # make first intent explicit
            self._publish_speed(0.0)
            return
        dt = now - self.t_prev
        # clamp dt to avoid huge steps after hiccups
        dt = max(0.01, min(0.2, dt))
        self.t_prev = now

        # Require at least one distance read; otherwise decay toward 0 gently
        if self.lead_d is None:
            v_next = max(0.0, self.v_last - self.decel_b * dt)
            self._publish_speed(v_next)
            self.v_last = v_next
            return

        # Case A: NO LEADER (free flow). Sentinel: d > z_max
        if not self.lead_valid:
            # Free-accel toward v_max with cap a
            v_acc_cap = self.v_meas + self.accel_a * dt
            # Optional comfortable decel cap to avoid sudden drops after sensor flips
            v_dec_cap = self.v_meas - self.decel_b * dt
            v_next = min(self.v_max, v_acc_cap)
            v_next = max(v_dec_cap, v_next)  # apply decel comfort
            v_next = max(0.0, v_next)
            self._publish_speed(v_next)
            self.v_last = v_next
            # Reset derivative state to avoid spikes when a leader reappears
            self.g_prev = None
            return

        # Case B: LEADER VALID
        # Compute gap to bumper
        g = max(0.0, self.lead_d - self.min_gap)

        # When validity just turned true OR g_prev missing, reseed and do a conservative step
        if (self.g_prev is None) or (self.prev_lead_valid != self.lead_valid):
            self.g_prev = g
            # Smoothly limit to current acceleration; don't jump down faster than b
            v_acc_cap = self.v_meas + self.accel_a * dt
            v_dec_cap = self.v_meas - self.decel_b * dt
            v_next = min(self.v_max, v_acc_cap)
            v_next = max(v_dec_cap, v_next)
            v_next = max(0.0, v_next)
            self._publish_speed(v_next)
            self.v_last = v_next
            return

        # Derivative and inferred leader speed
        g_dot = (g - self.g_prev) / dt
        self.g_prev = g
        v_lead_raw = g_dot + self.v_meas
        # clamp leader speed to plausible bounds
        v_lead_raw = max(0.0, min(self.v_max, v_lead_raw))
        # low-pass filter
        self.v_lead = self.vlead_alpha * v_lead_raw + (1.0 - self.vlead_alpha) * self.v_lead

        # Krauss safe speed
        term = g - self.v_meas * dt
        R = self.v_lead ** 2 + 4.0 * self.decel_b * term
        if R < 0.0:
            R = 0.0
        denom = self.v_lead + math.sqrt(R)
        denom = max(1e-6, denom)  # guard small denom
        v_safe = self.v_lead + (2.0 * self.decel_b * term) / denom
        v_safe = max(0.0, v_safe)

        # Candidate next speed with accel cap and comfortable decel cap
        v_acc_cap = self.v_meas + self.accel_a * dt
        v_dec_cap = self.v_meas - self.decel_b * dt
        v_cand = min(self.v_max, v_acc_cap, v_safe)
        v_next = max(v_dec_cap, v_cand)   # avoid braking harder than b
        # Hard stop if extremely close
        if self.lead_d < self.stop_gap:
            v_next = 0.0

        v_next = max(0.0, v_next)
        self._publish_speed(v_next)
        self.v_last = v_next

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("krauss_speed_controller")
    node = KraussSpeedController()
    node.run()
