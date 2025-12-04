#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Krauss-style speed setpoint controller (ROS1, Python3)
- Inputs:
    * perception/lead_car_distance : std_msgs/Float32 (meters). d > z_max => no leader (free flow).
    * wheel omega                  : encoder-estimated ego speed.
- Outputs:
    * cmd_vel_acc   : geometry_msgs/Twist (linear.x = speed setpoint in m/s, angular.z = 0)
    * v_setpoint    : std_msgs/Float32 (m/s), same as cmd_vel.linear.x for easy logging.
"""

import math
import time
import rospy

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from vpa_robot_interface.msg import WheelsEncoder


class KraussSpeedController(object):
    def __init__(self):
        # --- Parameters ---
        self.rate_hz    = rospy.get_param("~rate_hz", 20.0)     # control rate (Hz)
        self.v_max      = rospy.get_param("~v_max", 0.3)        # free-flow speed (m/s)
        self.accel_a    = rospy.get_param("~a", 0.09)           # accel cap (m/s^2)
        self.decel_b    = rospy.get_param("~b", 0.16)           # comfortable decel bound (m/s^2)
        self.min_gap    = rospy.get_param("~min_gap", 0.1)      # bumper clearance (m)
        self.stop_gap   = rospy.get_param("~z_stop", 0.1)       # hard stop threshold (m)

        # must match ACCLeadNode so sentinel semantics align:
        self.z_min      = rospy.get_param("~z_min", 0.1)        # min valid distance (m)
        self.z_max      = rospy.get_param("~z_max", 1.0)        # max valid distance (m)
        self.vlead_alpha= rospy.get_param("~vlead_alpha", 0.25) # LPF for inferred leader speed (0..1)

        self.b_model    = rospy.get_param("~b_model", 0.16)     # model decel for Krauss calc (m/s^2)
        self.tau        = rospy.get_param("~tau", 1.5)          # reaction time for Krauss calc (s)

        # Hybrid accel / startup behavior
        self.v_switch   = rospy.get_param("~v_switch", 0.05)    # (kept for compat, not used now)
        self.alpha_up   = rospy.get_param("~alpha_up", 0.5)     # (kept for compat, not used now)
        self.v_eps_start= rospy.get_param("~v_eps_start", 0.01) # below this, consider "stopped" (m/s)
        self.v_min_start= rospy.get_param("~v_min_start", 0.1) # min command to break static friction (m/s)
        self.a_start    = rospy.get_param("~a_start", 0.13)     # accel cap when starting from rest (m/s^2)

        # Stage / ladder controller: v_stage smoothly follows v_cmd_model
        self.v_stage    = 0.0
        self.stage_tol  = rospy.get_param("~stage_tol", 0.01)   # [m/s] tolerance for snapping to model
        self.a_stage    = rospy.get_param("~a_stage", 0.10)     # [m/s^2] macro accel limit for v_stage

        # --- State ---
        self.v_meas   = 0.0
        self.lead_d   = None
        self.g_prev   = None
        self.v_lead   = 0.0
        self.v_last   = 0.0
        self.t_prev   = None

        self.radius = 0.0318
        self.wheel_base = 0.1
        self.car_length = 0.18  # car length from head to tail, used to calculate min gap
        self.steer_thresh = 0.4
        self.turn_counter = 0
        self.allow_catch_up = False

        self.artificial_delay = rospy.get_param("~artificial_delay", 2.5)  # seconds

        self.is_turning = False

        self.local_brake = True
        self.stopped_by_front_car = False
        self.allow_catch_up = rospy.get_param("~allow_catch_up", False)

        self.desired_gap_headway = rospy.get_param("~desired_gap", 0.5)
        # desired gap for formation control (m), this is headway distance, not bumper to bumper distance
        self.desired_gap = self.desired_gap_headway - self.car_length  # convert to bumper to bumper distance

        # Debug timing
        self.last_debug_time = None

        self.true_stop_gap = None

        rospy.Subscriber('freeflowspd', Float32, self.freeflowspd_callback)

        # --- ROS I/O ---
        rospy.Subscriber("wheel_omega", WheelsEncoder, self.cb_speed, queue_size=1)
        rospy.Subscriber("perception/lead_car_distance", Float32, self.cb_lead_dist, queue_size=1)
        self.pub_twist = rospy.Publisher("cmd_vel_acc", Twist, queue_size=1)
        self.pub_vset  = rospy.Publisher("v_setpoint", Float32, queue_size=1)
        rospy.Subscriber("local_brake", Bool, self.local_brake_callback, queue_size=1)

        rospy.loginfo("[krauss_speed_controller] Initialized with v_max=%.3f a=%.3f b=%.3f tau=%.2f z_min=%.2f z_max=%.2f",
                      self.v_max, self.accel_a, self.b_model, self.tau, self.z_min, self.z_max)

    # --- Callbacks ---

    def local_brake_callback(self, msg):
        self.local_brake = bool(msg.data)
        if self.local_brake:
            # full drop-in when brake is pressed
            self.v_last = 0.0
            self.v_stage = 0.0
            self._publish_speed(0.0)

    def cb_speed(self, msg):
        self.v_meas = (msg.omega_left + msg.omega_right) * 0.5 * self.radius
        self.w_meas = (msg.omega_right - msg.omega_left) * self.radius / self.wheel_base  # turning detection

        if abs(self.w_meas) > self.steer_thresh:  # rad/s threshold for turning
            self.is_turning = True
        else:
            self.is_turning = False

        # Debug print for v_meas, with relative time since last zero speed
        # if self.v_meas > 0:
        #     if not hasattr(self, 'last_zero_time'):
        #         self.last_zero_time = time.time()
        #     relative_time = time.time() - self.last_zero_time
        #     rospy.loginfo(f"[DEBUG v_meas] t_rel={relative_time:.2f}s v_meas={self.v_meas:.3f} m/s")
        # else:
        #     self.last_zero_time = time.time()

    def cb_lead_dist(self, msg):
        self.lead_d = float(msg.data)

    def freeflowspd_callback(self, msg):
        self.v_max = float(msg.data)

    # --- Helpers ---

    def _publish_speed(self, v_cmd):
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = 0.0
        self.pub_twist.publish(twist)
        self.pub_vset.publish(Float32(data=v_cmd))

    def gap_keeping_speed(self, gap):
        if self.allow_catch_up and not self.is_turning:
            v_safe = self.safe_speed(gap, self.v_lead)
            if abs(v_safe - self.v_max) < 0.01:
                # free flow, try to correct headway error
                error = gap - self.desired_gap
                v_cmd = self.v_max + 0.7 * error  # simple P control
                v_cmd = min(1.2 * self.v_max, v_cmd)  # cap at 120 percent of v_max
                v_cmd = max(0.7 * self.v_max, v_cmd)  # cap at 70 percent of v_max
                return v_cmd
            else:
                return v_safe
        elif self.is_turning:
            # turning: range sometimes noisy
            if gap >= self.z_max:
                # no detection of lead car
                return 0.9 * self.v_max  # slightly under v_max to avoid oscillation
            return self.safe_speed(gap, self.v_lead)
        else:
            # straight with gap: use safe speed
            return self.safe_speed(gap, self.v_lead)

    def safe_speed(self, g, v_lead):
        inside = (self.b_model * self.tau)**2 + v_lead**2 + 2.0 * self.b_model * (g - self.min_gap)
        inside = max(0.0, inside)
        v_safe = -self.b_model * self.tau + math.sqrt(inside)
        v_safe = min(v_safe, self.v_max)
        return max(0.0, v_safe)

    # --- Main step ---
    def step(self):
        now = time.time()
        if self.t_prev is None:
            self.t_prev = now
            # make first intent explicit
            self._publish_speed(0.0)
            return

        dt = now - self.t_prev
        dt_max = 3.0 / self.rate_hz   # e.g. 3 ticks worth (for 20 Hz → 0.15 s)
        dt = max(0.01, min(dt_max, dt))
        self.t_prev = now

        # If local brake is active: hold everything at 0 (full drop-in)
        if self.local_brake:
            self.v_last = 0.0
            self.v_stage = 0.0
            v_next = 0.0
            self._publish_speed(v_next)
            return

        # Require at least one distance read; otherwise decay toward 0 gently
        if self.lead_d is None:
            if self.v_meas < 0.03:
                # we parked
                self.v_last = 0.0
            # simple decay of v_last toward 0
            v_next = max(0.0, self.v_last - self.decel_b * dt)
            v_next = min(v_next, self.v_last + self.accel_a * dt)
            self._publish_speed(v_next)
            self.v_last = v_next
            return

        g = max(0.0, self.lead_d)  # gap from bumper to bumper
        if self.g_prev is None:
            self.g_prev = g

        g_dot = (g - self.g_prev) / dt
        v_lead_inst = self.v_meas + g_dot
        # Low-pass filter on inferred leader speed
        self.v_lead = (1.0 - self.vlead_alpha) * self.v_lead + self.vlead_alpha * v_lead_inst

        self.g_prev = g

        if abs(self.v_lead) < 0.02:
            self.v_lead = 0.0  # assume stopped if very slow

        # only consider forward leader speed
        self.v_lead = max(0.0, self.v_lead)

        # Check if the front car starts moving after being stopped
        if self.v_meas == 0.0 and self.lead_d < 1.2 * self.stop_gap: # be robust to small gaps
            if self.true_stop_gap is None:
                self.true_stop_gap = self.lead_d
            if not hasattr(self, 'stopped_by_front_car') or not self.stopped_by_front_car:
                self.stopped_by_front_car = True
                self._publish_speed(0.0)
                return
        
        if self.stopped_by_front_car:
            if self.lead_d >= 1.1 * self.true_stop_gap:
                if not hasattr(self, 'front_car_moving_time'):
                    self.front_car_moving_time = now
                else:
                    if now - self.front_car_moving_time >= self.artificial_delay:
                        self.stopped_by_front_car = False
                        self.true_stop_gap = None
                        # will move next loop
                    else:
                        self._publish_speed(0.0)
                return
        

        # Ideal model speed from gap-keeping
        v_cmd_model = self.gap_keeping_speed(g)

        # ---------- v_stage update: time-based clamp toward v_cmd_model ----------
        dv_stage_max = self.a_stage * dt
        delta = v_cmd_model - self.v_stage

        if delta > self.stage_tol:
            # ramp up, but not faster than a_stage
            self.v_stage += min(delta, dv_stage_max)
        elif delta < -self.stage_tol:
            # ramp down, bounded as well
            self.v_stage += max(delta, -dv_stage_max)
        else:
            # close enough → snap to model
            self.v_stage = v_cmd_model
            if self.v_stage < 0.02:
                self.v_stage = 0.0
                v_next = 0.0
                self._publish_speed(v_next)
                self.v_last = v_next
                return

        # ---------------------------------------------------------
        # Final speed command: ramp from v_last toward v_stage
        # ---------------------------------------------------------
        v_target = self.v_stage
        v_next   = self.v_last  # always start from last command

        # --- Accelerating ---
        if v_target > v_next:
            if abs(self.v_last) < self.v_eps_start:
                accel_cap = self.a_start
            else:
                accel_cap = self.accel_a

            v_next = min(v_target, self.v_last + accel_cap * dt)

            # startup dead-zone compensation
            if abs(self.v_meas) < self.v_eps_start and v_next > 0.0:
                v_next = max(v_next, self.v_min_start)

        # --- Decelerating ---
        elif v_target < v_next:
            v_next = max(v_target, self.v_last - self.decel_b * dt)

        # --- Speed limit + hard stop ---
        v_next = min(v_next, self.v_max * 1.4)
        v_next = max(0.0, v_next)

        if self.lead_d < self.stop_gap:
            v_next = 0.0
            self.v_stage = 0.0

        # --- Debug print ---
        # if self.last_debug_time is None or (now - self.last_debug_time) > 0.1:
        #     dv_cmd = v_next - self.v_last
        #     dv_dt  = dv_cmd / dt if dt > 1e-6 else 0.0
        #     rospy.loginfo(
        #         "[krauss] v_cmd_model=%.3f v_stage=%.3f v_meas=%.3f v_next=%.3f dv/dt=%.3f",
        #         v_cmd_model, self.v_stage, self.v_meas, v_next, dv_dt
        #     )
        #     self.last_debug_time = now

        # --- Publish + update state ---
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
