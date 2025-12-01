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
from vpa_robot_interface.msg import WheelsEncoder

class KraussSpeedController(object):
    def __init__(self):
        # --- Parameters ---
        self.rate_hz    = rospy.get_param("~rate_hz", 20.0)     # control rate (Hz)
        self.v_max      = rospy.get_param("~v_max", 0.3)        # free-flow speed (m/s)
        self.accel_a    = rospy.get_param("~a", 0.093)              # accel cap (m/s^2)
        self.decel_b    = rospy.get_param("~b", 0.16)            # comfortable decel bound (m/s^2)
        self.min_gap    = rospy.get_param("~min_gap", 0.1)      # bumper clearance (m)
        self.stop_gap   = rospy.get_param("~z_stop", 0.1)      # hard stop threshold (m)
        # must match ACCLeadNode so sentinel semantics align:
        self.z_min      = rospy.get_param("~z_min", 0.1)       # min valid distance (m)
        self.z_max      = rospy.get_param("~z_max", 1)          # max valid distance (m)
        self.vlead_alpha= rospy.get_param("~vlead_alpha", 0.25) # LPF for inferred leader speed (0..1)

        self.b_model    = rospy.get_param("~b_model", 0.2)      # model decel for Krauss calc (m/s^2)
        self.tau     = rospy.get_param("~tau", 0.6)            # reaction time for Krauss calc (s)    
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

        self.is_turning = False

        self.allow_catch_up = rospy.get_param("~allow_catch_up", False)
        
        self.desired_gap_headway    = rospy.get_param("~desired_gap", 0.5) 
        # desired gap for formation control (m), this is headway distance, not bumper to bumper distan
        self.desired_gap = self.desired_gap_headway - self.car_length  # convert to bumper to bumper distance

        # check when v_safe will reduce

        # we know that v_safe = -b*tau + sqrt((b*tau)^2 + v_lead.^2 + 2*b.*(g - g_min));
        # now we set v_safte = v_max, and solve for g
        # self.d_safe_reduce = ( (self.v_max + self.b_model * self.tau)**2 - (self.b_model * self.tau)**2 ) / (2.0 * self.b_model) + self.min_gap
        # rospy.loginfo(f"[KraussSPD] v_safe will reduce when lead car is closer than {self.d_safe_reduce:.3f} m")

        # # calcaute the v_safe when at 90% of d_safe_reduce
        # g_90pct = self.d_safe_reduce * 0.9
        # inside_90pct = (self.b_model * self.tau)**2 + 0.0 + 2.0 * self.b_model * (g_90pct - self.min_gap)
        # self.v_safe_90pct = -self.b_model * self.tau + math.sqrt(inside_90pct)
        # rospy.loginfo(f"[KraussSPD] at 90% of d_safe_reduce, v_safe = {self.v_safe_90pct:.3f} m/s")

        # this is for the formation control so that we can let the following vehicle approach the leader, it will turn off before the Krauss model actually engage

        # if self.d_safe_reduce > self.desired_gap:
        #     # not gonna work
        #     rospy.logwarn(f"[KraussSPD] desired_gap {self.desired_gap:.3f} m is smaller than d_safe_reduce {self.d_safe_reduce:.3f} m, formation control may not work as expected")
        #     # signal shutdown
        #     rospy.signal_shutdown("Invalid parameters for formation control")
        

        rospy.Subscriber('freeflowspd', Float32, self.freeflowspd_callback)

        # --- ROS I/O ---
        rospy.Subscriber("wheel_omega", WheelsEncoder, self.cb_speed, queue_size=1)
        rospy.Subscriber("perception/lead_car_distance", Float32, self.cb_lead_dist, queue_size=1)
        self.pub_twist = rospy.Publisher("cmd_vel_acc", Twist, queue_size=1)
        self.pub_vset  = rospy.Publisher("v_setpoint", Float32, queue_size=1)

        rospy.loginfo("[krauss_speed_controller] Initialized with v_max=%.3f a=%.3f b=%.3f tau=%.2f z_min=%.2f z_max=%.2f",
                      self.v_max, self.accel_a, self.b_model, self.tau, self.z_min, self.z_max)

    # --- Callbacks ---
    def cb_speed(self, msg):
        self.v_meas = (msg.omega_left + msg.omega_right) * 0.5 * self.radius
        self.w_meas = (msg.omega_right - msg.omega_left) * self.radius / self.wheel_base # this is used for estimating if we are turning

        if abs(self.w_meas) > self.steer_thresh:  # rad/s threshold for turning
            self.turn_counter += 1
            if self.turn_counter >= 5:  # require 3 consecutive turning readings to set flag
            
                self.is_turning = True
            else:
                self.is_turning = False
        else:
            self.turn_counter = 0
            self.is_turning = False

    def cb_lead_dist(self, msg):
        self.lead_d = float(msg.data)

    def freeflowspd_callback(self, msg):
        self.v_max = float(msg.data)
    # --- Helpers ---
    def _publish_speed(self, v):
        tw = Twist()
        tw.linear.x = max(0.0, float(v))
        tw.angular.z = 0.0
        self.pub_twist.publish(tw)
        self.pub_vset.publish(Float32(data=tw.linear.x))

    def gap_keeping_speed(self,gap):
        # the gap is bumper to bumper distance
        if self.allow_catch_up and not self.is_turning:
            v_safe = self.safe_speed(gap, self.v_lead)
            if abs(v_safe - self.v_max) < 0.01:
                # we are in free flow regime
                # now it means we can try to catch up to desired gap
                # somedebug

                error = gap - self.desired_gap
                v_cmd = self.v_max + 0.7 * error  # simple P control
                v_cmd = min(1.2 * self.v_max, v_cmd)  # cap at 120% of v_max
                v_cmd = max(0.7 * self.v_max, v_cmd)  # cap at 80% of v_max
                return v_cmd
            else:
                return v_safe

            # if gap >= self.desired_gap:
            #     error = gap - self.desired_gap
            #     v_cmd = self.v_max + 0.5 * error  # simple P control
            #     v_cmd = min(1.2 * self.v_max, v_cmd)  # cap at 120% of v_max
            #     return v_cmd
            # elif gap < self.desired_gap and gap > self.d_safe_reduce:
            #     error = gap - self.desired_gap
            #     v_cmd = self.v_max + 1 * error  # simple P control toward v_safe at 90% d_safe_reduce
            #     # do not go below v_safe at 90% d_safe_reduce
            #     v_cmd = max(self.v_safe_90pct, v_cmd)
            #     print("gap: ", gap)
            #     print("v_cmd during catch up: ", v_cmd)
            #     return v_cmd
            # else:

            #     # use krauss safe speed calculation
            #     v_cmd = self.safe_speed(gap, self.v_lead)
            #     print("gap: ", gap)
            #     print("using krauss safe speed", v_cmd)
            #     return v_cmd
        elif self.is_turning:
            # if we turning , the range is problematic sometimes
            if gap >= self.z_max:
                # this is bascially no detection of lead car
                return 0.9 * self.v_max # we slightly under v_max to avoid oscillation
            return self.safe_speed(gap, self.v_lead)
        else:
            # we not turning but we also have to gap to keep
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
        # clamp dt to avoid huge steps after hiccups
        dt = max(0.01, min(0.2, dt))
        self.t_prev = now

        # Require at least one distance read; otherwise decay toward 0 gently
        # in the coding of this script, it will be feed with 1.6 when robot visually found no car ahead
        if self.lead_d is None:
            v_next = max(0.0, self.v_last - self.decel_b * dt)
            v_next = min(v_next, self.v_last + self.accel_a * dt)
            # slow discharge when signal turns
            self._publish_speed(v_next)
            self.v_last = v_next
            return
        
        g = max(0.0, self.lead_d) # this is gap from bumper to bumper
        if self.g_prev is None:
            self.g_prev = g

        g_dot = (g - self.g_prev) / dt
        v_lead_inst = self.v_meas + g_dot
        # Low-pass filter
        self.v_lead = (1.0 - self.vlead_alpha) * self.v_lead + self.vlead_alpha * v_lead_inst

        self.g_prev = g

        if abs(self.v_lead) < 0.02:
            self.v_lead = 0.0 # we assume stopped if very slow
        
        # Krauss safe speed calculation
        self.v_lead = max(0.0, self.v_lead)  # only consider forward speed

        v_next = self.gap_keeping_speed(g)
        
        # Apply acceleration / deceleration limits
        if v_next > self.v_last:
            v_next = min(v_next, self.v_last + self.accel_a * dt)
        # no cap on deceleration
        # Enforce speed limit
        v_next = min(v_next, self.v_max * 1.4)
        v_next = max(0.0, v_next) 

        # Hard stop if extremely close
        if self.lead_d < self.stop_gap:
            v_next = 0.0

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
