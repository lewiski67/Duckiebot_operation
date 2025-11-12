#!/usr/bin/env python3
import math, random, time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from vpa_robot_interface.msg import WheelsEncoder
class KraussSpeedController:
    def __init__(self):
        # --- Parameters ---
        self.rate_hz   = rospy.get_param("~rate_hz", 20.0)
        self.v_max     = rospy.get_param("~v_max", 0.45)      # m/s
        self.accel_a   = rospy.get_param("~a", 0.1)           # m/s²
        self.decel_b   = rospy.get_param("~b", 1.5)           # m/s²
        self.noise_xi  = rospy.get_param("~xi", 0.02)         # m/s
        self.min_gap   = rospy.get_param("~min_gap", 0.10)    # m (bumper clearance)
        self.stop_gap  = rospy.get_param("~z_stop", 0.10)     # m (hard stop)
        self.tof_max   = rospy.get_param("~tof_max", 2.0)     # m
        self.alpha_tof = rospy.get_param("~tof_alpha", 0.35)  # LPF for distance
        self.beta_vl   = rospy.get_param("~vlead_alpha", 0.25)# LPF for leader speed
        self.seed      = rospy.get_param("~rng_seed", 1234)
        random.seed(self.seed)

        # --- State ---
        self.v_meas = 0.0
        self.tof_filt = None
        self.v_lead = 0.0
        self.g_prev = None
        self.t_prev = None

        # --- ROS I/O ---
        rospy.Subscriber("wheel_omega", WheelsEncoder, self.encoder_cb, queue_size=1)
        rospy.Subscriber('perception/lead_car_distance', Float32, self.lead_car_distance_callback)
        self.pub_twist = rospy.Publisher("cmd_vel_acc", Twist, queue_size=10)
        self.pub_vset  = rospy.Publisher("v_setpoint", Float32, queue_size=10)

    # --- Callbacks ---
    def encoder_cb(self, msg):
        omega_left  = msg.omega_left
        omega_right = msg.omega_right
        v_left  = omega_left * 0.0318
        v_right = omega_right * 0.0318
        self.v_meas = (v_left + v_right) / 2.0

    def lead_car_distance_callback(self, msg):
        d = max(0.0, min(self.tof_max, float(msg.data)))
        if self.tof_filt is None:
            self.tof_filt = d
        else:
            a = self.alpha_tof
            self.tof_filt = a*d + (1-a)*self.tof_filt

    # --- Main loop ---
    def step(self):
        now = time.time()
        if self.t_prev is None:
            self.t_prev = now
            return
        dt = max(0.01, min(0.2, now - self.t_prev))
        self.t_prev = now

        # Require valid ToF
        if self.tof_filt is None:
            return

        # 1. Gap and derivative
        g = max(0.0, self.tof_filt - self.min_gap)
        if self.g_prev is None:
            self.g_prev = g
            return
        g_dot = (g - self.g_prev) / dt
        self.g_prev = g

        # 2. Estimate leader speed
        v_lead_raw = max(0.0, min(self.v_max, g_dot + self.v_meas))
        self.v_lead = self.beta_vl*v_lead_raw + (1-self.beta_vl)*self.v_lead

        # 3. Krauss safe speed
        term = g - self.v_meas*dt
        R = max(0.0, self.v_lead**2 + 4*self.decel_b*term)
        denom = max(1e-6, self.v_lead + math.sqrt(R))
        v_safe = self.v_lead + (2*self.decel_b*term) / denom
        v_safe = max(0.0, v_safe)

        # 4. Candidate next speed
        v_acc_cap = self.v_meas + self.accel_a*dt
        v_cand = min(self.v_max, v_acc_cap, v_safe)
        v_next = max(0.0, v_cand - random.uniform(0.0, self.noise_xi))

        # 5. Hard stop
        if g < self.stop_gap:
            v_next = 0.0

        # 6. Publish speed command
        tw = Twist()
        tw.linear.x = v_next
        tw.angular.z = 0.0
        self.pub_twist.publish(tw)
        self.pub_vset.publish(v_next)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("krauss_speed_controller")
    KraussSpeedController().run()
