#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool,Int32MultiArray, Int32, Float32
from enum import Enum, auto
from vpa_robot_operation.srv import AssignTask 
import socket
from map.signal_phases import get_phase_group_number

class MotionState(Enum):
    IDLE                    = auto()
    LANE_FOLLOWING          = auto()
    WAIT_SIGNAL             = auto()
    TRAJECTORY_FOLLOWING    = auto()
    LOST                    = auto()

class TaskState(Enum):
    NOT_ASSIGNED    = auto()
    ENTRY_PATH      = auto()
    EXIT_PATH       = auto()
    LOOP_PATH       = auto()


class TaskManager:
    def __init__(self,robot_name=None):
        self.current_task_state = TaskState.NOT_ASSIGNED
        self.entry_path = []
        self.exit_path  = []
        self.loop_path  = []
        self.loop_count = 0
        self.robot_name = robot_name if robot_name else socket.gethostname()
        try:
            rospy.wait_for_service('/center_manager/assign_task', timeout=5)
            self.assign_task = rospy.ServiceProxy('/center_manager/assign_task', AssignTask)
        except rospy.ROSException:
            rospy.logerr("Failed to connect to /center_manager/assign_task service")
            sys.exit(1)

    def clear_task(self):
        self.current_task_state = TaskState.NOT_ASSIGNED
        self.entry_path = []
        self.exit_path  = []
        self.loop_path  = []
        self.loop_count = 0

    def request_task(self):
        try:
            response = self.assign_task(self.robot_name)
            self.entry_path = response.entry_path
            self.exit_path  = response.exit_path
            self.loop_path  = response.loop_path
            self.loop_count = response.loop_count
            self.current_task_state = TaskState.ENTRY_PATH
            rospy.loginfo(f"[TaskManager] Task assigned: Entry Path: {self.entry_path}, Exit Path: {self.exit_path}, Loop Path: {self.loop_path}, Loop Count: {self.loop_count}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def update_task_state(self, id_reached):
        if self.current_task_state == TaskState.NOT_ASSIGNED:
            rospy.logwarn("[TaskManager] No task assigned. Cannot update task state.")
            return []

        elif self.current_task_state == TaskState.ENTRY_PATH:
            if id_reached in self.entry_path:
                idx = self.entry_path.index(id_reached)
                if idx == len(self.entry_path) - 1:
                    # Last tag in entry path, switch to LOOP_PATH
                    self.current_task_state = TaskState.LOOP_PATH
                    rospy.loginfo("[TaskManager] Entry path completed. Switching to LOOP_PATH.")
                    if not self.loop_path:
                        rospy.logwarn("[TaskManager] Loop path is empty. Switching to EXIT_PATH.")
                        self.current_task_state = TaskState.EXIT_PATH
                        return self.update_task_state(id_reached)  # Recursive call for exit path
                    return [self.loop_path[1]]  # First tag in loop path
                else:
                    return [self.entry_path[idx + 1]]

        elif self.current_task_state == TaskState.LOOP_PATH:
            if id_reached in self.loop_path:
                idx = self.loop_path.index(id_reached)
                if idx == len(self.loop_path) - 1:
                    # Last tag in loop path
                    if self.loop_count > 1:
                        self.loop_count -= 1
                        rospy.loginfo(f"[TaskManager] Loop continues. Remaining loops: {self.loop_count}")
                        return [self.loop_path[0]]  # Go back to the first tag in loop path
                    else:
                        # No more loops, switch to EXIT_PATH
                        self.current_task_state = TaskState.EXIT_PATH
                        rospy.loginfo("[TaskManager] Loop path completed. Switching to EXIT_PATH.")
                        if not self.exit_path:
                            rospy.logwarn("[TaskManager] Exit path is empty. Switching to NOT_ASSIGNED.")
                            self.current_task_state = TaskState.NOT_ASSIGNED
                            return []
                        return [self.exit_path[1]]  # First tag in exit path
                else:
                    return [self.loop_path[idx + 1]]

        elif self.current_task_state == TaskState.EXIT_PATH:
            if id_reached in self.exit_path:
                idx = self.exit_path.index(id_reached)
                if idx == len(self.exit_path) - 1:
                    # Last tag in exit path, switch to NOT_ASSIGNED
                    self.current_task_state = TaskState.NOT_ASSIGNED
                    rospy.loginfo("[TaskManager] Exit path completed. Task finished.")
                    return []
                return [self.exit_path[idx + 1]]

        rospy.logwarn("[TaskManager] ID not found in current path. No state update.")
        return []
    
class StateControlNode:

   # this is now hard coded for VPA testbed 
    START_TAG_ID    = 332
    END_TAG_ID      = 331

    def __init__(self):
        
        rospy.init_node('state_control_node')
        self.robot_name = socket.gethostname()

        self.debug_switch = False
        self.motion_state = MotionState.IDLE
        # Velocity command subscribers
        # We mux between following a lane (lf) or trajectory (tf)
        self.cmd_vel_lf = Twist()
        self.cmd_vel_tf = Twist()
        self.cmd_vel_acc = Twist()
        rospy.Subscriber('cmd_vel_lf', Twist, self.cmd_vel_lf_callback, queue_size=1)
        rospy.Subscriber('cmd_vel_tf', Twist, self.cmd_vel_tf_callback, queue_size=1)
        rospy.Subscriber('cmd_vel_acc', Twist, self.cmd_vel_acc_callback, queue_size=1)
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


        self.task_manager = TaskManager()


        self.global_brake       = True
        self.local_brake        = True
        self.global_brake_sub   = rospy.Subscriber('/global_brake', Bool, self.global_brake_callback)
        self.local_brake_sub    = rospy.Subscriber('local_brake', Bool, self.local_brake_callback)

        self.stop_lanefollowing = False
        self.ignore_stoplines = rospy.get_param('~ignore_stop_lines', False)
        self.near_stop_line_time = rospy.Time.now()
        rospy.Subscriber('perception/near_stop_line', Bool, self.near_stop_callback)

        self.traffic_signal_ok  = False
        self.green_phases       = []
        rospy.Subscriber('/green_phases', Int32MultiArray, self.signal_callback) # this topic is a list containing which phase groups are green

        self.tag_detect_timeout = int(3/0.05)
        self.detect_id = None
        self.next_id   = None
        # self.return_to_end_id = False
        self.last_detected_id_time = None
        rospy.Subscriber('perception/detected_tag_id', Int32, self.detected_tag_callback)

        self.traj_od_pub = rospy.Publisher('start_end_id', Int32MultiArray, queue_size=1)
        self.odom_reset_pub = rospy.Publisher('reset_odometry', Bool, queue_size=1)

        self.stop_trajectoryfollowing = True
        rospy.Subscriber('cur_traj_finished', Bool, self.cur_traj_finished_callback)
        
        rospy.Timer(rospy.Duration(0.05), self.timer_callback) # main loop at 20Hz

        self.lf_to_ws_lock = False  # Lock to prevent LF -> WS transition immediately after TF -> LF

        self.lf_to_ws_lock_timer = None

        self.collision_risk_stop = False
        rospy.Subscriber("front_range", Range, self.cb_lead_dist, queue_size=1)
        

    def cb_lead_dist(self, msg):
        distance = msg.range
        if distance < 0.05:  # threshold for collision risk
            self.collision_risk_stop = True
        else:
            self.collision_risk_stop = False

    def cmd_vel_acc_callback(self, msg):
        self.cmd_vel_acc = msg
    # Timer to release the LF -> WS lock
    def release_lf_to_ws_lock(self,event):
        self.lf_to_ws_lock = False
        rospy.loginfo("[StateControlNode] LF -> WS lock released.")

    def cur_traj_finished_callback(self, msg):
        if self.motion_state == MotionState.TRAJECTORY_FOLLOWING and msg.data:
            self.stop_trajectoryfollowing = msg.data # true 

    def detected_tag_callback(self, msg):
        if msg.data == self.detect_id:
            return  # Ignore duplicate detections of the same ID
        self.detect_id = msg.data
        self.last_detected_id_time = rospy.Time.now()
        if self.detect_id == self.START_TAG_ID:
            self.task_manager.request_task()
            self.next_id = self.task_manager.update_task_state(self.detect_id)
            rospy.loginfo(f"[StateControlNode] Detected ID: {self.detect_id}, Next ID: {self.next_id[0]}")
        elif self.detect_id == self.END_TAG_ID:
            print('-----------------------------')
            rospy.loginfo(f"[StateControlNode] Reached END_TAG_ID: {self.END_TAG_ID}. Resetting task manager.")
            print('-----------------------------')
            self.local_brake = True # software lock
            self.stop_lanefollowing = False 
            self.task_manager.clear_task()
            # self.return_to_end_id = True
            self.motion_state = MotionState.IDLE
        else:
            # we call update task state only here
            self.next_id = self.task_manager.update_task_state(self.detect_id)
            try:
                rospy.loginfo(f"[StateControlNode] Detected ID: {self.detect_id}, Next ID: {self.next_id[0]}")
            except Exception as e:
                rospy.logwarn(f"[StateControlNode] Error logging detected ID: {e}")

    def signal_callback(self, msg):
        # Check if our robot's ID is in the green phases
        self.green_phases = list(msg.data)

    def global_brake_callback(self, msg):
        self.global_brake = msg.data
    
    def local_brake_callback(self, msg):
        self.local_brake = msg.data
    
    def cmd_vel_lf_callback(self, msg):
        self.cmd_vel_lf = msg

    def cmd_vel_tf_callback(self, msg):
        self.cmd_vel_tf = msg

    def near_stop_callback(self, msg):
        if self.ignore_stoplines:
            return # not reacting to stop lines, loop in lane follow mode
        if not self.stop_lanefollowing and msg.data:
            if self.lf_to_ws_lock:
                return # ignore in case of lock
            if self.motion_state != MotionState.LANE_FOLLOWING:
                return # only care when in LF state
            self.stop_lanefollowing = True
            self.near_stop_line_time = rospy.Time.now()
            # this is when we set the flag to stop lanefollowing

    def check_traffic_signal(self):
        #  this is called when the stop_lanefollowing flag is set
        if self.motion_state == MotionState.WAIT_SIGNAL:
            # we need this only when waiting for a signal
            if self.last_detected_id_time is None:
                # this is first time we are checking after near stop line
                # and very likely the id is not detected yet
                # wait for it
            #     return
            # if self.last_detected_id_time < self.near_stop_line_time:
                # this means the last detected id is before we are near stop line
                # wait for a new detection
                self.tag_detect_timeout -= 1
                if self.tag_detect_timeout <= 0:
                    rospy.logwarn(f"[StateControlNode] Detected ID timeout after near stop line. Resetting detected ID.")
                    self.next_id = None
                    self.motion_state = MotionState.LOST
                    self.traffic_signal_ok = False
                    self.detect_id = None
                rospy.loginfo(f"[StateControlNode] Waiting for tag detection after near stop line. Timeout in {self.tag_detect_timeout*0.05:.2f} seconds.")
                self.traffic_signal_ok = False
                return

            if self.detect_id is not None:
                # we have a recently detected id
                # we need to check which what is our destination id and then we know which phase group it belongs to 
                self.tag_detect_timeout = int(3/0.05) # reset timeout
                if self.task_manager.current_task_state == TaskState.NOT_ASSIGNED:
                    if not hasattr(self, '_no_task_assigned_logged'):
                        rospy.logwarn(f"[StateControlNode] No task assigned while checking traffic signal. Assuming red signal.")
                        self._no_task_assigned_logged = True
                        
                    return
                if self.next_id is None:
                    rospy.logwarn(f"[StateControlNode] No next ID while checking traffic signal. Assuming red signal.")
                    self.traffic_signal_ok = False
                    return
                else:
                    if type(self.next_id) is list:
                        if self.next_id[0] == self.detect_id:
                            # we have not get the next id assigned yet
                            self.traffic_signal_ok = False
                            return 
                    elif self.next_id == self.detect_id:
                        self.traffic_signal_ok = False
                        return
                self.phase_num_list = get_phase_group_number(self.detect_id, self.next_id)
                if self.phase_num_list is None:
                    self.traffic_signal_ok = False
                    rospy.logwarn(f"[StateControlNode] No phase group found for IDs: ({self.detect_id}, {self.next_id}). Assuming red signal.")
                self.traffic_signal_ok = False
                for phase in self.phase_num_list:
                    if phase in self.green_phases:
                        self.traffic_signal_ok = True
                        rospy.loginfo(f"[StateControlNode] Current green phases: {self.green_phases}")
                        rospy.loginfo(f"[StateControlNode] Traffic signal OK for phase: {phase}")
                        break
            else:
                self.next_id = None
                self.traffic_signal_ok = False

    def prepare_for_trajectory(self):
        if self.next_id is not None:
            rospy.loginfo(f"[StateControlNode] Preparing trajectory to next ID: {self.next_id}")
            self.stop_trajectoryfollowing = False
            # inform trajectory following node
            self.start_end_id_msg = Int32MultiArray()
            if type(self.next_id) is list:
                self.start_end_id_msg.data = [self.detect_id, self.next_id[0]]
            else:
                self.start_end_id_msg.data = [self.detect_id, self.next_id]
            self.traj_od_pub.publish(self.start_end_id_msg)
            self.odom_reset_pub.publish(Bool(data=True)) # this will set ODOM
            # we assume now the trajectory following node will take over

    def state_transition(self):
        if not self.global_brake and not self.local_brake:
            # False or 'no braking' meaning operating normally
            if self.motion_state == MotionState.IDLE:
                # Transition logic from IDLE to LANE_FOLLOWING
                self.motion_state = MotionState.LANE_FOLLOWING
            elif self.motion_state == MotionState.LANE_FOLLOWING:
                # if self.stop_lanefollowing and self.traffic_signal_ok:
                #     self.motion_state = MotionState.TRAJECTORY_FOLLOWING
                #     rospy.loginfo(f"[StateControlNode] From LF Transitioning to TRAJECTORY_FOLLOWING state.")
                #     self.prepare_for_trajectory()
                if self.stop_lanefollowing:
                    self.motion_state = MotionState.WAIT_SIGNAL
                    rospy.loginfo(f"[StateControlNode] From LF Transitioning to WAIT_SIGNAL state.")
                else:
                    self.motion_state = MotionState.LANE_FOLLOWING
            elif self.motion_state == MotionState.WAIT_SIGNAL:
                if self.traffic_signal_ok and self.detect_id is not None:
                    self.motion_state = MotionState.TRAJECTORY_FOLLOWING
                    rospy.loginfo(f"[StateControlNode] From WAIT_SIGNAL Transitioning to TRAJECTORY_FOLLOWING state.")
                    self.prepare_for_trajectory()
                else:
                    self.motion_state = MotionState.WAIT_SIGNAL
            elif self.motion_state == MotionState.TRAJECTORY_FOLLOWING:
                if self.stop_trajectoryfollowing:
                    
                    self.odom_reset_pub.publish(Bool(data=False))  # reset ODOM
                    self.motion_state = MotionState.LANE_FOLLOWING
                    rospy.loginfo(f"[StateControlNode] From TF Transitioning to LANE_FOLLOWING state.")
                    self.detect_id = None
                    self.last_detected_id_time = None  
                    # Set LF -> WS lock and start timer to release it
                    self.lf_to_ws_lock = True
                    if self.lf_to_ws_lock_timer:
                        self.lf_to_ws_lock_timer.shutdown()
                    self.lf_to_ws_lock_timer = rospy.Timer(rospy.Duration(1), self.release_lf_to_ws_lock, oneshot=True)
                    self.stop_lanefollowing = False
            elif self.motion_state == MotionState.LOST:
                if not hasattr(self, '_lost_state_logged'):
                    rospy.loginfo(f"[StateControlNode] LOST state")
                    self._lost_state_logged = True
        else:
            # True or 'braking' meaning stop all motion
            self.motion_state = MotionState.IDLE

    def timer_callback(self, event):
        # Here you can implement the state machine logic
        self.state_transition()
        if self.motion_state == MotionState.IDLE:
            # this refers to the robot being stationary before starting any task
            cmd = Twist()  # zero velocity
        elif self.motion_state == MotionState.LANE_FOLLOWING:
            cmd = self.cmd_vel_lf
        elif self.motion_state == MotionState.WAIT_SIGNAL:
            cmd = Twist()  # zero velocity while waiting
            self.check_traffic_signal()
        elif self.motion_state == MotionState.TRAJECTORY_FOLLOWING:
            cmd = self.cmd_vel_tf
        else:
            cmd = Twist()  # default to zero velocity

        if self.motion_state in [MotionState.LANE_FOLLOWING, MotionState.TRAJECTORY_FOLLOWING]:
            cmd.linear.x = max(0.0, self.cmd_vel_acc.linear.x)  # prevent reversing
            if self.collision_risk_stop:
                cmd.linear.x = 0.0
                if not hasattr(self, '_collision_logged'):
                    rospy.loginfo("[StateControlNode] Collision risk detected. Stopping the robot.")
                    self._collision_logged = True

        # Publish the final cmd_vel
        self.cmd_vel_pub.publish(cmd)
        

if __name__ == '__main__':
    try:
        node = StateControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass