#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from ros_robot_pkg.srv import desiredTCP, setValue, moveRobotRelative
from std_srvs.srv import Trigger
from math import sin, cos

DEBUG = False

class JoystickTeleop:
    def __init__(self):
        # Initialize Node
        rospy.init_node('joystick_teleop_node', anonymous=False)
        
        # Retrieve frame list from parameter server
        ee_frames = rospy.get_param('/ee_frames', [])
        if ee_frames == []:
            rospy.logwarn("No end-effector frames found on parameter server '/ee_frames'. Using default ['TCP'].")
            ee_frames = ['TCP']
        
        rospy.loginfo(f"End-Effector Frames: {ee_frames}")
        
        # Configuration
        self.config = {
            'frame_list': list(ee_frames),
            'speed_presets': [0.05, 0.2, 0.5], 
            'linear_step': 0.001, # 1 mm
            'angular_step': 0.01745/2.0, # ~ 0.5 degree in radians
            'button_map': {
                'enable': 4, # LB
                'shift': 5, # RB
                'rot_shift': 2, # X (Rotation Shift)
                'mode_switch': 7, # Start
                'next_frame': 0,# A
                'next_speed': 1 # B
            },
            'axis_map': {

                'stick_left_x': 0, # Left Stick Horizontal
                'stick_left_y': 1, # Left Stick Vertical
                'stick_right_x': 3, # Right Stick Horizontal
                'stick_right_y': 4, # Right Stick Vertical
                'dpad_v': 7, # D-Pad Vertical
                'dpad_h': 6 # D-Pad Horizontal
            }        }

        # Internal State
        self.state = {
            'current_frame_idx': 0,
            'current_speed_idx': 0,
            'last_buttons': [], 
            'enable_active': False,
            'last_dpad_v_val': 0.0,   
            'last_dpad_h_val': 0.0,   
            'last_joy_time': 0,
            'control_mode': 'VELOCITY', # Options: 'VELOCITY', 'POSITION'
            'service_busy': False
        }

        self.current_cmd_vel = Twist()
        
        # Communications placeholders
        self.pub_vel = None
        self.sub_joy = None
        self.srv_set_tcp = None
        self.srv_set_speed = None
        self.srv_move_relative = None
        self.srv_stop = None
        self.srv_reupload = None
        
        # Startup
        self._init_communications()
        
        # Start Heartbeat (50Hz)
        self.timer = rospy.Timer(rospy.Duration(1.0/50.0), self._control_loop)

    def _init_communications(self):
        self.pub_vel = rospy.Publisher('/ur_cmd_vel', Twist, queue_size=1)
        rospy.loginfo("Publisher for /ur_cmd_vel ready.")

        rospy.loginfo("Waiting for ros_robot services...")
        try:
            rospy.wait_for_service('set_TCP', timeout=5)
            rospy.wait_for_service('move_ur_relative_async', timeout=5)
            rospy.wait_for_service('stop_ur', timeout=5)
            rospy.wait_for_service('reupload_ur', timeout=5)
            
            self.srv_set_tcp = rospy.ServiceProxy('set_TCP', desiredTCP)
            self.srv_move_relative = rospy.ServiceProxy('move_ur_relative_async', moveRobotRelative)            
            self.srv_set_speed = rospy.ServiceProxy('change_ref_vel', setValue)
            self.srv_stop = rospy.ServiceProxy('stop_ur', Trigger)
            self.srv_reupload = rospy.ServiceProxy('reupload_ur', Trigger)
            
            rospy.loginfo("Services connected.")
        except rospy.ROSException:
            rospy.logwarn("Service timeout. Discrete actions (Buttons) may fail.")

        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        self.state['last_joy_time'] = rospy.get_time()

    def joy_callback(self, msg):
        """
        Updates internal state. Does NOT publish.
        """
        self.state['last_joy_time'] = rospy.get_time()
        
        # Update Enable
        self.state['enable_active'] = self._process_safety(msg)

        if not self.state['enable_active']:
            self._reset_button_states()
            self._stop_robot()
            return

        # Update Velocity Variable
        self._process_velocity(msg)

        # Handle Discrete Actions
        self._process_buttons(msg)

    def _control_loop(self, event):
        """
        Heartbeat: Publishes self.current_cmd_vel at 50Hz
        """
        # Skip if Service is busy (D-Pad logic)
        if self.state['service_busy']: 
            if DEBUG:
                rospy.loginfo_throttle(1.0, "[Teleop] Heartbeat PAUSED (Busy)")
            return

        # Silence the heartbeat in POSITION mode. This prevents waking up the velocity script with "0.0" commands
        if self.state['control_mode'] == 'POSITION':
            return
        
        # Safety (Enable must be held)

        if not self.state['enable_active']:
            self.current_cmd_vel = Twist()
        
        # Publish
        self.pub_vel.publish(self.current_cmd_vel)
        
        if DEBUG:
            rospy.loginfo_throttle(0.5, f"[Teleop] Heartbeat sending: {self.current_cmd_vel.linear.x:.2f}")
        
    def _process_safety(self, msg):
        idx = self.config['button_map']['enable']
        if idx < len(msg.buttons):
            return msg.buttons[idx] == 1
        return False

    def _stop_robot(self):
        self.current_cmd_vel = Twist() # Reset class variable to zero

    def _reset_button_states(self):
        self.state['last_buttons'] = []

    def _apply_deadzone(self, value, threshold=0.1):
        if abs(value) < threshold: return 0.0
        return value

    def _process_velocity(self, msg):
        # Only allow in VELOCITY mode
        if self.state['control_mode'] != 'VELOCITY':
            self.current_cmd_vel = Twist()
            return

        linear_speed = self.config['speed_presets'][self.state['current_speed_idx']]
        angular_speed = self.config['speed_presets'][self.state['current_speed_idx']] * 2.0
        map_ax = self.config['axis_map']
        map_btn = self.config['button_map']

        # Check Shift State (RB)
        shift_active = False
        if map_btn['shift'] < len(msg.buttons):
            shift_active = msg.buttons[map_btn['shift']] == 1

        lx = self._apply_deadzone(msg.axes[map_ax['stick_left_x']])
        ly = self._apply_deadzone(msg.axes[map_ax['stick_left_y']])
        rx = self._apply_deadzone(msg.axes[map_ax['stick_right_x']])
        ry = self._apply_deadzone(msg.axes[map_ax['stick_right_y']])

        # Apply Logic
        if not shift_active:
            # --- STANDARD MODE (Translation + Yaw) ---
            self.current_cmd_vel.linear.x = ly * linear_speed   # Fwd/Back
            self.current_cmd_vel.linear.y = lx * linear_speed   # Left/Right
            self.current_cmd_vel.linear.z = ry * linear_speed   # Up/Down
            self.current_cmd_vel.angular.z = rx * angular_speed # Yaw
            
            # Zero out others for safety
            self.current_cmd_vel.angular.x = 0.0
            self.current_cmd_vel.angular.y = 0.0
            
        else:
            # --- ROTATION MODE (Pitch/Roll) ---
            self.current_cmd_vel.angular.y = ly * angular_speed # Pitch (Left Stick Y)
            self.current_cmd_vel.angular.x = rx * angular_speed # Roll (Right Stick X)
            
            # Zero out others for safety
            self.current_cmd_vel.linear.x = 0.0
            self.current_cmd_vel.linear.y = 0.0
            self.current_cmd_vel.linear.z = 0.0
            self.current_cmd_vel.angular.z = 0.0
            
    def _toggle_control_mode(self):
        if self.state['control_mode'] == 'VELOCITY':
            # SWITCHING TO POSITION MODE
            rospy.loginfo(">>> SWITCHING TO POSITION MODE (1 mm step) <<<")
            
            # Stop Robot & Kill Velocity Script on teach pendant
            self.state['service_busy'] = True # Pause Heartbeat
            self._stop_robot()
            self.pub_vel.publish(self.current_cmd_vel)
            
            try:
                self.srv_stop()         # Kill speedL script
                rospy.sleep(0.5)        # Wait for controller to settle
                self.srv_reupload()     # Upload standard script (takes time maybe 1-2 seconds)
                rospy.sleep(0.5)        # Wait for script to be ready
                
                self.state['control_mode'] = 'POSITION'
                rospy.loginfo(">>> POSITION MODE READY. Use D-Pad.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Mode switch failed: {e}")
            
            self.state['service_busy'] = False

        else:
            # SWITCHING TO VELOCITY MODE
            rospy.loginfo(">>> SWITCHING TO VELOCITY MODE. Use Sticks <<<")
            self.state['control_mode'] = 'VELOCITY'
            
    def _process_buttons(self, msg):
        # Init history
        if len(self.state['last_buttons']) != len(msg.buttons):
            self.state['last_buttons'] = [0] * len(msg.buttons)

        # --- MODE SWITCH ---
        btn_mode = self.config['button_map']['mode_switch']
        if msg.buttons[btn_mode] == 1 and self.state['last_buttons'][btn_mode] == 0:
            self._toggle_control_mode()
            
        # Buttons
        for action, btn_idx in self.config['button_map'].items():
            if action in ['enable', 'shift', 'rot_shift', 'mode_switch']: continue

            if msg.buttons[btn_idx] == 1 and self.state['last_buttons'][btn_idx] == 0:
                if action == 'next_frame':
                    self._action_cycle_frame()
                elif action == 'next_speed':
                    self._action_cycle_speed()

        self.state['last_buttons'] = list(msg.buttons)

        # D-Pad Step Logic
        if self.state['control_mode'] == 'POSITION':
            map_ax = self.config['axis_map']            
            map_btn = self.config['button_map']
            
            # Read Inputs
            curr_v = msg.axes[map_ax['dpad_v']] # Up/Down
            curr_h = msg.axes[map_ax['dpad_h']] # Left/Right
            
            # Check Shift (RB) for Z-Axis toggle
            lin_shift = False
            if map_btn['shift'] < len(msg.buttons):
                lin_shift = msg.buttons[map_btn['shift']] == 1            
                
            # Check Shift (X) for Rotation toggle
            rot_shift = False
            if map_btn['rot_shift'] < len(msg.buttons):
                rot_shift = msg.buttons[map_btn['rot_shift']] == 1
            # Detect Rising Edge to prevent repeated actions on hold
                        
            # --- Vertical Axis ---
            if curr_v != 0 and self.state['last_dpad_v_val'] == 0:
                step_dir = 1 if curr_v > 0 else -1
                
                if rot_shift:
                    self._action_step_move('pitch', step_dir) # Rot Y
                elif lin_shift:
                    self._action_step_move('z', step_dir)     # Lin Z
                else:
                    self._action_step_move('x', step_dir)     # Lin X
            
            # --- Horizontal Axis (Y) ---
            if curr_h != 0 and self.state.get('last_dpad_h_val', 0) == 0:
                step_dir = 1 if curr_h > 0 else -1
                
                if rot_shift:
                    self._action_step_move('roll', step_dir)  # Rot X
                else:
                    self._action_step_move('y', step_dir)     # Lin Y
                    
            # Update State
            self.state['last_dpad_v_val'] = curr_v
            self.state['last_dpad_h_val'] = curr_h
                    
    # ACTIONS
    def _action_cycle_frame(self):
        self.state['current_frame_idx'] = (self.state['current_frame_idx'] + 1) % len(self.config['frame_list'])
        target = self.config['frame_list'][self.state['current_frame_idx']]
        rospy.loginfo(f"Switching Frame -> {target}")
        try:
            self.srv_set_tcp(target)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set TCP: {e}")

    def _action_cycle_speed(self):
        self.state['current_speed_idx'] = (self.state['current_speed_idx'] + 1) % len(self.config['speed_presets'])
        speed = self.config['speed_presets'][self.state['current_speed_idx']]
        rospy.loginfo(f"Speed -> {speed} m/s")
        try:
            self.srv_set_speed(speed)
        except rospy.ServiceException:
            pass

    def _action_step_move(self, axis, direction):
        step = self.config['linear_step'] * direction
        frame = self.config['frame_list'][self.state['current_frame_idx']]
        target = Pose()
        target.orientation.w = 1.0
        
        # Linear Steps
        if axis in ['x', 'y', 'z']:
            step = self.config['linear_step'] * direction
            if axis == 'x': target.position.x = step
            elif axis == 'y': target.position.y = step
            elif axis == 'z': target.position.z = step
            rospy.loginfo(f"Step Lin {axis.upper()} {step*1000:.1f}mm")

        # Angular Steps in Quaternions
        elif axis in ['pitch', 'roll']:
            step = self.config['angular_step'] * direction
            # Half-angle formula for quaternion: sin(theta/2)
            sin_half = math.sin(step / 2.0)
            cos_half = math.cos(step / 2.0)
            
            target.orientation.w = cos_half
            if axis == 'roll':  target.orientation.x = sin_half # Rot around X
            if axis == 'pitch': target.orientation.y = sin_half # Rot around Y
            rospy.loginfo(f"Step Rot {axis.upper()} {math.degrees(step):.2f}deg")
        
        self.srv_move_relative(frame=frame, target_pose=target, relative_frame=frame)
        
if __name__ == "__main__":
    try:
        JoystickTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass