#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from ros_robot_pkg.srv import desiredTCP, setValue, moveRobotRelative
from std_srvs.srv import Trigger

class JoystickTeleop:
    def __init__(self):
        # 1. Initialize Node
        rospy.init_node('joystick_teleop_node', anonymous=False)

        # 2. Configuration
        self.config = {
            'frame_list': ['TCP', 'davis', 'pressure_ft'],
            'speed_presets': [0.05, 0.2, 0.5], 
            'button_map': {
                'deadman': 4,
                'shift': 5,
                'mode_switch': 7,
                'next_frame': 0,# A
                'next_speed': 1 # B
            },
        'axis_map': {
                # UPDATE THESE KEYS:
                'stick_left_x': 0,   # Left/Right (was linear_y)
                'stick_left_y': 1,   # Up/Down (was linear_x)
                'stick_right_x': 3,  # Yaw (was angular_z)
                'stick_right_y': 4,  # Z (was linear_z)
                'dpad_v': 7,
                'dpad_h': 6
            }        }

        # 3. Internal State
        self.state = {
            'current_frame_idx': 0,
            'current_speed_idx': 0,
            'last_buttons': [], 
            'deadman_active': False,
            'last_dpad_v_val': 0.0,   
            'last_dpad_h_val': 0.0,   
            'last_joy_time': 0,
            'control_mode': 'VELOCITY', # Options: 'VELOCITY', 'POSITION'
            'service_busy': False
        }

        self.current_cmd_vel = Twist()
        
        # 4. Communications placeholders
        self.pub_vel = None
        self.sub_joy = None
        self.srv_set_tcp = None
        self.srv_set_speed = None
        self.srv_move_relative = None
        self.srv_stop = None
        self.srv_reupload = None
        
        # 5. Startup
        self._init_communications()
        
        # 6. Start Heartbeat (50Hz)
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
        
        # 1. Update Deadman
        self.state['deadman_active'] = self._process_safety(msg)

        if not self.state['deadman_active']:
            self._reset_button_states()
            self._stop_robot() # Zeros out the class variable
            return

        # 2. Update Velocity Variable (FIXED: Updates class variable)
        self._process_velocity(msg)

        # 3. Handle Discrete Actions
        self._process_buttons(msg)

    def _control_loop(self, event):
        """
        Heartbeat: Publishes self.current_cmd_vel at 50Hz
        """
        # 1. Skip if Service is busy (D-Pad logic)
        if self.state['service_busy']: 
            # DEBUG PRINT
            # rospy.loginfo_throttle(1.0, "[Teleop] Heartbeat PAUSED (Busy)")
            return

        # 2. NEW: Silence the heartbeat in POSITION mode
        # This prevents waking up the velocity script with "0.0" commands
        if self.state['control_mode'] == 'POSITION':
            return
        # 2. Safety Gate (Deadman must be held)
        # Note: We REMOVED the time check. As long as 'deadman_active' is True 
        # (which it stays until you release the button), we keep publishing.
        if not self.state['deadman_active']:
            self.current_cmd_vel = Twist()
        
        # 3. Publish
        self.pub_vel.publish(self.current_cmd_vel)
        
        # DEBUG PRINT
        # rospy.loginfo_throttle(0.5, f"[Teleop] Heartbeat sending: {self.current_cmd_vel.linear.x:.2f}")
        
    def _process_safety(self, msg):
        idx = self.config['button_map']['deadman']
        if idx < len(msg.buttons):
            return msg.buttons[idx] == 1
        return False

    def _stop_robot(self):
        self.current_cmd_vel = Twist() # Reset class variable to zero

    def _reset_button_states(self):
        self.state['last_buttons'] = []
        # Do not reset dpad val here to prevent triggers on resume

    def _apply_deadzone(self, value, threshold=0.1):
        if abs(value) < threshold: return 0.0
        return value

    def _process_velocity(self, msg):
        # 1. Gatekeeper: Only allow in VELOCITY mode
        if self.state['control_mode'] != 'VELOCITY':
            self.current_cmd_vel = Twist()
            return

        speed = self.config['speed_presets'][self.state['current_speed_idx']]
        map_ax = self.config['axis_map']
        map_btn = self.config['button_map']

        # 2. Check Shift State (RB)
        shift_active = False
        if map_btn['shift'] < len(msg.buttons):
            shift_active = msg.buttons[map_btn['shift']] == 1

        lx = self._apply_deadzone(msg.axes[map_ax['stick_left_x']])
        ly = self._apply_deadzone(msg.axes[map_ax['stick_left_y']])
        rx = self._apply_deadzone(msg.axes[map_ax['stick_right_x']])
        ry = self._apply_deadzone(msg.axes[map_ax['stick_right_y']])

        # 3. Apply Logic
        if not shift_active:
            # --- STANDARD MODE (Translation + Yaw) ---
            self.current_cmd_vel.linear.x = ly * speed  # Fwd/Back
            self.current_cmd_vel.linear.y = lx * speed  # Left/Right
            self.current_cmd_vel.linear.z = ry * speed  # Up/Down
            self.current_cmd_vel.angular.z = rx * speed # Yaw
            
            # Zero out others
            self.current_cmd_vel.angular.x = 0.0
            self.current_cmd_vel.angular.y = 0.0
            
        else:
            # --- ROTATION MODE (Pitch/Roll) ---
            self.current_cmd_vel.angular.y = ly * speed # Pitch (Left Stick Y)
            self.current_cmd_vel.angular.x = rx * speed # Roll (Right Stick X)
            
            # Safety: Zero out translation
            self.current_cmd_vel.linear.x = 0.0
            self.current_cmd_vel.linear.y = 0.0
            self.current_cmd_vel.linear.z = 0.0
            self.current_cmd_vel.angular.z = 0.0
            
    def _toggle_control_mode(self):
        if self.state['control_mode'] == 'VELOCITY':
            # SWITCHING TO POSITION MODE
            rospy.loginfo(">>> SWITCHING TO POSITION MODE (Jogging) <<<")
            
            # 1. Stop Robot & Kill Velocity Script
            self.state['service_busy'] = True # Pause Heartbeat
            self._stop_robot()
            self.pub_vel.publish(self.current_cmd_vel)
            
            try:
                self.srv_stop()         # Kill speedL script
                rospy.sleep(0.5)        # Wait for controller to settle
                self.srv_reupload()     # Upload standard script (Heavy Op)
                rospy.sleep(0.5)        # Wait for script to be ready
                
                self.state['control_mode'] = 'POSITION'
                rospy.loginfo(">>> POSITION MODE READY. Use D-Pad.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Mode switch failed: {e}")
            
            self.state['service_busy'] = False

        else:
            # SWITCHING TO VELOCITY MODE
            rospy.loginfo(">>> SWITCHING TO VELOCITY MODE (Sticks) <<<")
            # No heavy lifting needed; speedL takes over automatically
            self.state['control_mode'] = 'VELOCITY'
    def _process_buttons(self, msg):
        # Init history
        if len(self.state['last_buttons']) != len(msg.buttons):
            self.state['last_buttons'] = [0] * len(msg.buttons)

        # --- MODE SWITCH (Button 7) ---
        btn_mode = self.config['button_map']['mode_switch']
        if msg.buttons[btn_mode] == 1 and self.state['last_buttons'][btn_mode] == 0:
            self._toggle_control_mode()
            
        # A. Buttons
        for action, btn_idx in self.config['button_map'].items():
            if action == 'deadman': continue

            if msg.buttons[btn_idx] == 1 and self.state['last_buttons'][btn_idx] == 0:
                if action == 'next_frame':
                    self._action_cycle_frame()
                elif action == 'next_speed':
                    self._action_cycle_speed()

        self.state['last_buttons'] = list(msg.buttons)

        # B. D-Pad Step Logic
        if self.state['control_mode'] == 'POSITION':
            map_ax = self.config['axis_map']            
            
            # 1. Read Inputs
            curr_v = msg.axes[map_ax['dpad_v']] # Up/Down
            curr_h = msg.axes[map_ax['dpad_h']] # Left/Right
            
            # 2. Check Shift (RB) for Z-Axis toggle
            shift_active = False
            if self.config['button_map']['shift'] < len(msg.buttons):
                shift_active = msg.buttons[self.config['button_map']['shift']] == 1

            # 3. Detect Rising Edge (Prevent holding)
            # We track V and H separately to allow diagonal presses if needed (though rare)
            
            # --- Vertical Axis (X or Z) ---
            if curr_v != 0 and self.state['last_dpad_v_val'] == 0:
                step_dir = 1 if curr_v > 0 else -1
                
                if shift_active:
                    # Shift + Up/Down -> Z Axis
                    self._action_step_move('z', step_dir)
                else:
                    # Up/Down -> X Axis (Forward/Back)
                    self._action_step_move('x', step_dir)

            # --- Horizontal Axis (Y) ---
            # Note: You might need to add 'last_dpad_h_val' to self.state if you want strict debounce per axis
            if curr_h != 0 and self.state.get('last_dpad_h_val', 0) == 0:
                step_dir = 1 if curr_h > 0 else -1
                # Left/Right -> Y Axis
                self._action_step_move('y', step_dir)

            # Update State
            self.state['last_dpad_v_val'] = curr_v
            self.state['last_dpad_h_val'] = curr_h
                    
    # --- ACTIONS ---
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
        step = 0.001 * direction
        frame = self.config['frame_list'][self.state['current_frame_idx']]
        target = Pose()
        target.orientation.w = 1.0
        
        # Select Axis
        if axis == 'x':
            target.position.x = step
        elif axis == 'y':
            target.position.y = step
        elif axis == 'z':
            target.position.z = step
        
        rospy.loginfo(f"Step {axis.upper()} {step*1000}mm in {frame}")
        self.srv_move_relative(frame=frame, target_pose=target, relative_frame=frame)
        
if __name__ == "__main__":
    try:
        JoystickTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass