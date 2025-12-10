#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

class JoyMapper:
    def __init__(self):
        rospy.init_node('joy_mapper', anonymous=True)
        
        self.last_axes = []
        self.last_buttons = []
        
        rospy.Subscriber("/joy", Joy, self.callback)
        rospy.loginfo("--- Joystick Mapper Running ---")
        rospy.loginfo("Press any button or move any stick to see its ID.")

    def callback(self, msg):
        # 1. Initialize on first message
        if not self.last_axes:
            self.last_axes = list(msg.axes)
            self.last_buttons = list(msg.buttons)
            rospy.loginfo(f"Detected: {len(msg.axes)} Axes, {len(msg.buttons)} Buttons")
            return

        # 2. Check Axes Changes
        for i, val in enumerate(msg.axes):
            # Check for significant change (ignore tiny noise)
            if abs(val - self.last_axes[i]) > 0.1:
                rospy.loginfo(f"AXIS {i} changed to {val:.2f}")
                self.last_axes[i] = val

        # 3. Check Button Changes
        for i, val in enumerate(msg.buttons):
            if val != self.last_buttons[i]:
                state = "PRESSED" if val == 1 else "RELEASED"
                rospy.loginfo(f"BUTTON {i} -> {state}")
                self.last_buttons[i] = val

if __name__ == '__main__':
    try:
        JoyMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass