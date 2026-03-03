#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty
import select

msg = """
Lite3 MPC Keyboard Teleop
---------------------------
Movement:
    W: Forward
    S: Backward
    A: Left
    D: Right
    Q: Turn Left (Yaw)
    E: Turn Right (Yaw)

Actions:
    1: Toggle Control On/Off (A button)
    2: Start Trot / Change Gait (X button)
    3: Stop (B button)
    Space: Stand Up / Sit Down (RB button)

CTRL-C to quit
"""

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.1, self.publish_joy)
        
        # State
        self.axes = [0.0] * 8
        self.buttons = [0] * 12
        
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_joy(self):
        key = self.get_key()
        
        # Determine if any movement key is pressed to manage 'fading'
        movement_key_pressed = False

        if key == 'w':
            self.axes[4] = 1.0
            movement_key_pressed = True
        elif key == 's':
            self.axes[4] = -1.0
            movement_key_pressed = True
        elif key == 'a':
            self.axes[3] = 1.0
            movement_key_pressed = True
        elif key == 'd':
            self.axes[3] = -1.0
            movement_key_pressed = True
        elif key == 'q':
            self.axes[0] = 1.0
            movement_key_pressed = True
        elif key == 'e':
            self.axes[0] = -1.0
            movement_key_pressed = True
        
        # If no movement key was pressed in this sample, decelerate or reset
        if not movement_key_pressed:
            # We decay faster than the C++ filter but not instantly
            self.axes[4] *= 0.5
            self.axes[3] *= 0.5
            self.axes[0] *= 0.5
            if abs(self.axes[4]) < 0.1: self.axes[4] = 0.0
            if abs(self.axes[3]) < 0.1: self.axes[3] = 0.0
            if abs(self.axes[0]) < 0.1: self.axes[0] = 0.0

        # Reset buttons (pulse logic)
        self.buttons = [0] * 12

        if key == '1':
            self.buttons[0] = 1
        elif key == '2':
            self.buttons[2] = 1
        elif key == '3':
            self.buttons[1] = 1
        elif key == ' ':
            self.buttons[5] = 1
        elif key == '\x03': # CTRL-C
            rclpy.shutdown()
            sys.exit(0)

        # Print feedback (clearing the line for a cleaner look)
        sys.stdout.write('\r' + ' ' * 80 + '\r')
        if any(a != 0.0 for a in self.axes) or any(b != 0 for b in self.buttons):
            sys.stdout.write(f"Teleop Sent -> Vx: {self.axes[4]:.1f}, Vy: {self.axes[3]:.1f}, Yaw: {self.axes[0]:.1f}, Buttons: {self.buttons}")
            sys.stdout.flush()

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons
        self.pub.publish(joy_msg)

def main():
    print(msg)
    rclpy.init()
    node = TeleopJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
