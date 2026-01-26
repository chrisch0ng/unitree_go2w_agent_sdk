#!/usr/bin/env python3
"""
Keyboard control for Unitree Go2W
No ROS required - direct SDK control

Controls:
  W/S - Forward/Backward
  A/D - Strafe Left/Right
  Q/E - Rotate Left/Right
  SPACE - Stop
  ESC - Quit

Usage: python3 keyboard_control.py
"""

from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import sys
import tty
import termios
import select

# Movement speeds
VX_SPEED = 0.4    # Forward/backward (m/s)
VY_SPEED = 0.3    # Strafe (m/s)
VYAW_SPEED = 0.8  # Rotation (rad/s)

HELP_TEXT = """
╔═══════════════════════════════════════════╗
║     Unitree Go2W Keyboard Control         ║
╠═══════════════════════════════════════════╣
║                                           ║
║            W - Forward                    ║
║                                           ║
║   Q        A     S     D        E         ║
║ Rotate   Strafe Back Strafe  Rotate       ║
║  Left    Left        Right   Right        ║
║                                           ║
║         SPACE - Stop                      ║
║         ESC   - Quit                      ║
║                                           ║
╚═══════════════════════════════════════════╝
"""

def get_key():
    """Get a single keypress without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
            return key.lower()
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    print("=" * 50)
    print("Unitree Go2W Keyboard Control")
    print("=" * 50)

    # Initialize connection
    print("\nConnecting to robot via eth0...")
    try:
        ChannelFactoryInitialize(0, "eth0")
    except Exception as e:
        print(f"ERROR: Failed to initialize channel: {e}")
        print("Make sure the robot is connected via Ethernet (eth0)")
        sys.exit(1)

    print("Initializing SportClient...")
    dog = SportClient()
    dog.SetTimeout(10.0)
    dog.Init()
    print("Connected!\n")

    print(HELP_TEXT)

    vx, vy, vyaw = 0.0, 0.0, 0.0

    try:
        while True:
            key = get_key()

            if key == '\x1b':  # ESC
                print("\nQuitting...")
                break
            elif key == 'w':
                vx, vy, vyaw = VX_SPEED, 0.0, 0.0
                print(f"Forward  (vx={vx:.1f})")
            elif key == 's':
                vx, vy, vyaw = -VX_SPEED, 0.0, 0.0
                print(f"Backward (vx={vx:.1f})")
            elif key == 'a':
                vx, vy, vyaw = 0.0, VY_SPEED, 0.0
                print(f"Strafe Left  (vy={vy:.1f})")
            elif key == 'd':
                vx, vy, vyaw = 0.0, -VY_SPEED, 0.0
                print(f"Strafe Right (vy={vy:.1f})")
            elif key == 'q':
                vx, vy, vyaw = 0.0, 0.0, VYAW_SPEED
                print(f"Rotate Left  (vyaw={vyaw:.1f})")
            elif key == 'e':
                vx, vy, vyaw = 0.0, 0.0, -VYAW_SPEED
                print(f"Rotate Right (vyaw={vyaw:.1f})")
            elif key == ' ':
                vx, vy, vyaw = 0.0, 0.0, 0.0
                print("STOP")
            elif key is None:
                # No key pressed, gradually stop
                vx, vy, vyaw = 0.0, 0.0, 0.0

            # Send command to robot
            dog.Move(vx, vy, vyaw)

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        print("Stopping robot...")
        dog.Move(0.0, 0.0, 0.0)
        print("Done.")

if __name__ == "__main__":
    main()
