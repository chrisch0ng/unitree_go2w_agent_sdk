#!/usr/bin/env python3
"""
Simple movement demo for Unitree Go2W
No ROS required - direct SDK control

Usage: python3 test_movement.py
"""

from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import time
import sys

def main():
    print("=" * 50)
    print("Unitree Go2W Movement Demo")
    print("=" * 50)

    # Initialize connection
    print("\n[1/3] Connecting to robot via eth0...")
    try:
        ChannelFactoryInitialize(0, "eth0")
    except Exception as e:
        print(f"ERROR: Failed to initialize channel: {e}")
        print("Make sure the robot is connected via Ethernet (eth0)")
        sys.exit(1)

    print("[2/3] Initializing SportClient...")
    dog = SportClient()
    dog.SetTimeout(10.0)
    dog.Init()
    print("[3/3] Ready!\n")

    try:
        # Demo sequence
        print("Starting movement demo (Ctrl+C to stop)...\n")

        # Forward
        print(">>> Moving FORWARD (2 sec)")
        for _ in range(10):
            dog.Move(0.3, 0.0, 0.0)
            time.sleep(0.2)

        # Stop briefly
        print(">>> Stopping (1 sec)")
        dog.Move(0.0, 0.0, 0.0)
        time.sleep(1.0)

        # Rotate left
        print(">>> Rotating LEFT (2 sec)")
        for _ in range(10):
            dog.Move(0.0, 0.0, 0.5)
            time.sleep(0.2)

        # Stop briefly
        print(">>> Stopping (1 sec)")
        dog.Move(0.0, 0.0, 0.0)
        time.sleep(1.0)

        # Rotate right
        print(">>> Rotating RIGHT (2 sec)")
        for _ in range(10):
            dog.Move(0.0, 0.0, -0.5)
            time.sleep(0.2)

        # Stop briefly
        print(">>> Stopping (1 sec)")
        dog.Move(0.0, 0.0, 0.0)
        time.sleep(1.0)

        # Backward
        print(">>> Moving BACKWARD (2 sec)")
        for _ in range(10):
            dog.Move(-0.3, 0.0, 0.0)
            time.sleep(0.2)

        # Final stop
        print(">>> Stopping")
        dog.Move(0.0, 0.0, 0.0)

        print("\n" + "=" * 50)
        print("Demo complete!")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n\nInterrupted! Stopping robot...")
        dog.Move(0.0, 0.0, 0.0)
        print("Stopped.")

if __name__ == "__main__":
    main()
