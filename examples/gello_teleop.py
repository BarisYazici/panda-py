"""
Gello to Panda Teleoperation Script

Reads joint positions from Gello's Dynamixel servos and commands them
to the Panda robot using panda-py's JointPosition controller at 100Hz.

Usage:
    python gello_teleop.py <robot-hostname> [--port /dev/ttyUSB0] [--speed 0.2]

Example:
    python gello_teleop.py 192.168.1.100 --port /dev/ttyUSB0

With calibration:
    python gello_teleop.py 192.168.1.100 \\
        --joint-signs "1 -1 1 1 1 -1 1" \\
        --joint-offsets "0.0 1.5708 0.0 0.0 0.0 -1.5708 0.0"
"""

import argparse
import signal
import sys
from typing import Tuple

import numpy as np
from gello_reader import GelloReader

import panda_py
import panda_py.libfranka
from panda_py import controllers

# Global flag for graceful shutdown
running = True


def parse_float_tuple(s: str) -> Tuple[float, ...]:
    """Parse space-separated floats into a tuple."""
    return tuple(float(x) for x in s.split())


def parse_int_tuple(s: str) -> Tuple[int, ...]:
    """Parse space-separated ints into a tuple."""
    return tuple(int(x) for x in s.split())


def signal_handler(signum, frame):
    """Handle Ctrl+C for graceful shutdown."""
    global running
    print("\nShutdown requested...")
    running = False


def main():
    global running
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Gello to Panda teleoperation"
    )
    parser.add_argument(
        "hostname",
        type=str,
        help="Panda robot hostname or IP address"
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="Gello serial port (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=57600,
        help="Dynamixel baudrate (default: 1000000)"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.2,
        help="Initial movement speed factor (default: 0.2)"
    )
    parser.add_argument(
        "--filter",
        type=float,
        default=0.001,
        help="Position filter coefficient (default: 0.1, lower = smoother)"
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=100.0,
        help="Control loop frequency in Hz (default: 100)"
    )
    parser.add_argument(
        "--joint-signs",
        type=str,
        default="1 1 1 1 1 -1 1",
        help="Joint direction signs (1 or -1, space-separated, default: '1 -1 1 1 1 -1 1')"
    )
    parser.add_argument(
        "--joint-offsets",
        type=str,
        default="0 0 0 0 0 0 0",
        help="Joint calibration offsets in radians (space-separated, default: all zeros)"
    )
    parser.add_argument(
        "--gripper-open-deg",
        type=float,
        default=-180.0,
        help="Gripper angle in degrees when fully open (default: -180.0)"
    )
    parser.add_argument(
        "--gripper-closed-deg",
        type=float,
        default=-215.0,
        help="Gripper angle in degrees when fully closed (default: -215.0)"
    )
    
    args = parser.parse_args()
    
    # Parse calibration tuples
    joint_signs = parse_int_tuple(args.joint_signs)
    joint_offsets = parse_float_tuple(args.joint_offsets)
    
    if len(joint_signs) != 7:
        print(f"Error: joint-signs must have 7 values, got {len(joint_signs)}")
        return 1
    if len(joint_offsets) != 7:
        print(f"Error: joint-offsets must have 7 values, got {len(joint_offsets)}")
        return 1
    
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    gello = None
    panda = None
    gripper = None
    ctrl = None
    
    try:
        # ============================================================
        # 1. Initialize Gello reader with calibration
        # ============================================================
        print(f"Connecting to Gello on {args.port}...")
        print(f"Joint signs: {joint_signs}")
        print(f"Joint offsets: {[f'{x:.3f}' for x in joint_offsets]}")
        
        gello = GelloReader(
            port=args.port,
            baudrate=args.baudrate,
            joint_signs=joint_signs,
            joint_offsets=joint_offsets,
            enable_gripper=True,
            gripper_open_deg=args.gripper_open_deg,
            gripper_closed_deg=args.gripper_closed_deg,
        )
        
        # Test read to verify connection
        initial_gello_q = gello.read_joint_positions()
        print(f"Gello initial position (rad): {np.round(initial_gello_q, 3)}")
        
        # ============================================================
        # 2. Initialize Panda robot
        # ============================================================
        print(f"\nConnecting to Panda at {args.hostname}...")
        panda = panda_py.Panda(args.hostname)
        
        # Recover from any previous errors
        panda.recover()
        
        # Connect to gripper
        print("Connecting to Panda gripper...")
        gripper = panda_py.libfranka.Gripper(args.hostname)
        
        current_panda_q = panda.get_state().q
        gripper_state = gripper.read_once()
        print(f"Panda current joints (rad): {np.round(current_panda_q, 3)}")
        print(f"Panda gripper width: {gripper_state.width:.4f} m (max: {gripper_state.max_width:.4f} m)")
        
        # ============================================================
        # 3. Initial alignment - move Panda to Gello's position
        # ============================================================
        print(f"\nMoving Panda to Gello's position (speed factor: {args.speed})...")
        print("Please ensure the path is clear!")
        
        # Re-read Gello position just before moving
        target_q = gello.read_joint_positions()
        print(f"Moving Panda to Gello's position (speed factor: {args.speed})...")
        success = panda.move_to_joint_position(
                    target_q,
                    speed_factor=args.speed,
                    success_threshold=2.0
                  )
        if not success:
            raise RuntimeError("Failed to move Panda to initial position")
        
        print("Initial alignment complete!")
        
        # ============================================================
        # 4. Start JointPosition controller
        # ============================================================
        print(f"\nStarting JointPosition controller (filter: {args.filter})...")
        ctrl = controllers.JointPosition(filter_coeff=args.filter)
        panda.start_controller(ctrl)
        
        # ============================================================
        # 5. Main teleoperation loop
        # ============================================================
        print(f"\n{'='*60}")
        print("TELEOPERATION ACTIVE")
        print(f"Frequency: {args.frequency} Hz")
        print("Press Ctrl+C to stop")
        print(f"{'='*60}\n")
        
        loop_count = 0
        last_gripper_width = None
        gripper_max_width = gripper_state.max_width
        
        with panda.create_context(frequency=args.frequency) as ctx:
            while ctx.ok() and running:
                # Read Gello joint positions
                q_gello = gello.read_joint_positions()
                
                # Command arm joints to Panda
                ctrl.set_control(q_gello)
                
                # Read Gello positions including gripper
                gello_gripper_normalized = gello.read_gripper_normalized()
                
                # Convert normalized (0=open, 1=closed) to width
                target_gripper_width = gripper_max_width * (1.0 - gello_gripper_normalized)
                
                if last_gripper_width is None or abs(target_gripper_width - last_gripper_width) > 0.001:
                    gripper.move(target_gripper_width, speed=0.1)
                    last_gripper_width = target_gripper_width
                
                # Print status every second
                loop_count += 1
                if loop_count % int(args.frequency) == 0:
                    q_panda = panda.get_state().q
                    time = panda.get_state().time.to_msec()
                    error = np.linalg.norm(q_gello - q_panda)
                    actual_gripper_state = gripper.read_once()
                    actual_gripper_normalized = 1.0 - (actual_gripper_state.width / actual_gripper_state.max_width)
                    print(f"Time: {time:.1f}ms | "
                          f"Position error: {error:.4f} rad | "
                          f"Gripper: gello={gello_gripper_normalized:.3f} -> act={actual_gripper_normalized:.3f} (width={actual_gripper_state.width:.4f}m)")
        
        print("\nTeleoperation loop ended")
        
    except Exception as e:
        print(f"\nError: {e}")
        
        # Try to recover robot on error
        if panda is not None:
            try:
                print("Attempting to recover robot...")
                panda.recover()
            except Exception as recover_error:
                print(f"Recovery failed: {recover_error}")
        
        return 1
    
    finally:
        # ============================================================
        # Cleanup
        # ============================================================
        print("\nCleaning up...")
        
        if ctrl is not None:
            try:
                panda.stop_controller()
                print("Controller stopped")
            except Exception:
                pass
        
        if gripper is not None:
            try:
                gripper.stop()
                print("Gripper stopped")
            except Exception as e:
                print(f"Gripper stop failed: {e}")
        
        if gello is not None:
            gello.close()
        
        print("Shutdown complete")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
