"""
Single Joint Teleoperation Test with Rate Limiting

Only teleoperates one joint from Gello to Panda with proper rate limiting
based on Franka's velocity, acceleration, and jerk limits.

Usage:
    python gello_teleop_single_joint.py <robot-hostname> --port /dev/ttyACM0 --baudrate 57600
"""

import argparse
import signal
import sys

import numpy as np
from gello_reader import GelloReader
from rate_limiter import JointPositionRateLimiter

import panda_py
import panda_py.libfranka
from panda_py import controllers

# Global flag for graceful shutdown
running = True


def signal_handler(signum, frame):
    """Handle Ctrl+C for graceful shutdown."""
    global running
    print("\nShutdown requested...")
    running = False


def main():
    global running
    
    parser = argparse.ArgumentParser(
        description="Single joint teleoperation test with rate limiting"
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
        help="Gello serial port (default: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=57600,
        help="Dynamixel baudrate (default: 57600)"
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=7,
        choices=[1, 2, 3, 4, 5, 6, 7, 8],
        help="Which joint to teleoperate (1-7 for arm, 8 for gripper, default: 7)"
    )
    parser.add_argument(
        "--joint-sign",
        type=int,
        default=1,
        choices=[-1, 1],
        help="Joint direction sign (1 or -1, default: 1)"
    )
    parser.add_argument(
        "--joint-offset",
        type=float,
        default=0.0,
        help="Joint offset in radians (default: 0.0)"
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="Speed scale factor 0-1 (default: 1.0 = full Franka speed)"
    )
    parser.add_argument(
        "--filter",
        type=float,
        default=0.001,
        help="Controller filter coefficient (default: 1.0 = no filter, lower = smoother)"
    )
    parser.add_argument(
        "--no-rate-limit",
        action="store_true",
        help="Disable rate limiting (use only for debugging)"
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
    joint_idx = args.joint - 1  # Convert to 0-indexed
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    gello = None
    panda = None
    gripper = None
    ctrl = None
    
    # Check if this is gripper teleoperation
    is_gripper = (args.joint == 8)
    
    try:
        # ============================================================
        # 1. Initialize Gello reader
        # ============================================================
        print(f"Connecting to Gello on {args.port} (baudrate: {args.baudrate})...")
        gello = GelloReader(
            port=args.port,
            baudrate=args.baudrate,
            # No calibration - read raw values
            joint_signs=tuple([1] * 7),
            joint_offsets=tuple([0.0] * 7),
            # Enable gripper reading if we're teleoperating the gripper
            enable_gripper=is_gripper,
            gripper_open_deg=args.gripper_open_deg,
            gripper_closed_deg=args.gripper_closed_deg,
        )
        
        if not is_gripper:
            # Read initial Gello position
            gello_raw = gello.read_joint_positions()
            print(f"Gello raw positions (rad): {np.round(gello_raw, 3)}")
            print(f"Gello joint {args.joint} raw: {gello_raw[joint_idx]:.4f} rad")
        else:
            # Read initial Gello gripper position
            gello_gripper_rad = gello.read_gripper_radians()
            gello_gripper_normalized = gello.read_gripper_normalized()
            print(f"Gello gripper angle: {gello_gripper_rad:.4f} rad ({np.degrees(gello_gripper_rad):.2f} deg)")
            print(f"Gello gripper normalized: {gello_gripper_normalized:.4f} (0=open, 1=closed)")
        
        # ============================================================
        # 2. Connect to Panda
        # ============================================================
        print(f"\nConnecting to Panda at {args.hostname}...")
        panda = panda_py.Panda(args.hostname)
        panda.recover()
        
        # Connect to gripper
        print("Connecting to Panda gripper...")
        gripper = panda_py.libfranka.Gripper(args.hostname)
        
        # Get current Panda position (this will be our base)
        panda_q = np.array(panda.get_state().q)
        
        if is_gripper:
            # Get current gripper state
            gripper_state = gripper.read_once()
            panda_gripper_width = gripper_state.width
            panda_gripper_max = gripper_state.max_width
            # Normalize to 0-1 (0 = open, 1 = closed, matching gello_software convention)
            panda_gripper_normalized = 1.0 - (panda_gripper_width / panda_gripper_max)
            print(f"Panda current joints (rad): {np.round(panda_q, 3)}")
            print(f"Panda gripper width: {panda_gripper_width:.4f} m (max: {panda_gripper_max:.4f} m)")
            print(f"Panda gripper normalized: {panda_gripper_normalized:.4f} (0=open, 1=closed)")
        else:
            print(f"Panda current joints (rad): {np.round(panda_q, 3)}")
            print(f"Panda joint {args.joint}: {panda_q[joint_idx]:.4f} rad")
        
        # ============================================================
        # 3. Calculate offset for this joint (or gripper)
        # ============================================================
        if is_gripper:
            # For gripper, we read from Gello's gripper servo
            print(f"\n{'='*60}")
            print(f"GRIPPER CALIBRATION")
            print(f"{'='*60}")
            print(f"Gello gripper angle: {gello_gripper_rad:.4f} rad ({np.degrees(gello_gripper_rad):.2f} deg)")
            print(f"Gello gripper normalized: {gello_gripper_normalized:.4f} (0=open, 1=closed)")
            print(f"Panda gripper normalized: {panda_gripper_normalized:.4f} (0=open, 1=closed)")
            print(f"")
            print(f"If gripper doesn't move correctly, adjust --gripper-open-deg and --gripper-closed-deg")
            print(f"Current range: {args.gripper_open_deg:.1f}° to {args.gripper_closed_deg:.1f}°")
            
            # Calculate offset between Gello and Panda normalized values
            gripper_offset = gello_gripper_normalized - panda_gripper_normalized
            print(f"Offset (Gello - Panda): {gripper_offset:.4f}")
            
            use_auto = input("\nUse auto-calculated offset? [Y/n]: ").strip().lower()
            if use_auto != 'n':
                offset = gripper_offset
                print(f"Using auto offset: {offset:.4f}")
            else:
                offset = 0.0
                print(f"Using zero offset")
        else:
            # When Gello is at gello_raw, Panda should be at panda_q[joint_idx]
            # calibrated = sign * (gello_raw - offset) should equal panda_q
            # Therefore: offset = gello_raw - panda_q / sign
            
            auto_offset = gello_raw[joint_idx] - (panda_q[joint_idx] / args.joint_sign)
            
            print(f"\n{'='*60}")
            print(f"CALIBRATION INFO FOR JOINT {args.joint}")
            print(f"{'='*60}")
            print(f"Gello raw:     {gello_raw[joint_idx]:+.4f} rad")
            print(f"Panda current: {panda_q[joint_idx]:+.4f} rad")
            print(f"User offset:   {args.joint_offset:+.4f} rad")
            print(f"Auto offset:   {auto_offset:+.4f} rad (calculated)")
            print(f"Joint sign:    {args.joint_sign:+d}")
            
            use_auto = input("\nUse auto-calculated offset? [Y/n]: ").strip().lower()
            if use_auto != 'n':
                offset = auto_offset
                print(f"Using auto offset: {offset:.4f}")
            else:
                offset = args.joint_offset
                print(f"Using user offset: {offset:.4f}")
        
        # ============================================================
        # 4. Initialize rate limiter (optional, only for arm joints)
        # ============================================================
        rate_limiter = None
        if not is_gripper and not args.no_rate_limit:
            print(f"\nInitializing rate limiter (speed scale: {args.speed_scale})...")
            rate_limiter = JointPositionRateLimiter(scale_factor=args.speed_scale)
            rate_limiter.reset(panda_q)
        elif is_gripper:
            print("\nRate limiting not applicable for gripper (using direct control)")
        else:
            print("\nRate limiting DISABLED - direct passthrough")
        
        # ============================================================
        # 5. Start controller (only for arm joints)
        # ============================================================
        if not is_gripper:
            print(f"Starting JointPosition controller (filter: {args.filter})...")
            ctrl = controllers.JointPosition(filter_coeff=args.filter)
            panda.start_controller(ctrl)
        
        # Store fixed position for all other joints
        fixed_q = panda_q.copy()
        
        # Store initial gripper state
        if is_gripper:
            initial_gripper_width = gripper.read_once().width
            initial_gripper_max = gripper.read_once().max_width
        
        # ============================================================
        # 6. Main teleoperation loop
        # ============================================================
        print(f"\n{'='*60}")
        if is_gripper:
            print(f"TELEOPERATING GRIPPER")
        else:
            print(f"TELEOPERATING JOINT {args.joint}")
        if rate_limiter:
            print(f"Rate limiting: ON (speed scale: {args.speed_scale:.0%})")
        elif not is_gripper:
            print(f"Rate limiting: OFF")
        if not is_gripper:
            print(f"Controller filter: {args.filter}")
            print(f"Other joints fixed at current position")
        print(f"Press Ctrl+C to stop")
        print(f"{'='*60}\n")
        
        loop_count = 0
        last_gripper_width = None
        
        # Use appropriate frequency: 1000Hz for arm, lower for gripper (gripper is slower)
        loop_freq = 1000.0 if not is_gripper else 100.0
        
        if is_gripper:
            # For gripper, we don't need a control context
            import time
            while running:
                # Read Gello gripper position
                gello_gripper_normalized = gello.read_gripper_normalized()
                
                # Apply calibration offset
                target_normalized = gello_gripper_normalized - offset
                # Clamp to [0, 1]
                target_normalized = max(0.0, min(1.0, target_normalized))
                
                # Convert normalized (0=open, 1=closed) to width in meters
                # Panda: width = max_width * (1 - normalized)
                target_width = panda_gripper_max * (1.0 - target_normalized)
                
                # Only command if changed significantly (gripper is slow)
                if last_gripper_width is None or abs(target_width - last_gripper_width) > 0.002:
                    # Use moderate speed for responsive operation
                    gripper.move(target_width, speed=0.1)
                    last_gripper_width = target_width
                
                # Print status every 0.5 seconds
                loop_count += 1
                if loop_count % 50 == 0:  # 50 ticks at 100Hz = 0.5s
                    actual_state = gripper.read_once()
                    actual_normalized = 1.0 - (actual_state.width / actual_state.max_width)
                    print(f"Gripper: gello={gello_gripper_normalized:.3f} -> "
                          f"cmd={target_normalized:.3f} -> "
                          f"width={target_width:.4f}m -> "
                          f"act={actual_normalized:.3f} "
                          f"(width={actual_state.width:.4f}m)")
                
                time.sleep(1.0 / loop_freq)  # Sleep for gripper loop
        else:
            # For arm joints, use Panda context
            with panda.create_context(frequency=loop_freq) as ctx:
                while ctx.ok() and running:
                    # Read Gello
                    gello_q = gello.read_joint_positions()
                    
                    # Apply calibration to the single joint
                    calibrated_joint = args.joint_sign * (gello_q[joint_idx] - offset)
                    
                    # Build target: fixed positions + one teleoperated joint
                    target_q = fixed_q.copy()
                    target_q[joint_idx] = calibrated_joint
                    
                    # Apply rate limiting if enabled
                    if rate_limiter:
                        command_q = rate_limiter.limit(target_q)
                    else:
                        command_q = target_q
                    
                    # Command to Panda
                    ctrl.set_control(command_q)
                    
                    # Print status every 0.5 seconds
                    loop_count += 1
                    if loop_count % 500 == 0:
                        actual_q = panda.get_state().q[joint_idx]
                        if rate_limiter:
                            velocity = rate_limiter.last_velocity[joint_idx]
                            print(f"J{args.joint}: cmd={calibrated_joint:+.3f} -> "
                                  f"lim={command_q[joint_idx]:+.3f} -> "
                                  f"act={actual_q:+.3f} rad | "
                                  f"vel={velocity:+.3f} rad/s")
                        else:
                            print(f"J{args.joint}: cmd={calibrated_joint:+.3f} -> "
                                  f"act={actual_q:+.3f} rad")
        
        print("\nTeleoperation ended")
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        print("\nCleaning up...")
        
        if ctrl is not None and panda is not None:
            try:
                panda.stop_controller()
                print("Controller stopped")
            except Exception as e:
                print(f"Stop controller failed: {e}")
        
        if gripper is not None:
            try:
                gripper.stop()
                print("Gripper stopped")
            except Exception as e:
                print(f"Gripper stop failed: {e}")
        
        if gello is not None:
            gello.close()
        
        print("Done")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
