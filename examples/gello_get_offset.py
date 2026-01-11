#!/usr/bin/env python3
"""
Gello Offset Calibration Script

Determines the joint offsets needed to calibrate Gello for use with Panda robot.
Optionally connects to Panda and moves it to the calibration pose so you can
visually match Gello's position.

Usage:
    # With Panda visualization (recommended):
    python gello_get_offset.py --panda 192.168.1.100 --port /dev/ttyUSB0

    # Without Panda (manual positioning):
    python gello_get_offset.py --port /dev/ttyUSB0 --start-joints "0 0 0 -1.57 0 1.57 0"

The output offsets should be used when creating a GelloReader:
    gello = GelloReader(port="/dev/ttyUSB0", joint_offsets=(...))
"""

import argparse
import sys
from typing import Optional, Tuple

import numpy as np
from gello_reader import GelloReader


def parse_float_tuple(s: str) -> Tuple[float, ...]:
    """Parse space-separated floats into a tuple."""
    return tuple(float(x) for x in s.split())


def parse_int_tuple(s: str) -> Tuple[int, ...]:
    """Parse space-separated ints into a tuple."""
    return tuple(int(x) for x in s.split())


def move_panda_to_calibration_pose(
    panda: object,
    target_joints: Tuple[float, ...],
    speed_factor: float = 0.2,
) -> bool:
    """
    Move Panda to the calibration pose using JointPosition controller.
    
    Args:
        panda: Panda robot instance (already connected)
        target_joints: Target joint configuration in radians
        speed_factor: Movement speed (0-1) - affects interpolation time
        
    Returns:
        True if successful, False otherwise
    """
    from panda_py import controllers
    
    try:
        current_q = panda.get_state().q
        print(f"Current Panda joints (rad): {[f'{x:.3f}' for x in current_q]}")
        print(f"Target calibration pose:    {[f'{x:.3f}' for x in target_joints]}")
        
        print(f"\nMoving Panda to calibration pose...")
        print("Please ensure the workspace is clear!")
        
        # Start JointPosition controller
        ctrl = controllers.JointPosition(filter_coeff=0.01)
        panda.start_controller(ctrl)
        
        target_q = np.array(target_joints)
        q_start = current_q.copy()
        
        # Use joint motion controller to smoothly move to target
        alignment_frequency = 1000.0  # Higher frequency for smoother motion
        # Adjust max time based on speed_factor (slower speed = more time)
        max_alignment_time = 10.0 / max(speed_factor, 0.1)  # Scale inversely with speed
        position_threshold = 0.01  # Position error threshold in radians
        
        with panda.create_context(frequency=alignment_frequency, max_runtime=max_alignment_time) as ctx:
            while ctx.ok():
                # Interpolate from start to target position
                elapsed_time = ctrl.get_time()
                # Use a smooth interpolation (ease-in-out)
                # Scale time to [0, 1] over max_alignment_time
                t = min(elapsed_time / max_alignment_time, 1.0)
                # Smooth step function (ease-in-out)
                t_smooth = t * t * (3.0 - 2.0 * t)
                
                q_desired = q_start + (target_q - q_start) * t_smooth
                dq_desired = np.zeros(7)
                
                ctrl.set_control(q_desired, dq_desired)
                
                # Check if we're close enough to target
                current_state = panda.get_state()
                position_error = np.linalg.norm(current_state.q - target_q)
                
                if position_error < position_threshold:
                    print(f"Alignment complete! Final error: {position_error:.4f} rad")
                    break
                
                # Print progress every 500ms
                if ctx.num_ticks % 500 == 0:
                    print(f"Alignment progress: error = {position_error:.4f} rad, time = {elapsed_time:.2f}s")
        
        # Stop controller
        panda.stop_controller()
        
        # Verify final position
        final_state = panda.get_state()
        final_q = final_state.q
        final_error = np.linalg.norm(final_q - target_q)
        
        if final_error > position_threshold * 2:
            print(f"Warning: Alignment may not have completed. Final error: {final_error:.4f} rad")
            return False
        else:
            print(f"Panda now at (rad): {[f'{x:.3f}' for x in final_q]}")
            return True
            
    except Exception as e:
        print(f"Error moving Panda: {e}")
        return False


def get_calibration_offsets(
    gello: GelloReader,
    start_joints: Tuple[float, ...],
    joint_signs: Tuple[int, ...],
    num_samples: int = 10,
    panda: Optional[object] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute calibration offsets for Gello using direct calculation.
    
    Uses the same offset calculation as gello_teleop_single_joint.py:
    offset = gello_raw - (panda_joint / joint_sign)
    
    This is based on the calibration formula:
    calibrated = sign * (gello_raw - offset) = panda_joint
    
    Args:
        gello: GelloReader instance
        start_joints: Expected joint configuration in radians (used if panda is None)
        joint_signs: Direction mapping (1 or -1) for each joint
        num_samples: Number of readings to average
        panda: Optional Panda robot instance to get actual joint positions
        
    Returns:
        Tuple of (offsets, residual_errors)
    """
    start_joints = np.array(start_joints)
    joint_signs = np.array(joint_signs)
    
    # Warmup reads
    for _ in range(10):
        gello.read_raw_radians()
    
    # Average multiple readings for stability
    readings = []
    for _ in range(num_samples):
        readings.append(gello.read_raw_radians())
    gello_raw = np.mean(readings, axis=0)
    
    # Get target joints - use actual Panda position if available, otherwise use start_joints
    if panda is not None:
        try:
            print("Reading Panda state...")
            panda_q = np.array(panda.get_state().q)
            print(f"Panda state: {panda_q}")
            target_joints = panda_q
            print(f"\nRaw Gello readings (rad): {[f'{x:.4f}' for x in gello_raw]}")
            print(f"Panda current joints (rad): {[f'{x:.4f}' for x in target_joints]}")
        except Exception as e:
            print(f"Warning: Could not read Panda state: {e}")
            print("Using provided start_joints instead")
            target_joints = start_joints
            print(f"\nRaw Gello readings (rad): {[f'{x:.4f}' for x in gello_raw]}")
            print(f"Target joints (rad):      {[f'{x:.4f}' for x in target_joints]}")
    else:
        target_joints = start_joints
        print(f"\nRaw Gello readings (rad): {[f'{x:.4f}' for x in gello_raw]}")
        print(f"Target joints (rad):      {[f'{x:.4f}' for x in target_joints]}")
    
    # Direct offset calculation: offset = gello_raw - (panda_joint / joint_sign)
    # This is the same formula used in gello_teleop_single_joint.py
    offsets = np.zeros(7)
    errors = np.zeros(7)
    
    for i in range(7):
        # Calculate offset using the direct formula
        offsets[i] = gello_raw[i] - (target_joints[i] / joint_signs[i])
        
        # Verify calibration by computing error
        calibrated = joint_signs[i] * (gello_raw[i] - offsets[i])
        errors[i] = abs(calibrated - target_joints[i])
    
    return offsets, errors


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate Gello joint offsets for Panda teleoperation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # With Panda visualization (recommended):
    python gello_get_offset.py --panda 192.168.1.100 --port /dev/ttyUSB0
    
    # Without Panda connection:
    python gello_get_offset.py \\
        --port /dev/ttyUSB0 \\
        --start-joints "0 0 0 -1.57 0 1.57 0" \\
        --joint-signs "1 -1 1 1 1 -1 1"
"""
    )
    parser.add_argument(
        "--panda",
        type=str,
        default=None,
        help="Panda robot hostname/IP. If provided, moves Panda to calibration pose."
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for Gello (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="Dynamixel baudrate (default: 1000000)"
    )
    parser.add_argument(
        "--start-joints",
        type=str,
        default="0 0 0 -1.5708 0 1.5708 0",
        help="Target Panda joint configuration in radians (space-separated)"
    )
    parser.add_argument(
        "--joint-signs",
        type=str,
        default="1 -1 1 1 1 -1 1",
        help="Joint direction signs (1 or -1, space-separated)"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.2,
        help="Panda movement speed factor (default: 0.2)"
    )
    
    args = parser.parse_args()
    
    # Parse tuple arguments
    start_joints = parse_float_tuple(args.start_joints)
    joint_signs = parse_int_tuple(args.joint_signs)
    
    if len(start_joints) != 7:
        print(f"Error: start-joints must have 7 values, got {len(start_joints)}")
        sys.exit(1)
    if len(joint_signs) != 7:
        print(f"Error: joint-signs must have 7 values, got {len(joint_signs)}")
        sys.exit(1)
    if not all(s in (-1, 1) for s in joint_signs):
        print("Error: joint-signs must contain only 1 or -1")
        sys.exit(1)
    
    print("=" * 60)
    print("GELLO OFFSET CALIBRATION")
    print("=" * 60)
    print(f"\nGello port: {args.port}")
    print(f"Baudrate: {args.baudrate}")
    print(f"Target joints (rad): {[f'{x:.4f}' for x in start_joints]}")
    print(f"Joint signs: {joint_signs}")
    
    panda = None
    
    # If Panda hostname provided, move robot to calibration pose
    if args.panda:
        print("\n" + "-" * 60)
        print("STEP 1: Moving Panda to calibration pose")
        print("-" * 60)
        
        import panda_py
        
        try:
            panda = panda_py.Panda(args.panda)
            panda.recover()
            
            success = move_panda_to_calibration_pose(
                panda,
                start_joints,
                args.speed
            )
            
            if not success:
                print("\nFailed to move Panda. Exiting.")
                sys.exit(1)
            
            print("\n" + "=" * 60)
            print("PANDA IS NOW IN CALIBRATION POSE")
            print("=" * 60)
            print("\nLook at the Panda robot and position Gello to match its pose.")
            print("Make sure each Gello joint angle matches the corresponding Panda joint.")
            print("\nTip: Start from the base and work your way to the end-effector.")
        except Exception as e:
            print(f"Error connecting to Panda: {e}")
            print("Continuing without Panda connection...")
            panda = None
    else:
        print("\n" + "-" * 60)
        print("No Panda hostname provided (--panda)")
        print("You must manually position Gello to match the target joint configuration.")
        print("-" * 60)
    
    print("\n" + "*" * 60)
    input("Press ENTER when Gello is positioned correctly...")
    print("*" * 60)
    
    try:
        print("\n" + "-" * 60)
        print("STEP 2: Reading Gello and computing offsets")
        print("-" * 60)
        
        # Create reader with no calibration (we're finding it)
        gello = GelloReader(
            port=args.port,
            baudrate=args.baudrate,
            joint_signs=joint_signs,
            joint_offsets=(0.0,) * 7,  # No offsets for calibration
        )
        
        # Compute offsets using direct calculation (same as gello_teleop_single_joint.py)
        offsets, errors = get_calibration_offsets(
            gello, start_joints, joint_signs, panda=panda
        )
        
        # Print results
        print("\n" + "=" * 60)
        print("CALIBRATION RESULTS")
        print("=" * 60)
        
        print(f"\nComputed offsets (rad): {[f'{x:.4f}' for x in offsets]}")
        print(f"Residual errors:        {[f'{x:.4f}' for x in errors]}")
        
        # Check if errors are acceptable
        max_error = np.max(errors)
        if max_error > 0.1:
            print(f"\n⚠️  WARNING: Max error {max_error:.4f} rad is high!")
            print("    Consider repositioning Gello more accurately.")
        else:
            print(f"\n✓ Calibration looks good (max error: {max_error:.4f} rad)")
        
        # Generate code snippet
        print("\n" + "-" * 60)
        print("Copy this configuration to your code:")
        print("-" * 60)
        
        offset_str = ", ".join([f"{x:.4f}" for x in offsets])
        signs_str = ", ".join([str(s) for s in joint_signs])
        
        print(f"""
# Gello calibration for Panda
JOINT_SIGNS = ({signs_str})
JOINT_OFFSETS = ({offset_str})

gello = GelloReader(
    port="{args.port}",
    joint_signs=JOINT_SIGNS,
    joint_offsets=JOINT_OFFSETS,
)
""")
        
        # Generate command line for teleop
        print("-" * 60)
        print("Or use these command line arguments for gello_teleop.py:")
        print("-" * 60)
        offset_cli = " ".join([f"{x:.4f}" for x in offsets])
        signs_cli = " ".join([str(s) for s in joint_signs])
        print(f"""
python gello_teleop.py {args.panda if args.panda else '<panda-hostname>'} \\
    --port {args.port} \\
    --joint-signs "{signs_cli}" \\
    --joint-offsets "{offset_cli}"
""")
        
        # Verify calibration
        print("-" * 60)
        print("Verification (current calibrated reading):")
        print("-" * 60)
        
        # Create new reader with computed calibration
        gello.close()
        gello = GelloReader(
            port=args.port,
            baudrate=args.baudrate,
            joint_signs=joint_signs,
            joint_offsets=tuple(offsets),
        )
        
        calibrated = gello.read_joint_positions()
        print(f"Calibrated Gello: {[f'{x:.4f}' for x in calibrated]}")
        print(f"Target (Panda):   {[f'{x:.4f}' for x in start_joints]}")
        print(f"Max error:        {np.max(np.abs(calibrated - np.array(start_joints))):.4f} rad")
        
        gello.close()
        
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)
    
    finally:
        if panda is not None:
            try:
                # Panda cleanup if needed
                pass
            except Exception:
                pass
    
    print("\n" + "=" * 60)
    print("CALIBRATION COMPLETE!")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
