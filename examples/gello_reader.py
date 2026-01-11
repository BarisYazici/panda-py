"""
Gello Reader - Dynamixel SDK wrapper for reading joint positions from Gello robot.

This module provides a simple interface for reading joint positions from
the Gello 7-DOF robot's Dynamixel servos using bulk read for minimal latency.

Supports calibration via joint_signs and joint_offsets for proper mapping
to Panda robot joint space.
"""

from typing import List, Optional, Tuple

import numpy as np
from dynamixel_sdk import (COMM_SUCCESS, GroupBulkRead, PacketHandler,
                           PortHandler)

# Dynamixel Protocol 2.0 constants
PROTOCOL_VERSION = 2.0

# Control table addresses for Dynamixel XL330 / XM430 series
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

# Servo configuration
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1

# Position conversion constants (for 4096 resolution encoders)
ENCODER_RESOLUTION = 4096
ENCODER_CENTER = 2048  # Center position (0 degrees)

# Default calibration for Panda-compatible Gello
# These should be determined using gello_get_offset.py for your specific hardware
DEFAULT_JOINT_SIGNS: Tuple[int, ...] = (1, -1, 1, 1, 1, -1, 1)
DEFAULT_JOINT_OFFSETS: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# Gripper servo ID (typically servo 8 on Gello)
DEFAULT_GRIPPER_SERVO_ID = 8

# Gripper calibration constants (in degrees, converted to radians internally)
# These define the angle range for gripper open (0) to closed (1)
# Based on gello_software convention: gripper_config = (servo_id, open_degrees, closed_degrees)
# You may need to adjust these for your specific Gello hardware
GRIPPER_OPEN_DEG = -180.0   # Angle in degrees when gripper is fully open
GRIPPER_CLOSED_DEG = -215.0 # Angle in degrees when gripper is fully closed


class GelloReader:
    """
    Reader class for Gello robot's Dynamixel servos.
    
    Provides efficient bulk reading of all 7 joint positions
    with conversion to radians and calibration support.
    Optionally supports reading gripper position from servo 8.
    
    Args:
        port: Serial port for Dynamixel communication (e.g., "/dev/ttyUSB0")
        baudrate: Communication baudrate (default 1000000 for high-speed)
        servo_ids: List of 7 servo IDs (default [1, 2, 3, 4, 5, 6, 7])
        joint_signs: Direction mapping for each joint (1 or -1)
        joint_offsets: Calibration offsets in radians for each joint
        enable_gripper: Whether to read from gripper servo (default False)
        gripper_servo_id: ID of the gripper servo (default 8)
        gripper_open_deg: Angle in degrees when gripper is fully open
        gripper_closed_deg: Angle in degrees when gripper is fully closed
    """
    
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1000000,
        servo_ids: Optional[List[int]] = None,
        joint_signs: Optional[Tuple[int, ...]] = None,
        joint_offsets: Optional[Tuple[float, ...]] = None,
        enable_gripper: bool = False,
        gripper_servo_id: int = DEFAULT_GRIPPER_SERVO_ID,
        gripper_open_deg: float = GRIPPER_OPEN_DEG,
        gripper_closed_deg: float = GRIPPER_CLOSED_DEG,
    ):
        self.port = port
        self.baudrate = baudrate
        self.servo_ids = servo_ids if servo_ids is not None else [1, 2, 3, 4, 5, 6, 7]
        self.joint_signs = np.array(joint_signs if joint_signs is not None else DEFAULT_JOINT_SIGNS)
        self.joint_offsets = np.array(joint_offsets if joint_offsets is not None else DEFAULT_JOINT_OFFSETS)
        
        # Gripper configuration - convert degrees to radians (like gello_software)
        self.enable_gripper = enable_gripper
        self.gripper_servo_id = gripper_servo_id
        self.gripper_open_rad = gripper_open_deg * np.pi / 180.0
        self.gripper_closed_rad = gripper_closed_deg * np.pi / 180.0
        
        if len(self.servo_ids) != 7:
            raise ValueError("Gello requires exactly 7 servo IDs")
        if len(self.joint_signs) != 7:
            raise ValueError("joint_signs must have exactly 7 elements")
        if len(self.joint_offsets) != 7:
            raise ValueError("joint_offsets must have exactly 7 elements")
        if not all(s in (-1, 1) for s in self.joint_signs):
            raise ValueError("joint_signs must contain only 1 or -1")
        
        # Initialize port handler
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        # Open port
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {port}")
        
        # Set baudrate
        if not self.port_handler.setBaudRate(baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to set baudrate to {baudrate}")
        
        # Initialize bulk read for all servos (arm joints)
        self.bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        for servo_id in self.servo_ids:
            success = self.bulk_read.addParam(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            if not success:
                self.close()
                raise RuntimeError(f"Failed to add servo {servo_id} to bulk read")
        
        # Add gripper servo to bulk read if enabled
        if self.enable_gripper:
            success = self.bulk_read.addParam(
                self.gripper_servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            if not success:
                self.close()
                raise RuntimeError(f"Failed to add gripper servo {self.gripper_servo_id} to bulk read")
        
        # Disable torque on all servos (passive mode for reading)
        self._disable_torque()
        
        print(f"GelloReader initialized on {port} @ {baudrate} baud")
        print(f"Servo IDs: {self.servo_ids}")
        print(f"Joint signs: {self.joint_signs.tolist()}")
        print(f"Joint offsets: {[f'{x:.3f}' for x in self.joint_offsets]}")
        if self.enable_gripper:
            print(f"Gripper enabled: servo {self.gripper_servo_id} (open={gripper_open_deg:.1f}°, closed={gripper_closed_deg:.1f}°)")
    
    def _disable_torque(self) -> None:
        """Disable torque on all servos for passive reading."""
        all_servo_ids = list(self.servo_ids)
        if self.enable_gripper:
            all_servo_ids.append(self.gripper_servo_id)
        
        for servo_id in all_servo_ids:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
            if result != COMM_SUCCESS:
                print(f"Warning: Failed to disable torque on servo {servo_id}")
    
    def _to_signed_int32(self, value: int) -> int:
        """
        Convert unsigned 32-bit value to signed.
        
        Dynamixel extended position mode can return negative positions,
        which appear as large unsigned values (> 0x7FFFFFFF).
        """
        if value > 0x7FFFFFFF:
            return value - 0x100000000
        return value
    
    def _encoder_to_radians(self, encoder_value: int) -> float:
        """
        Convert encoder value to radians.
        
        Handles both standard (0-4095) and extended position mode (signed).
        Center position (2048) = 0 radians.
        Full rotation (4096 steps) = 2*pi radians.
        """
        # Convert to signed in case of extended position mode
        signed_value = self._to_signed_int32(encoder_value)
        return (signed_value - ENCODER_CENTER) * (2.0 * np.pi / ENCODER_RESOLUTION)
    
    def read_joint_positions(self) -> np.ndarray:
        """
        Read all 7 joint positions using bulk read.
        
        Applies calibration (joint_signs and joint_offsets) to convert
        raw Gello positions to Panda-compatible joint angles.
        
        Returns:
            np.ndarray: 7-element array of joint positions in radians
        
        Raises:
            RuntimeError: If bulk read communication fails
        """
        raw_positions = self.read_raw_radians()
        
        # Apply calibration: panda_joint = sign * (gello_raw - offset)
        return self.joint_signs * (raw_positions - self.joint_offsets)
    
    def read_raw_radians(self) -> np.ndarray:
        """
        Read all 7 joint positions as radians WITHOUT calibration.
        
        Returns:
            np.ndarray: 7-element array of uncalibrated positions in radians
        """
        # Execute bulk read
        result = self.bulk_read.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Bulk read failed: {self.packet_handler.getTxRxResult(result)}"
            )
        
        positions = np.zeros(7, dtype=np.float64)
        
        for i, servo_id in enumerate(self.servo_ids):
            # Check if data is available
            if not self.bulk_read.isAvailable(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise RuntimeError(f"Failed to read position from servo {servo_id}")
            
            # Get encoder value and convert to radians
            encoder_value = self.bulk_read.getData(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            positions[i] = self._encoder_to_radians(encoder_value)
        
        return positions
    
    def read_raw_positions(self) -> np.ndarray:
        """
        Read all 7 joint positions as raw encoder values (signed).
        
        Handles extended position mode where values can be negative.
        
        Returns:
            np.ndarray: 7-element array of raw encoder values (signed int32)
        """
        result = self.bulk_read.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Bulk read failed: {self.packet_handler.getTxRxResult(result)}"
            )
        
        positions = np.zeros(7, dtype=np.int32)
        
        for i, servo_id in enumerate(self.servo_ids):
            if not self.bulk_read.isAvailable(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise RuntimeError(f"Failed to read position from servo {servo_id}")
            
            raw_value = self.bulk_read.getData(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            positions[i] = self._to_signed_int32(raw_value)
        
        return positions
    
    def read_gripper_raw(self) -> int:
        """
        Read the gripper servo's raw encoder value.
        
        Returns:
            int: Raw encoder value (signed int32)
        
        Raises:
            RuntimeError: If gripper is not enabled or read fails
        """
        if not self.enable_gripper:
            raise RuntimeError("Gripper not enabled. Set enable_gripper=True in constructor.")
        
        # Execute bulk read (this reads all servos including gripper)
        result = self.bulk_read.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Bulk read failed: {self.packet_handler.getTxRxResult(result)}"
            )
        
        if not self.bulk_read.isAvailable(
            self.gripper_servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        ):
            raise RuntimeError(f"Failed to read position from gripper servo {self.gripper_servo_id}")
        
        raw_value = self.bulk_read.getData(
            self.gripper_servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        return self._to_signed_int32(raw_value)
    
    def read_gripper_radians(self) -> float:
        """
        Read the gripper position in radians (same conversion as arm joints).
        
        Returns:
            float: Gripper position in radians
        
        Raises:
            RuntimeError: If gripper is not enabled or read fails
        """
        raw = self.read_gripper_raw()
        return self._encoder_to_radians(raw)
    
    def read_gripper_normalized(self) -> float:
        """
        Read the gripper position as a normalized value.
        
        Uses the same approach as gello_software:
        1. Convert encoder to radians
        2. Normalize using open/closed angles in radians
        
        Returns:
            float: Gripper position normalized to 0.0 (open) to 1.0 (closed)
        
        Raises:
            RuntimeError: If gripper is not enabled or read fails
        """
        gripper_rad = self.read_gripper_radians()
        
        # Normalize: 0 = open, 1 = closed
        # Linear interpolation between open and closed angles
        range_size = self.gripper_closed_rad - self.gripper_open_rad
        if range_size == 0:
            return 0.0
        
        normalized = (gripper_rad - self.gripper_open_rad) / range_size
        # Clamp to [0, 1]
        return max(0.0, min(1.0, normalized))
    
    def read_all(self) -> Tuple[np.ndarray, Optional[float]]:
        """
        Read all joint positions and optionally the gripper position in one call.
        
        This is more efficient than calling read_joint_positions() and 
        read_gripper_normalized() separately as it only performs one bulk read.
        
        Returns:
            Tuple of (joint_positions, gripper_normalized)
            - joint_positions: 7-element array of joint positions in radians
            - gripper_normalized: Gripper position (0=open, 1=closed) or None if disabled
        """
        # Execute bulk read once
        result = self.bulk_read.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Bulk read failed: {self.packet_handler.getTxRxResult(result)}"
            )
        
        # Read joint positions
        positions = np.zeros(7, dtype=np.float64)
        for i, servo_id in enumerate(self.servo_ids):
            if not self.bulk_read.isAvailable(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise RuntimeError(f"Failed to read position from servo {servo_id}")
            
            encoder_value = self.bulk_read.getData(
                servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            positions[i] = self._encoder_to_radians(encoder_value)
        
        # Apply calibration
        joint_positions = self.joint_signs * (positions - self.joint_offsets)
        
        # Read gripper if enabled
        gripper_normalized = None
        if self.enable_gripper:
            if not self.bulk_read.isAvailable(
                self.gripper_servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise RuntimeError(f"Failed to read position from gripper servo {self.gripper_servo_id}")
            
            raw_value = self.bulk_read.getData(
                self.gripper_servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            # Convert to radians (same as arm joints)
            gripper_rad = self._encoder_to_radians(raw_value)
            
            # Normalize gripper using open/closed angles in radians
            range_size = self.gripper_closed_rad - self.gripper_open_rad
            if range_size != 0:
                gripper_normalized = (gripper_rad - self.gripper_open_rad) / range_size
                gripper_normalized = max(0.0, min(1.0, gripper_normalized))
            else:
                gripper_normalized = 0.0
        
        return joint_positions, gripper_normalized
    
    def close(self) -> None:
        """Close the serial port and clean up resources."""
        self.bulk_read.clearParam()
        self.port_handler.closePort()
        print("GelloReader closed")


if __name__ == "__main__":
    # Simple test script
    import time
    
    gello = GelloReader(port="/dev/ttyUSB0")
    
    print("\nReading joint positions at 100Hz for 5 seconds...")
    print("Press Ctrl+C to stop\n")
    
    try:
        start = time.time()
        count = 0
        while time.time() - start < 5.0:
            positions = gello.read_joint_positions()
            count += 1
            if count % 100 == 0:
                print(f"Joints (rad): {np.round(positions, 3)}")
            time.sleep(0.01)  # 100Hz
        
        elapsed = time.time() - start
        print(f"\nRead {count} samples in {elapsed:.2f}s ({count/elapsed:.1f} Hz)")
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        gello.close()
