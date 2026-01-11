"""
Simple joint motion example demonstrating basic joint position control
and printing robot state information.

This example moves the robot through predefined joint positions and
prints the robot state at each step.

@warning Before executing this example, make sure there is enough space around the robot.
"""
import sys

import numpy as np

import panda_py
from panda_py import controllers


def print_robot_state(state):
    """Print relevant robot state information."""
    print("\n" + "="*60)
    print("Robot State:")
    print("="*60)
    print(f"Joint Positions (q):     {np.array2string(np.array(state.q), precision=3, separator=', ')}")
    print(f"Joint Velocities (dq):   {np.array2string(np.array(state.dq), precision=3, separator=', ')}")

    # Convert O_T_EE to numpy array and reshape to 4x4 matrix (column-major order)
    O_T_EE = np.array(state.O_T_EE).reshape(4, 4, order='F')
    ee_position = O_T_EE[:3, 3]
    print(f"End-effector Position:   {np.array2string(ee_position, precision=3, separator=', ')}")
    print(f"Control Success Rate:    {state.control_command_success_rate:.3f}")
    print("="*60 + "\n")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

    # Connect to the robot
    panda = panda_py.Panda(sys.argv[1])

    # Get current joint position
    print("Getting initial robot state...")
    initial_state = panda.get_state()
    print_robot_state(initial_state)

    # Move to start position
    print("Moving to start position...")
    panda.move_to_start()
    state = panda.get_state()
    print_robot_state(state)

    # Define some target joint positions (7 joints)
    # These are safe positions that form a simple motion sequence
    q_start = panda.q.copy()

    # Position 1: Slight variation from start
    q1 = q_start.copy()
    q1[0] += 0.3  # Move joint 0
    q1[3] -= 0.3  # Move joint 3

    # Position 2: Another variation
    q2 = q_start.copy()
    q2[1] += 0.2  # Move joint 1
    q2[3] -= 0.5  # Move joint 3
    q2[6] += 0.3  # Move joint 6

    # Position 3: Third variation
    q3 = q_start.copy()
    q3[0] -= 0.3  # Move joint 0
    q3[4] += 0.3  # Move joint 4

    positions = [q1, q2, q3, q_start]

    print(f"\nMoving through {len(positions)} joint positions...")

    # Move through each position and print state
    for i, target_q in enumerate(positions):
        print(f"\n--- Moving to position {i+1} ---")
        panda.move_to_joint_position(target_q)

        # Get and print robot state after reaching target
        state = panda.get_state()
        print_robot_state(state)

        print(f"Target reached: Position {i+1}")

    # Demonstrate smooth motion using a controller
    print("\n\n--- Demonstrating smooth sinusoidal joint motion ---")
    print("Moving joint 6 in a sinusoidal pattern for 5 seconds...")

    q_initial = panda.q.copy()
    ctrl = controllers.JointPosition()
    panda.start_controller(ctrl)

    runtime = 5.0
    with panda.create_context(frequency=1000, max_runtime=runtime) as ctx:
        while ctx.ok():
            q_desired = q_initial.copy()
            # Create sinusoidal motion on joint 6
            q_desired[6] = q_initial[6] + 0.3 * np.sin(2 * np.pi * ctrl.get_time() / 2.0)

            # Set desired joint positions and velocities
            dq_desired = np.zeros(7)
            dq_desired[6] = 0.3 * (2 * np.pi / 2.0) * np.cos(2 * np.pi * ctrl.get_time() / 2.0)

            ctrl.set_control(q_desired, dq_desired)

            # Print state every 500ms
            if ctx.num_ticks % 500 == 0:
                state = panda.get_state()
                print(f"\nTime: {ctrl.get_time():.2f}s")
                print(f"Joint 6 position: {state.q[6]:.3f} rad")
                print(f"Joint 6 velocity: {state.dq[6]:.3f} rad/s")

    panda.stop_controller()

    # Final state
    print("\n\n--- Final Robot State ---")
    final_state = panda.get_state()
    print_robot_state(final_state)

    print("Example completed successfully!")
