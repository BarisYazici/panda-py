# Gello to Panda Teleoperation

A minimal-latency teleoperation pipeline that reads joint positions from a [Gello](https://github.com/wuphilipp/gello_mechanical) 7-DOF robot's Dynamixel servos and commands them to a Franka Panda robot in real-time.

## Architecture

```
┌─────────────────┐     USB Serial      ┌─────────────────────────────┐
│  Gello Robot    │ ──────────────────► │  Teleoperation Script       │
│  (Dynamixel x7) │     1 Mbaud         │  ┌─────────────────────┐    │
└─────────────────┘                     │  │ GelloReader         │    │
                                        │  │ (bulk read 7 joints)│    │
                                        │  └──────────┬──────────┘    │
                                        │             │ 100 Hz        │
                                        │  ┌──────────▼──────────┐    │
                                        │  │ JointPosition Ctrl  │    │
                                        │  │ (filtered setpoint) │    │
                                        │  └──────────┬──────────┘    │
                                        └─────────────┼───────────────┘
                                                      │ 1000 Hz torque
                                        ┌─────────────▼───────────────┐
                                        │      Franka Panda Robot     │
                                        └─────────────────────────────┘
```

## Requirements

### Hardware

- **Franka Panda Robot** (FER) or **Franka Research 3** (FR3)
- **Gello Robot** - 7-DOF 3D-printed leader arm with Dynamixel servos
- **USB-to-Serial Adapter** - U2D2 or similar for Dynamixel communication
- **Real-time capable PC** - Connected to Panda via Ethernet, Gello via USB

### Software

Install the required Python packages:

```bash
pip install panda-python dynamixel-sdk numpy
```

Or using the provided requirements file:

```bash
pip install -r examples/gello_teleop/requirements.txt
```

### System Setup

1. **Panda Network**: Ensure your PC is connected to the Panda's control network (typically `192.168.1.x`)

2. **USB Permissions**: Add your user to the `dialout` group for serial port access:
   ```bash
   sudo usermod -aG dialout $USER
   # Log out and back in for changes to take effect
   ```

3. **Identify Gello Port**: Find the Dynamixel adapter:
   ```bash
   ls /dev/ttyUSB*
   # or
   ls /dev/serial/by-id/
   ```

## Usage

### Basic Usage

```bash
python gello_teleop.py <panda-hostname>
```

Example:
```bash
python gello_teleop.py 192.168.1.100
```

### Command Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Gello serial port |
| `--baudrate` | `1000000` | Dynamixel communication speed |
| `--frequency` | `100` | Control loop rate (Hz) |
| `--filter` | `0.1` | Position smoothing (0-1, lower = smoother) |
| `--speed` | `0.2` | Initial alignment movement speed |
| `--joint-signs` | `1 -1 1 1 1 -1 1` | Joint direction mapping (1 or -1) |
| `--joint-offsets` | `0 0 0 0 0 0 0` | Calibration offsets in radians |

### Full Example

```bash
python gello_teleop.py 192.168.1.100 \
    --port /dev/ttyUSB0 \
    --baudrate 1000000 \
    --frequency 100 \
    --filter 0.1 \
    --speed 0.2
```

### With Calibration

```bash
python gello_teleop.py 192.168.1.100 \
    --port /dev/ttyUSB0 \
    --joint-signs "1 -1 1 1 1 -1 1" \
    --joint-offsets "0.0 1.5708 0.0 0.0 0.0 -1.5708 0.0"
```

## Operation

1. **Startup**: The script connects to both robots and reads Gello's current position
2. **Alignment**: Panda moves to match Gello's joint configuration (ensure clear path!)
3. **Teleoperation**: Move Gello and Panda follows in real-time
4. **Shutdown**: Press `Ctrl+C` for graceful stop

## Calibration

Before first use, you must calibrate your Gello to properly map its joint positions to Panda's joint space.

### Why Calibration is Needed

1. **Joint Signs**: Gello servos may rotate in opposite directions compared to Panda joints
2. **Joint Offsets**: Gello's zero position may not match Panda's zero position

### Calibration Procedure

#### Option A: With Panda Visualization (Recommended)

The calibration script can move Panda to the calibration pose so you can visually match Gello:

1. **Run calibration with Panda connected**:
   ```bash
   python gello_get_offset.py \
       --panda 192.168.1.100 \
       --port /dev/ttyUSB0 \
       --joint-signs "1 -1 1 1 1 -1 1"
   ```

2. **Watch Panda move** to the calibration pose (home position by default)

3. **Position Gello** to match Panda's pose - look at the robot and mirror each joint

4. **Press Enter** when Gello matches Panda

5. **Copy the output** and use it when running teleoperation

#### Option B: Without Panda Connection

If you can't connect to Panda during calibration:

1. **Position Gello** to match the home pose manually:
   ```
   Panda home: [0, 0, 0, -π/2, 0, π/2, 0] radians
   ```

2. **Run calibration script**:
   ```bash
   python gello_get_offset.py \
       --port /dev/ttyUSB0 \
       --start-joints "0 0 0 -1.5708 0 1.5708 0" \
       --joint-signs "1 -1 1 1 1 -1 1"
   ```

3. **Copy the output** and use it when running teleoperation:
   ```bash
   python gello_teleop.py 192.168.1.100 \
       --joint-signs "1 -1 1 1 1 -1 1" \
       --joint-offsets "0.0 1.5708 0.0 0.0 0.0 -1.5708 0.0"
   ```

### Understanding Joint Signs

Joint signs indicate direction mapping between Gello and Panda:
- `1` = Same direction (Gello clockwise → Panda clockwise)  
- `-1` = Opposite direction (Gello clockwise → Panda counter-clockwise)

Default for Panda-compatible Gello: `1 -1 1 1 1 -1 1`

### Understanding Joint Offsets

Offsets align Gello's zero position with Panda's. The formula is:
```
panda_joint = joint_sign × (gello_raw - offset)
```

Offsets are typically multiples of π/2 (90°) depending on how servo horns were mounted.

## Configuration

### Gello Servo IDs

By default, servo IDs are `[1, 2, 3, 4, 5, 6, 7]`. If your Gello uses different IDs, modify `gello_reader.py`:

```python
gello = GelloReader(
    port="/dev/ttyUSB0",
    servo_ids=[1, 2, 3, 4, 5, 6, 7]  # Adjust as needed
)
```

### Filter Coefficient

The `--filter` parameter controls position smoothing:
- `1.0` = No filtering (immediate response, may be jerky)
- `0.1` = Moderate filtering (smooth, slight lag) **[recommended]**
- `0.01` = Heavy filtering (very smooth, noticeable lag)

### Frequency

- `100 Hz` - Recommended for most use cases
- `200 Hz` - Lower latency, requires fast USB communication
- Higher rates may cause communication timeouts

## Troubleshooting

### Communication Errors

**Problem**: `Failed to open port /dev/ttyUSB0`
- Check USB connection and port permissions
- Verify correct port with `ls /dev/ttyUSB*`

**Problem**: `Bulk read failed`
- Verify servo IDs match your Gello configuration
- Check baudrate matches servo settings (default 1 Mbaud)
- Ensure all 7 servos are powered

### Robot Errors

**Problem**: Panda enters error state
- The script will attempt automatic recovery
- If recovery fails, use Desk interface to clear errors
- Reduce movement speed or increase filtering

**Problem**: Communication violation
- Reduce `--frequency` to 50 Hz
- Increase `--filter` to 0.2 or higher

## Files

```
examples/
├── gello_teleop/
│   ├── README.md           # This file
│   └── requirements.txt    # Python dependencies
├── gello_reader.py         # Dynamixel SDK wrapper with calibration support
├── gello_get_offset.py     # Calibration script to find joint offsets
└── gello_teleop.py         # Main teleoperation script
```

## Safety

- Always ensure the robot's workspace is clear before starting
- Keep emergency stop within reach
- Start with low speed factor (`--speed 0.1`) for initial tests
- The Panda has built-in joint limit protection

## License

This example is part of [panda-py](https://github.com/JeanElsner/panda-py) and is licensed under the Apache-2.0 License.
