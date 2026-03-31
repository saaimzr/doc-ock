# kitCube robotic arm (stm32 + uart world targets)

this is a 4dof robotic arm controlled by an stm32. the firmware runs the motion loop (pwm sync + smooth trapezoid trajectories) and waits for 3d targets over `uart` to execute a full pick-and-place cycle inside its own workspace.

## what it does

1. **manual mode (teleop)**: use the on-board joysticks to move base/shoulder/elbow/claw.
2. **auto mode**: send a 3d target `(x, y, z)` in a shared “world” frame. the stm32 converts it into the robot’s workspace coordinates and then solves joint angles via analytic inverse kinematics (2-link shoulder/elbow + base yaw). if ik is out of reach or the geometry is badly off, it can fall back to the small empirical pose table so motion still works.
3. **retrainable in practice**: you can change the “goal” without changing the firmware by swapping the target source (traditional cv, isaac sim, or a vla pipeline) and just sending new coordinates.

## key idea: stm32 consumes world targets

the stm32 code in `kitCube/Core/Src/main.c` expects `uart` lines and supports:

- `T x y z` (or `V x y z`, or `PICK x y z`)
- `x,y,z`
- `X=<x>,Y=<y>,Z=<z>`

optional configuration lets you align isaac sim / vla coordinates to the arm workspace:

- `CFG ORIGIN ox oy oz` (world origin that should map to the workspace origin)
- `CFG SCALE s` (world units -> cm)
- `CFG MAP mx my mz sx sy sz`
  - `m*` choose which incoming world axis goes to workspace `x/y/z` (`0=x_world, 1=y_world, 2=z_world`)
  - `s*` optionally flips axes (`+1` or `-1`)

## hardware

- **stm32f4** (nucleo board)
- **pca9685 pwm driver** (via `i2c2`) for servo control
- **4 servos**: base, shoulder, elbow, claw
- **uart link** from a host computer (python / isaac sim / vla pipeline) to the stm32 (`usart3`)

## traditional cv workflow

traditional cv typically gives you either:

- an object pose in some camera frame (then you project it into the world/workspace frame), or
- a 2d pixel + depth (then you back-project into 3d).

once you have `(x, y, z)` in the same “world” you plan to use for the robot, just stream it to the stm32:

- make sure you set `CFG ORIGIN` / `CFG SCALE` / `CFG MAP` once (if needed)
- send `AUTO` to enter auto mode
- send `T x y z` for each pick target

if you want a drop-in example for the same “click 4 corners + detect red blob” approach you already used, see:

`host/cv_uart_sender.py`

run it like:

```bash
python host/cv_uart_sender.py --port COM6 --send-auto --z 4.5
```

## how the arm moves (what’s actually on stm32)

the firmware:

- keeps servo pulse “counts” constrained to min/max limits
- generates smooth motion using a simple trapezoid profile
- uses analytic inverse kinematics for the shoulder/elbow pair from `(r, z)` in the arm plane, and base yaw from `(x, y)`
- keeps the old empirical pose table (`arm_pose_table`) as a backup if ik solve fails
- converts logical angles into servo pulse-counts using the per-servo min/center/max values

this keeps compute time low so the uart loop can stay responsive.

## uart command reference

send one command per line, terminated by `\n` at **115200 baud**.

### target messages

- `T <x> <y> <z>`
- `V <x> <y> <z>` (alias)
- `PICK <x> <y> <z>` (alias)
- `<x>,<y>,<z>` (legacy)
- `X=<x>,Y=<y>,Z=<z>` (legacy)

when a valid target arrives, stm32 stores it and (once in auto mode) runs one pick-and-place cycle.

### mode / safety

- `AUTO`  
  enters auto mode and waits for new targets.
- `ABORT`  
  interrupts the active auto sequence (returns to home).

### coordinate alignment

- `CFG ORIGIN ox oy oz`  
  sets the world->workspace translation.
- `CFG SCALE s`  
  sets the scaling factor from world units to **cm**.
- `CFG MAP mx my mz sx sy sz`  
  selects axis mapping and sign flips.

default behavior is **identity** mapping with `origin=(0,0,0)` and `scale=1`, meaning the stm32 assumes the incoming values are already in the workspace coordinate system (cm).

## ik + motion calibration (what you will tune)

the analytic ik and the motion workspace are defined by constants in `kitCube/Core/Src/main.c`.

### the parameters you typically change

world alignment (host -> stm32):
- `CFG ORIGIN ox oy oz`
- `CFG SCALE s`
- `CFG MAP mx my mz sx sy sz`

paper-space ik geometry (stm32 side):
- `IK_BASE_X_CM`, `IK_BASE_Y_CM`: where the base rotation axis projects onto the paper plane (`z=0`), in the same workspace frame your cv outputs.
- `IK_SHOULDER_Z_CM`: shoulder pivot height above the paper plane.
- `IK_LINK1_CM`, `IK_LINK2_CM`: effective shoulder->elbow and elbow->claw-center link lengths (axis-to-axis distances).
- `IK_BASE_FORWARD_ABS_DEG`: base absolute angle offset so that `base_deg - IK_BASE_FORWARD_ABS_DEG` becomes the logical 0° that maps to `BASE_CENTER_COUNT`.
- `IK_SHOULDER_NEUTRAL_ABS_DEG`: shoulder absolute angle offset so that the logical 0° maps to `ARM1_CENTER_COUNT`.
- `IK_ELBOW_NEUTRAL_REL_DEG`: elbow neutral offset so that the logical 0° maps to `ARM2_CENTER_COUNT`.

servo angle mapping (logical degrees -> pwm counts):
- `BASE_MIN_DEG`, `BASE_MAX_DEG`
- `SHOULDER_MIN_DEG`, `SHOULDER_MAX_DEG`
- `ELBOW_MIN_DEG`, `ELBOW_MAX_DEG`

ik reach behavior:
- `IK_REACH_MARGIN_CM`: buffer around min/max reach.
- `IK_CLAMP_UNREACHABLE_TARGETS`: if `1`, targets outside reach get scaled back instead of failing.

scripted motion bounds:
- `AUTO_DROP_X_CM`, `AUTO_DROP_Y_CM`: drop zone on the paper (in workspace cm).
- `AUTO_Z_MIN_CM`, `AUTO_Z_MAX_CM`
- `AUTO_LIFT_DELTA_Z_CM`

### what you must measure (practical workflow)

1. verify your cv workspace frame is consistent with the ik frame
   (`z=0` is the paper plane; `(0,0)` is the paper corner closest to the robot and to the robot’s right; `+x` is left (robot POV); `+y` is away (robot POV)).

2. measure (or tune) `IK_BASE_X_CM`, `IK_BASE_Y_CM`
   start from shoulder/elbow held near a “known good” pose; adjust base offsets until base yaw alone places `(x,y)` close for multiple targets

3. measure (or tune) `IK_SHOULDER_Z_CM`, `IK_LINK1_CM`, `IK_LINK2_CM`
   use your manual touch poses: known `(x,y,z)` with measured `(shoulder, elbow)` counts; tune these until ik-predicted shoulder/elbow counts match across multiple points

4. tune neutral offsets
   adjust `IK_SHOULDER_NEUTRAL_ABS_DEG` and `IK_ELBOW_NEUTRAL_REL_DEG` so the manual touch poses land at the right counts

5. tune scripted z behavior using host output
   the host cv must output a meaningful `z` (cm above paper); the stm32 uses that `z` for pickup and uses the same `z` for the drop

### manual touch anchors (useful for tuning)

if you already collected “successful touch” points by hand, you can use them to sanity-check your ik geometry offsets.

example anchors you mentioned:
- pickup-ish near `(x=9.2, y=12.0, z~1.2cm)` reached with counts roughly `(base=450, shoulder=220, elbow=565)`
- `(x=2.4, y=12.3, z~2.5cm)` reached with counts roughly `(base=370, shoulder=360, elbow=590)`
- `(x=20.0, y=13.7, z~3.4cm)` reached with counts roughly `(base=550, shoulder=330, elbow=600)`

when ik is tuned, feeding those `(x,y,z)` points back into the stm32 should reproduce similar shoulder/elbow counts (within your tolerance).

### empirical table backup

the firmware keeps the older empirical pose table (`arm_pose_table`) as a backup.
it becomes relevant when ik is badly off or numerically fails.

## host-side bridge for isaac sim / vla

there’s a small script that sends targets from stdin (or jsonl) to the stm32:

`host/isaac_vla_to_stm32.py`

### install

```bash
pip install pyserial
```

### example: simple x y z text stream

create `targets.txt`:

```text
0.12 0.15 0.04
0.18 0.14 0.05
```

run:

```bash
python host/isaac_vla_to_stm32.py --port COM5 --baud 115200 --scale 100 --origin 0 0 0 --send-auto < targets.txt
```

if your isaac sim output is already in cm, set `--scale 1` (or omit it).

### example: json lines (vla-friendly)

`targets.jsonl`:

```json
{"x": 0.12, "y": 0.15, "z": 0.04}
{"x": 0.18, "y": 0.14, "z": 0.05}
```

run:

```bash
python host/isaac_vla_to_stm32.py --port COM5 --baud 115200 --send-auto < targets.jsonl
```

## isaac sim integration (practical approach)

isaac sim (or any perception stack) should:

1. produce a world-frame pick coordinate `(x, y, z)` for the next object
2. optionally save/compute the mapping parameters (`origin`, `scale`, `map`)
3. stream the coordinates to the bridge script (stdin) or directly send uart lines

the firmware itself does not run the vla/cv model; it only performs the motion + ik solve from the received coordinates.

## isaac sim data collection (teleop-style)

to collect high-quality data in isaac sim, the common pattern is:

1. run perception + planning in isaac sim (including any vla model that outputs target coordinates)
2. stream `(x, y, z)` targets into the bridge
3. log the synchronized isaac sim state for each target you send (arm pose, object pose, timing)

the current firmware interface runs a full pick-and-place cycle per target. if you later want true continuous teleoperation (waypoints along the path), you can extend the uart protocol to accept intermediate trajectories, but the “send target -> execute cycle” loop is already enough for dataset generation where the supervisor cares most about end-effector task success.

## teleoperation in practice

- start the firmware
- the stm32 boots in **manual** mode
- move the arm using the joysticks
- switch to **auto** by pressing the blue user button (or sending `AUTO` over uart)

## build notes

this repo is a stm32cubeide project under `kitCube/`.

- open the `kitCube/*.ioc` file in cubeide
- build the project
- flash to the nucleo board

