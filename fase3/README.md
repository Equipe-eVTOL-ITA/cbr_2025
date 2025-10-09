# cbr_fase3 – Gesture-Guided Mission

## Overview

`cbr_fase3` implements the third-phase mission for the 2025 CBR drone: the vehicle takes off, searches for the operator’s hand, and executes hand-gesture commands until it is told to land or return home. The mission is built on top of the shared `cbr_drone_lib` offboard interface, the `gesture_classifier` package for vision, and the lightweight `fsm` framework for sequencing.

Highlights:

- Fully gesture-driven control loop with configurable PID gains for yaw and altitude corrections.
- Reusable takeoff, landing, and return-home behaviors adapted from fase1.
- YAML-driven configuration for simulation, onboard flights, and ground-station monitoring.

## Architecture

### Onboard stack

- `cbr_fase3` (`fase3` executable): finite-state machine coordinating the mission.
- `cbr_drone_lib` nodes: PX4 bridge, offboard control primitives, telemetry, and system health.
- `camera_publisher`: streams RGB images from the OAK-D camera.
- `gesture_classifier`: detects hands, provides gesture labels, and returns the normalized hand position.
- `telemetry_handler` (`telemetry_recorder`): optional logging of mission telemetry to rosbag.

### Optional ground-station stack

- `telemetry_handler`: telemetry aggregation, dashboard, and recording utilities.
- `rviz2`: visualization profile to monitor the drone pose and mission state.

## State machine at a glance

| State               | Purpose                                                                 | Transitions (event → next)                                              |
| ------------------- | ------------------------------------------------------------------------ | ----------------------------------------------------------------------- |
| **INITIAL TAKEOFF** | Arms, stores the home pose, and climbs to the configured hover altitude. | `INITIAL TAKEOFF COMPLETED` → `SEARCH`                                  |
| **SEARCH**          | Pans in place to look for an open hand.                                  | `HAND FOUND` → `GESTURE CONTROL`                                        |
| **GESTURE CONTROL** | Tracks the hand and executes motion gestures.                            | `LAND NOW` → `LANDING`, `GO HOME` → `RETURN HOME`                       |
| **LANDING**         | Performs a controlled descent once a landing command is received.        | `LANDED` → `TAKEOFF`                                                    |
| **TAKEOFF**         | Re-ascends to the previous hover height after a temporary landing.       | `TAKEOFF COMPLETED` → `GESTURE CONTROL`                                 |
| **RETURN HOME**     | Flies back to the stored home coordinates and finishes the mission.      | `AT HOME` → `FINISHED`                                                  |

All states fall back to `ERROR` if any transition emits `SEG FAULT`.

## Gesture vocabulary

The classifier returns two hands; the FSM uses the secondary entry (`gestures[1]`) when available for directional control. Commands are debounced with short buffers to avoid flicker.

| Gesture label   | Behavior in `GESTURE CONTROL`                                          |
| --------------- | ---------------------------------------------------------------------- |
| `Closed_Fist`   | Fly backward (−X) while maintaining yaw and altitude PID corrections.  |
| `Pointing_Up`   | Fly forward (+X).                                                      |
| `Victory`       | Strafe left (+Y).                                                      |
| `ILoveYou`      | Strafe right (−Y).                                                     |
| *(any other)*   | Hold position, only correcting yaw/altitude via the PID loops.         |
| `Thumb_Down`    | After a short confirmation window, trigger `LAND NOW`.                 |
| `Open_Palm`     | During search: marks the hand as found. In control: trigger `GO HOME`. |

If the hand X position remains steady for several frames, the drone commands zero velocity to damp drift.

## Key parameters

Primary mission parameters live in [`config/fsm.yaml`](config/fsm.yaml):

| Parameter                               | Description                                                                                          |
| --------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `fictual_home_x/y/z`                   | Initial home position written to the blackboard and PX4.                                             |
| `takeoff_height`                        | Target Z (NED) for both initial and post-landing takeoffs.                                           |
| `return_home_timeout`                  | Safety timeout (seconds) for the return-home sequence before declaring success.                      |
| `max_vertical_velocity` / `max_horizontal_velocity` | Limits for velocity steps in takeoff and horizontal motions.                              |
| `position_tolerance`                    | Distance threshold to declare takeoff/return-home complete.                                          |
| `control_speed`                         | Linear velocity magnitude applied for directional gestures.                                          |
| `yaw_speed` / `search_yaw_range`        | Sweep rate and range used while searching for the operator’s hand.                                  |
| `landing_velocity_max/min`              | Clamp descent velocity during landing to keep it smooth.                                             |
| `max_base_height`                       | Highest expected base top (NED `z`, negative above home). Used to shape the landing ramp.            |
| `landing_timeout`                       | Extra safety time added to the computed landing profile (seconds).                                   |
| `yaw_pid_*`, `climb_pid_*`              | PID gains applied to the hand-centroid error for yaw and altitude regulation.                        |

Camera and perception parameters are split between [`config/onboard.yaml`](config/onboard.yaml) and [`config/simulation.yaml`](config/simulation.yaml); adjust image sizes or classifier limits there. Ground-station overrides live in [`config/ground_station.yaml`](config/ground_station.yaml).

## Running the mission

### 1. Build the workspace

```bash
source install/setup.bash  # or ./install/setup.bash if not yet sourced
./tasks/build.sh cbr_2025  # wraps colcon build --packages-select cbr_fase3
```

You can run `colcon build --packages-select cbr_fase3` manually if the helper script is unavailable.

### 2. Simulation workflow

1. Launch the Gazebo world you want to test (for example `./tasks/simulate.sh warehouse`).
2. If the world is running in gz-sim, start the image bridge (`./tasks/image_bridge.sh`).
3. In a sourced shell, start the mission stack:

   ```bash
   ros2 launch cbr_fase3 simulation.launch.py
   ```

   Use `mission:=fase3` to select an alternative executable if you clone the FSM.

### 3. Onboard workflow

1. Connect to the aircraft’s companion computer and source the workspace.
2. Start the mission bundle:

   ```bash
   ./tasks/onboard.sh cbr_fase3
   ```

   The script launches the minimal telemetry stack plus the gesture classifier. The FSM waits 5 s before connecting to PX4, giving the perception nodes time to warm up.

### 4. Ground station monitoring

1. On the ground station PC, source the workspace and run:

   ```bash
   ./tasks/ground_station.sh cbr_fase3
   ```

2. RViz and the telemetry dashboard will start automatically and subscribe to the topics listed in `config/ground_station.yaml`.

## Debugging tips

- Verify that the classifier publishes both gestures and hand coordinates:
  ```bash
  ros2 topic echo /gesture_classifier/gestures
  ros2 topic echo /gesture_classifier/hand_location
  ```
- Inspect the FSM state and parameters:
  ```bash
  ros2 topic echo /cbr_fase3_fsm/state
  ros2 param list /cbr_fase3_fsm
  ```
- If the drone does not move, confirm offboard control is active (`/telemetry/drone_status`) and that the takeoff height matches the simulated world.
- Landing/return-home timeouts are logged to the console; increase them in `fsm.yaml` when flying outdoors.

## Next steps

- Tune PID gains (`yaw_pid_*`, `climb_pid_*`) based on your camera placement and latency.
- Add failsafes for lost-hand detection or low battery by extending the state machine with additional transitions.
- Record flight bags (`telemetry_recorder`) to refine gesture recognition post-flight.

