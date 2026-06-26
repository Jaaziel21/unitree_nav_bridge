# unitree_nav_bridge

A ROS 2 bridge between standard velocity commands (`geometry_msgs/Twist` on `/cmd_vel`) and **Unitree's proprietary motion API** for the **Go2** quadruped.

Unitree's SDK exposes the low-level primitive `SportClient::Move(vx, vy, vyaw)`, but it does **not** ship the glue to the standard ROS 2 stack nor the safety logic needed to drive the robot from `/cmd_vel`. This package provides that middle layer: it turns standard velocity commands into Unitree API requests, adding a *watchdog*, locomotion-mode checking and velocity saturation.

> **Status:** The `/cmd_vel → Unitree` bridge is working (drive the Go2 from teleop or any `Twist` source).
> Full **Nav2 autonomous navigation** integration (SLAM + EKF + planner) is **🚧 coming soon**.

---

## 🎥 Demo

https://github.com/user-attachments/assets/acf24a38-5336-4679-bbe4-97f8372b6731

---

## Architecture

```
                                /cmd_vel (Twist)        ┌─────────────────────┐   /api/sport/request   ┌────────────┐
   teleop / any Twist source ─────────────────────────►│  unitree_nav_bridge │ ─────────────────────► │  Go2 (API) │
                                                        │   (this package)    │     unitree_api/Req    │  SportMode │
                                                        └─────────────────────┘                        └────────────┘
                                                                  ▲                                          │
                                                                  └──── lf/sportmodestate (current mode) ────┘

   ┌─────────────────────────────────────────────────────────────────────────────────────────────┐
   │   Coming soon — full Nav2 stack                                                             │
   │                                                                                               │
   │   Go2 LiDAR ──► pointcloud_to_laserscan ──► slam_toolbox ──► Nav2 ──► /cmd_vel ──► (bridge)    │
   │   /utlidar/robot_odom ──► robot_localization (EKF) ──► odom → base_footprint TF               │
   └─────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Current data flow:**
1. Any source publishes velocity commands on `/cmd_vel` (`geometry_msgs/Twist`).
2. `unitree_nav_bridge` translates them into Unitree's motion API, applying the safety logic below.
3. The robot's current locomotion mode (`lf/sportmodestate`) is monitored to decide whether it is safe to move.

**Planned (coming soon):** the LiDAR-based SLAM + EKF + Nav2 stack that will autonomously generate `/cmd_vel`. Example configs for it are already included under `config/`.

---

## Features

- **`/cmd_vel` → `SportClient::Move` translation** — connect any `Twist` source (teleop today, Nav2 next) to the Go2.
- **Safety watchdog** — if no command arrives for `cmd_timeout` seconds, the robot stops automatically.
- **Locomotion-mode check** — commands are only sent when the quadruped is in a safe mode (standing / walking); avoids stressing the hardware while the motors are in *damping*.
- **Velocity saturation** — clamps `vx`, `vy`, `vyaw` to the robot's physical limits.
- **Fixed-rate control loop** (50 Hz by default), decoupled from command arrival via a timer.
- **Fully parameterizable** through YAML — topics, rate, limits and valid modes with no recompilation.
- Runs on a `MultiThreadedExecutor` so the control loop is not serialized behind a slow callback.

---

## Dependencies

- ROS 2 (tested on **Humble**)
- [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) (provides `unitree_api` and `unitree_go`)
- _Coming soon:_ `nav2`, `slam_toolbox`, `robot_localization`, `pointcloud_to_laserscan` for the full navigation stack.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select unitree_nav_bridge
source install/setup.bash
```

## Usage

Launch the bridge (with the defaults in `config/bridge_params.yaml`):

```bash
ros2 launch unitree_nav_bridge bridge.launch.py
```

With your own parameter file:

```bash
ros2 launch unitree_nav_bridge bridge.launch.py params_file:=/path/to/my_params.yaml
```

Then drive the robot by publishing to `/cmd_vel`, e.g. with teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Configuration

All operational values live in [`config/bridge_params.yaml`](config/bridge_params.yaml):

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cmd_vel_topic` | string | `/cmd_vel` | Input velocity topic. |
| `state_topic` | string | `lf/sportmodestate` | Robot state/mode topic. |
| `control_frequency` | double | `50.0` | `Move()` send rate (Hz). |
| `cmd_timeout` | double | `0.3` | Seconds without `cmd_vel` before stopping (watchdog). |
| `valid_locomotion_modes` | int[] | `[0, 7, 9]` | Modes in which it is safe to move the robot. |
| `max_vx` / `min_vx` | double | `1.5` / `-1.0` | Forward/backward velocity limits (m/s). |
| `max_vy` | double | `0.5` | Lateral velocity limit, symmetric (m/s). |
| `max_vyaw` | double | `1.0` | Yaw rate limit, symmetric (rad/s). |

>  **About the modes:** the `mode` values in `SportModeState` (7 = standing, 9 = walking, 12 = damping…) may vary across firmware versions. Verify yours before operating.

`config/` also ships example configs for the upcoming navigation stack:
- `ekf.yaml` — odometry fusion with `robot_localization`.
- `mapper.yaml` — mapping with `slam_toolbox`.
- `qos.yaml` — `pointcloud_to_laserscan` to generate `/utlidar/scan`.

---

## Topics

| Direction | Topic | Type |
|---|---|---|
| Sub | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Sub | `lf/sportmodestate` | `unitree_go/msg/SportModeState` |
| Pub | `/api/sport/request` | `unitree_api/msg/Request` |

---

## Package layout

```
unitree_nav_bridge/
├── src/
│   ├── unitree_bridge.cpp        # bridge node (cmd_vel -> Unitree)
│   └── ros2_sport_client.cpp     # Unitree Sport API client
├── include/unitree_nav_bridge/
│   ├── ros2_sport_client.h
│   └── patch.hpp
├── config/
│   ├── bridge_params.yaml        # bridge parameters
│   ├── ekf.yaml                  # robot_localization (EKF)   [coming soon]
│   ├── mapper.yaml               # slam_toolbox               [coming soon]
│   └── qos.yaml                  # pointcloud_to_laserscan    [coming soon]
├── launch/
│   └── bridge.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Roadmap

- [x] `/cmd_vel → Unitree` bridge with watchdog, mode check and velocity saturation
- [x] YAML-based parameterization + launch file
- [ ] LiDAR `pointcloud_to_laserscan` + `slam_toolbox` mapping
- [ ] `robot_localization` (EKF) odometry fusion
- [ ] Full Nav2 autonomous navigation
- [ ] Demo video

---

## License

[MIT](LICENSE)

> `ros2_sport_client.{h,cpp}` and `patch.hpp` come from the Unitree Robotics examples and keep their original copyright.
