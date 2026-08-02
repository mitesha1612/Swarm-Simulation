# ESP32 Colour-Triggered Swarm Robot System

A swarm of three autonomous robots that navigate an obstacle-filled arena, detect a green colour target, and converge on it cooperatively — with no central controller.

The system exists at two levels: a **ROS 2 + Gazebo simulation** (Antbots) for development and testing, and a **physical deployment** on ESP32-VROOM hardware.

## 🎥 Demo Video

**[📥 Download from Releases](https://github.com/mitesha1612/Swarm-Simulation/releases/tag/v1.0)**

The simulation shows three autonomous robots navigating an arena, detecting a green target, and converging cooperatively using the Lévy-flight search pattern with obstacle avoidance.

> Go to the Releases page above to download the simulation demo video.

---

## 📌 Project Overview

Each robot roams independently using an ant-inspired Lévy-flight search pattern. IR sensors handle obstacle avoidance. A camera (simulation) or colour sensor (hardware) watches for green. The moment any bot reaches the green target, it broadcasts the location to its peers over a shared topic (simulation) or ESP-NOW (hardware), and the others navigate to converge.

---

## 🧰 Hardware (Per Physical Robot)

| Component | Qty | Purpose |
|---|---|---|
| ESP32-VROOM | 1 | Main MCU + Wi-Fi / ESP-NOW |
| IR Sensor | 3 | Obstacle detection — left, centre, right |
| Colour Sensor (TCS3200 / TCS34725) | 1 | Green target detection |
| BO Motor + driver | 2 | Differential drive |
| Buzzer | 1 | Audible alert on green detection |

**Total bots: 3**

---

## 🖥️ Simulation Stack

| Layer | Technology |
|---|---|
| Physics / rendering | Gazebo Classic (gazebo_ros) |
| Robot middleware | ROS 2 Humble |
| Robot description | URDF / Xacro (`antbot.urdf.xacro`) |
| Swarm logic | Python (`swarm_logic.py`) |
| GUI dashboard | Tkinter desktop app (`swarm_gui_server.py`) |
| Arena | Custom SDF world (`arena.world`) — 2 × 2 m walled enclosure |

### ROS 2 Package Layout

```
Antbots/src/
├── antbot_description/          # URDF/Xacro robot model
│   └── urdf/antbot.urdf.xacro   # Differential-drive bot with camera + LiDAR
├── antbot_gazebo/               # Simulation world and spawn
│   ├── worlds/arena.world       # 2×2 m arena with walls + coloured cubes
│   └── launch/spawn_swarm.launch.py
└── antbot_navigation/           # Swarm intelligence
    ├── antbot_navigation/
    │   ├── swarm_logic.py        # Per-bot state machine (one node per bot)
    │   └── swarm_gui_server.py   # Tkinter control dashboard
    └── launch/swarm.launch.py    # Full launch: Gazebo + 3 bots + GUI
```

---

## 🔄 State Machine (Per Bot)

Each bot runs an independent `SwarmBot` ROS 2 node with four states:

```
┌────────────┐   green blob      ┌──────────────┐   very close     ┌─────────────┐
│   SEARCH   │ ─────────────────►│   APPROACH   │ ────────────────►│   STOPPED   │
│ Lévy walk  │                   │ visual servo │                   │ broadcast   │
└────────────┘                   └──────────────┘                   └─────────────┘
      ▲  blob lost                                                         │
      └──────────────────────────────────────────────────────              │
                                                                           │
      peer broadcast received                                              │
┌─────────────────┐◄─────────────────────────────────────────────────────-┘
│  GO_TO_TARGET   │
│ navigate to XY  │
└─────────────────┘
         │ arrived (dist < 0.32 m)
         └──► STOPPED
```

### Avoidance Priority (applied in every tick)

```
boundary_avoid()  →  ir_avoid()  →  bot_repulsion()  →  state behaviour
```

- **Boundary** — turns away from arena walls at ±0.22 m margin
- **IR obstacle** — dodges cubes detected within 0.20 m (3 sectors: ±10°, ±10–30°)
- **Bot repulsion** — peer odometry-based separation force below 0.30 m

> In `APPROACH` state, IR obstacle avoidance is intentionally bypassed so the bot can drive right up to the green cube.

---

## 📡 ROS 2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/{ns}/cmd_vel` | `geometry_msgs/Twist` | Logic → motors |
| `/{ns}/odom` | `nav_msgs/Odometry` | Gazebo → logic |
| `/{ns}/camera/image_raw` | `sensor_msgs/Image` | Camera → logic (green detection via OpenCV HSV blob) |
| `/{ns}/scan` | `sensor_msgs/LaserScan` | LiDAR → logic (IR sectors) |
| `/{ns}/state` | `std_msgs/String` | Logic → GUI |
| `/green_cube_location` | `geometry_msgs/Point` | Finder bot → peers |
| `/swarm_control` | `std_msgs/String` | GUI → all bots (`START` / `STOP` / `RESET`) |

---

## 🗺️ Arena

The Gazebo world (`arena.world`) is a **2 × 2 m** enclosed arena:

- 4 blue boundary walls at ±1.05 m on both axes, height 0.25 m
- Coloured cube obstacles scattered inside (red, blue, and **one green target**)
- Bots spawn in the bottom-left quadrant: `(-0.75, -0.75)`, `(-0.75, -0.45)`, `(-0.45, -0.75)`

---

## 🖥️ Control Dashboard

`swarm_gui_server.py` launches a Tkinter desktop window alongside Gazebo.

| Control | Action |
|---|---|
| ▶ START SWARM | All bots leave `PAUSED` → `SEARCH` |
| ⏸ STOP SWARM | All bots → `PAUSED` |
| 🔄 RESET ALL | Resets bot states + triggers Gazebo `/reset_world` |

State badges update live: `PAUSED` · `SEARCH` · `APPROACH` · `GO_TO_TARGET` · `STOPPED`

---

## 🚀 Running the Simulation

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (`ros-humble-gazebo-ros-pkgs`)
- Python packages: `opencv-python`, `cv_bridge`, `numpy`

### Build

```bash
cd ~/Antbots
colcon build
source install/setup.bash
```

### Launch (full system — Gazebo + bots + GUI)

```bash
ros2 launch antbot_navigation swarm.launch.py
```

This starts Gazebo with the arena, spawns all three bots after a 5-second delay, launches one `swarm_logic` node per bot, and opens the GUI dashboard.

### Launch (Gazebo only, no logic nodes)

```bash
ros2 launch antbot_gazebo spawn_swarm.launch.py
```

---

## 🧪 Simulation Testing Checklist

- [ ] All three bots spawn without overlap
- [ ] Bots roam and avoid walls + cubes in `SEARCH` state
- [ ] Camera detects green cube → bot transitions `SEARCH → APPROACH`
- [ ] Bot reaches cube → transitions `APPROACH → STOPPED` and publishes `/green_cube_location`
- [ ] Peer bots receive broadcast → transition `SEARCH → GO_TO_TARGET`
- [ ] All three bots converge and reach `STOPPED`
- [ ] GUI badges reflect live state changes
- [ ] RESET returns all bots to start positions and `PAUSED` state

---

## 🔌 Physical Deployment (ESP32)

The simulation maps directly to the hardware layer:

| Simulation | Hardware Equivalent |
|---|---|
| LiDAR scan sectors | 3 × IR proximity sensors |
| OpenCV HSV blob detection | TCS3200 / TCS34725 colour sensor |
| ROS 2 topic `/green_cube_location` | ESP-NOW broadcast packet |
| Odometry-based peer positions | RF distance / RSSI estimation |
| `cmd_vel` Twist | PWM signals to BO motor driver |

### Default ESP32 Pin Reference

| GPIO | Function |
|---|---|
| 25 | IR Sensor — Left |
| 26 | IR Sensor — Centre |
| 27 | IR Sensor — Right |
| 18 | Motor A — IN1 |
| 19 | Motor A — IN2 |
| 21 | Motor B — IN1 |
| 22 | Motor B — IN2 |
| 32 | Colour Sensor (S0 / S2 / OUT — adjust per module) |
| 33 | Buzzer |

### ESP-NOW Message Structure

```c
typedef struct {
  uint8_t  event;        // 0x01 = GREEN_FOUND
  uint8_t  sender_id;    // 1, 2, or 3
  uint8_t  reserved[6];  // Future: grid position / RSSI hint
} swarm_msg_t;
```

Each bot registers the MAC addresses of its two peers at startup. Messages are broadcast; all peers receive and switch to navigation mode.

---

## 🔭 Future Improvements

- Encoder-based odometry on physical bots for accurate position tracking
- RSSI-based bearing estimation to replace pure heading navigation
- Expand to N bots via dynamic ESP-NOW peer registration
- Add red / blue colour avoidance behaviour (currently only green triggers)
- Web dashboard via ESP32 soft-AP to mirror the Tkinter GUI on mobile
- RViz2 integration for live swarm position visualisation
