# HowYouSeeMe

A ROS 2 perception and spatial memory system that builds a live semantic 3D map of any environment, enriches it with AI vision models, and exposes the robot's world model to any LLM via MCP.

[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Kinect](https://img.shields.io/badge/Kinect-v2-orange)](docs/Kinect2_ROS2_Bridge_Setup.md)
[![CUDA](https://img.shields.io/badge/CUDA-12.6+-green)](https://developer.nvidia.com/cuda-toolkit)
[![Models](https://img.shields.io/badge/AI_Models-5-purple)](docs/CV_PIPELINE_V2_GUIDE.md)

---

## What it does

HowYouSeeMe is built around a continuous perception loop. A Microsoft Kinect v2 streams registered RGB-D frames at 14.5 Hz while a BlueLily IMU (MPU6500) supplies 800 Hz inertial data. Both streams feed ORB-SLAM3, which tracks the camera pose in real time and incrementally builds a dense 3D map of the environment. Every frame is simultaneously passed through YOLO11, which detects and classifies objects; the resulting bounding boxes are back-projected into the SLAM coordinate frame to produce a semantically annotated point cloud. The entire state of the visible world — object labels, 3D positions, detection confidences, timestamps — is serialised into a rolling `world_state.json` that always reflects what the robot can currently see.

Significant perception events (a new person entering the frame, a known object moving, an unusual detection) are written as checkpoints to a short-term memory store on disk. A pool of async workers then enriches each checkpoint with higher-cost models: SAM2 tiny for precise instance masks, InsightFace for face identification across sessions, and FER for emotion estimation. None of these models are kept loaded continuously; they are instantiated, run, and torn down per checkpoint to stay within the 4 GB VRAM budget. Named memories — user-pinned objects like "my keys" or "the charging dock" — are written to a persistent JSON store that survives reboots and accumulates over the robot's lifetime.

The robot's world knowledge is exposed to any LLM through a Model Context Protocol server running on port 8090. Tools like `query_world`, `where_is`, `navigate_to`, and `get_camera_frame` give an LLM structured, grounded access to real-time and historical spatial data. Ally, the human-facing voice and chat interface, connects to this MCP endpoint in Robot Mode and can issue commands — "find the 3D printer", "remember where the apple is", "what can you see?" — that resolve to ROS 2 actions on the robot.

The navigation stack takes a semantic goal (a natural-language description, a named memory, or exact 3D coordinates) and resolves it into a path using an RRT* global planner over the NVBLOX ESDF and a DWA local planner for real-time obstacle avoidance. If the target object has never been seen, the robot enters visual search mode: it sweeps the room while scoring each frame with YOLO confidence and CLIP similarity against a visual description of the target, then back-projects the first confident detection to 3D and pathplans precisely to it. Between active sessions, a sleep-time pipeline runs OpenSplat on accumulated keyframes to produce a Gaussian splat that replaces the sparser TSDF background on the next wakeup, improving map quality passively over time.

---

## Hardware

| Component       | Part                        | Interface             |
|-----------------|-----------------------------|-----------------------|
| RGB-D camera    | Microsoft Kinect v2         | USB 3.0               |
| IMU             | BlueLily (MPU6500)          | /dev/ttyACM0 115200   |
| Compute (dev)   | RTX 3050 laptop, Ubuntu 22  | CUDA 12.x             |
| Compute (robot) | Jetson AGX Orin 64GB        | JetPack 6 `[PLANNED]` |

---

## System Architecture

### The 5 tiers

**Tier 1 — Always on**

ORB-SLAM3 (pose + map), YOLO11 (object detection), event checkpointer.
These three nodes run continuously. Nothing in this tier is ever killed.
VRAM cost: ~1 GB. CPU: 3–4 cores.

**Tier 2 — Event-driven**

Async frame analyser, SAM2 (precise masks), InsightFace (face ID), FER (emotion).
Spawned when triggered by Tier 1 events, killed when idle.
Only run when a checkpoint justifies the compute cost.

**Tier 3 — World synthesis**

World synthesiser (merges all perception into `world_state.json` every 3 s).
Named memory store (persistent object location tracking).

**Tier 4 — Persistence**

```
/tmp/stm/                                        short-term checkpoint memory
/tmp/world_state.json                            live world model
~/howyouseeme_persistent/named_memories.json     survives reboots
```

**Tier 5 — Interface**

- MCP server (port 8090) — LLM tool interface
- RViz semantic overlay — live visualisation
- Rerun logger — timeline recording and session replay
- Ally integration — voice + chat interface via Robot Mode

### Navigation stack `[PLANNED/WIP]`

```
Semantic goal resolver → RRT* global planner → DWA local planner → /cmd_vel
Visual search mode for unknown objects (YOLO + CLIP + optional web image fetch)
```

### Sleep-time map enhancement `[PLANNED]`

```
Keyframe exporter → sleep detector → OpenSplat Gaussian splat → splat loader
Runs offline while robot is idle. Improves map quality each session.
```

---

## Key ROS 2 Topics

| Topic                          | Type             | Publisher            | Hz     |
|-------------------------------|------------------|----------------------|--------|
| /kinect2/hd/image_color        | Image            | kinect2_bridge       | 14.5   |
| /kinect2/hd/image_depth_rect   | Image            | kinect2_bridge       | 14.5   |
| /imu/data                      | Imu              | bluelily_bridge      | 800    |
| /orb_slam3/pose                | PoseStamped      | orb_slam3            | 14.5   |
| /tsdf/pointcloud               | PointCloud2      | tsdf_integrator      | 1      |
| /splat/background              | PointCloud2      | splat_loader         | 0.1    |
| /cv_pipeline/results           | String (JSON)    | yolo_node            | 14.5   |
| /semantic/markers              | MarkerArray      | semantic_projection  | 0.3    |
| /semantic/world_state          | String (JSON)    | world_synthesiser    | 0.3    |
| /memory/checkpoint_saved       | String           | event_checkpointer   | event  |
| /memory/updated                | String           | named_memory_node    | event  |
| /robot/sleeping                | Bool             | sleep_detector       | event  |
| /navigation/status             | String (JSON)    | navigation_manager   | 1      |
| /navigation/path               | Path             | rrt_star_planner     | event  |
| /cmd_vel                       | Twist            | dwa_local_planner    | 10     |

---

## MCP Tools

All tools exposed at `http://localhost:8090/mcp`

| Tool                  | Description                                                        |
|-----------------------|--------------------------------------------------------------------|
| query_world           | Returns full world state — objects, people, events                 |
| where_is(label)       | Last known 3D position of any object or person                     |
| remember_object       | Pin an object for persistent tracking                              |
| recall_memory         | Get a pinned object's current confirmed location                   |
| forget_memory         | Stop tracking a named memory                                       |
| get_recent_events     | Last N perception events with timestamps                           |
| get_checkpoint        | Full metadata for a specific checkpoint event                      |
| get_camera_frame      | Latest RGB frame as base64 JPEG for vision LLMs                    |
| get_robot_status      | Natural language summary of robot state                            |
| get_robot_context     | System prompt context block for Ally Robot Mode                    |
| navigate_to(query)    | Pathfind to object/room described in natural language              |
| navigate_to_pose      | Pathfind to exact 3D coordinates                                   |
| cancel_navigation     | Stop robot immediately                                             |
| navigation_status     | Current nav state, goal, progress, ETA                             |

---

## AI Models

| Model        | Purpose                    | Tier | VRAM    | When active   |
|--------------|----------------------------|------|---------|---------------|
| YOLO11       | Object detection           | 1    | ~400 MB | Always        |
| SAM2 tiny    | Precise segmentation       | 2    | ~280 MB | On event      |
| InsightFace  | Face identification        | 2    | ~200 MB | Person event  |
| FER          | Emotion detection          | 2    | ~150 MB | Person event  |
| FastSAM      | Fast segmentation          | 2    | ~200 MB | On demand     |
| CLIP         | Visual search matching     | nav  | ~200 MB | Visual search |
| TensorRT     | YOLO acceleration          | opt  | —       | If exported   |

Total peak (all active): ~1.43 GB VRAM — fits on 4 GB with headroom.

---

## Navigation `[PLANNED/WIP]`

### Goal types

- **Named memory**: "my keys" → direct pathplan to pinned location
- **Known object**: "cup" → last seen position from `world_state.json`
- **Room label**: "kitchen" → SLAM map room centroid
- **Unknown object**: "3D printer" → visual search mode

### Visual search mode

For objects never seen before:
1. LLM generates visual description of target
2. Optionally fetch reference image from web (SerpAPI or LLM-suggested URL)
3. Robot sweeps room — rotate in place, then expanding spiral
4. Each frame scored: YOLO confidence + CLIP similarity vs description
5. On detection: back-project to 3D, save named memory, pathplan precisely

### Planners

- **Global**: RRT* on NVBLOX ESDF (falls back to A* on 2D occupancy if no ESDF)
- **Local**: DWA — samples velocity commands, scores by heading + clearance + speed
- **Replan**: triggered when obstacle appears within 0.15 m of planned path

---

## Sleep-time Map Enhancement `[PLANNED]`

While the robot is idle (no sensor activity for 60 s):
1. Keyframe exporter has been saving RGB frames + ORB-SLAM3 poses to disk
2. Sleep detector fires, launches OpenSplat on accumulated keyframes
3. OpenSplat runs overnight at `--resolution 2 --iters 5000` on robot GPU
4. Finished `.ply` splat loaded as `/splat/background` on next wakeup
5. Splat improves progressively — better map quality each sleep cycle
6. RViz layers: splat base → live TSDF → semantic markers → robot pose

---

## Ally Integration `[PLANNED/WIP]`

Ally (https://github.com/[your-ally-repo]) is the human-facing interface.
HowYouSeeMe registers as an MCP server in Ally's Robot Mode.

**What Ally can do when connected:**

> Voice: "What can you see right now?"
> → `query_world` → describes all detected objects with 3D positions

> Voice: "Go find the 3D printer"
> → `navigate_to("3D printer")` → visual search → pathplan → arrive

> Voice: "Remember where the apple is"
> → `remember_object` → YOLO watches for apple → pins location on detection

> Voice: "Show me what you see"
> → `get_camera_frame` → base64 JPEG → vision LLM describes scene

Connection: `http://localhost:8090/mcp`
Robot Mode toggle in Ally automatically injects robot status as system context.

---

## Visualisation

### RViz (live operation)

Displays: TSDF point cloud, splat background, semantic labels, robot pose, navigation path, goal markers.

### Rerun (debugging + session replay)

Records all streams with timestamps. Scrub back to any moment and see exactly what the robot perceived — RGB, depth, YOLO detections, pose, world state — all synced.

```bash
pip install rerun-sdk --break-system-packages
# Sessions saved to ~/howyouseeme_sessions/session_<date>.rrd
rerun ~/howyouseeme_sessions/session_latest.rrd
```

---

## Compute Targets

| Platform              | Status      | Notes                                                                   |
|-----------------------|-------------|-------------------------------------------------------------------------|
| RTX 3050 laptop       | `[DONE]`    | Development platform, 4 GB VRAM ceiling                                 |
| Jetson AGX Orin 64GB  | `[PLANNED]` | Target robot platform, 64 GB unified memory                             |
| Isaac ROS migration   | `[PLANNED]` | cuVSLAM + NVBLOX replace current SLAM/TSDF. Same codebase, one CMake flag change |

---

## Phase Implementation Status

| Feature                                   | Status      | Notes                                                         |
|-------------------------------------------|-------------|---------------------------------------------------------------|
| Phase 1 — IMU-camera calibration (Kalibr) | `[WIP]`     | IMU disabled for now, identity transform placeholder          |
| Phase 2 — ORB-SLAM3 RGB-D                 | `[DONE]`    | Running without IMU fusion, visual-only mode                  |
| Phase 3 — Open3D TSDF integrator node     | `[DONE]`    | `tsdf_integrator_node.py`, 1 Hz map update on CPU             |
| Phase 4 — Semantic projection node        | `[DONE]`    | YOLO detections back-projected to 3D; floating labels in RViz via `/semantic/markers` |
| Tier 1–5 memory system                    | `[WIP]`     | Event checkpointer done; async analyser in progress; world synthesiser in progress; named memory store planned; MCP server planned |
| Sleep-time Gaussian splat pipeline        | `[PLANNED]` | Keyframe exporter, sleep detector, OpenSplat launcher, splat loader all designed, not yet implemented |
| Navigation stack                          | `[PLANNED]` | RRT*, DWA, semantic resolver, visual search all designed      |
| Ally integration                          | `[PLANNED]` | MCP server must be running first                              |
| Rerun visualisation                       | `[WIP]`     | Logger node written, not yet in launch file                   |
| Isaac ROS migration                       | `[PLANNED]` | Pending Jetson AGX Orin hardware purchase                     |

---

## Quick Start

```bash
# Dependencies
sudo apt install ros-humble-desktop ros-humble-cv-bridge
pip install open3d ultralytics rerun-sdk --break-system-packages

# Build
cd ros2_ws && colcon build
source install/setup.bash

# Launch full stack
ros2 launch kinect2_slam howyouseeme_full.launch.py

# MCP server (separate terminal)
python3 ros2_ws/src/kinect2_slam/python/mcp_server.py

# Visualisation (separate terminal)
ros2 launch kinect2_slam rviz.launch.py
# or for Rerun:
rerun  # viewer opens automatically when logger node starts
```

---

## Repo Structure

```
HowYouSeeMe/
├── ros2_ws/src/
│   ├── kinect2_ros2_cuda/        Kinect v2 ROS 2 bridge — do not modify
│   ├── bluelily_bridge/          IMU serial bridge
│   │   └── python/
│   │       └── bluelily_imu_node.py
│   ├── cv_pipeline/              AI model nodes
│   │   └── python/
│   │       ├── yolo_node.py
│   │       ├── sam2_service_node.py
│   │       ├── insightface_service_node.py
│   │       ├── semantic_projection_node.py
│   │       └── visual_search_node.py
│   └── kinect2_slam/             Core SLAM and memory system
│       ├── launch/
│       │   └── howyouseeme_full.launch.py
│       └── python/
│           ├── tsdf_integrator_node.py
│           ├── event_checkpointer_node.py
│           ├── async_analyser_node.py
│           ├── world_synthesiser_node.py
│           ├── named_memory_node.py
│           ├── keyframe_exporter_node.py
│           ├── sleep_detector_node.py
│           ├── splat_loader_node.py
│           ├── rerun_logger_node.py
│           ├── navigation_manager_node.py
│           ├── rrt_star_planner.py
│           ├── dwa_local_planner.py
│           ├── semantic_goal_resolver.py
│           └── web_image_fetcher.py
├── scripts/
│   ├── run_opensplat.sh
│   └── merge_splats.py
├── BlueLily/                     IMU firmware — do not modify
├── mcp_server.py                 MCP HTTP server on :8090
└── README.md
```

---

## Related Projects

- **Ally** — https://github.com/[your-ally-repo]
  Desktop AI overlay, Robot Mode connects to HowYouSeeMe via MCP

- **DroidCore** — [link if public]
  Full robot control stack that HowYouSeeMe feeds into

---

## Research Context

This system implements concepts from:

- **ConceptFusion (2023)** — language-embedded 3D maps
- **SayPlan (2023)** — LLM task planning via 3D scene graphs
- **HomeRobot (Meta, 2023)** — open vocabulary mobile manipulation
- **OpenScene / LERF** — queryable neural scene representations

The specific combination of modular on-demand model loading, sleep-time Gaussian splat densification, MCP as the LLM-robot interface standard, and persistent named spatial memory as an open deployable system is the novel contribution of this project.
