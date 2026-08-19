# unitree_indoor_slam

Indoor 2D/3D LiDAR SLAM for the Unitree Go2 simulation (ROS 2 Jazzy + Gazebo
Harmonic). The package ships **launch files, tuned parameter sets and the maps
recorded with them** for three interchangeable SLAM back-ends:

| Back-end | Launch file | Input | Output | Notes |
|---|---|---|---|---|
| **slam_toolbox** (recommended) | `unitree_slam_toolbox.launch.py` | `/scan` (2D) | `/map` + `map→odom` TF | Ceres pose-graph, loop closure, built-in map saver |
| **gmapping** (RBPF particle filter) | `unitree_gmapping.launch.py` | `/scan` (2D) | `/map` + `map→odom` TF | Needs the external [`ros2_gmapping`](https://github.com/khaledgabr77/ros2_gmapping) repo |
| **RTAB-Map** (3D LiDAR) | `unitree_rtabmap.launch.py` | `/velodyne_points/points` (3D) + `/odom` | `/map` + `map→odom` TF + `rtabmap.db` | ICP registration, forced 3-DoF, projects a 2D grid |

This package **does not** start Gazebo, the robot or the controllers — it only
consumes topics/TF published by `unitree_go2_sim`. Start the simulation first,
then start one SLAM back-end.

---

## Table of contents

1. [Package layout](#1-package-layout)
2. [How it works](#2-how-it-works)
3. [Dependencies and build](#3-dependencies-and-build)
4. [Step 1 — start the simulation](#4-step-1--start-the-simulation)
5. [Step 2 — pre-flight checks (before you move the robot)](#5-step-2--pre-flight-checks-before-you-move-the-robot)
6. [Step 3 — start a SLAM back-end](#6-step-3--start-a-slam-back-end)
7. [Step 4 — drive the robot and record the map](#7-step-4--drive-the-robot-and-record-the-map)
8. [Step 5 — save the map](#8-step-5--save-the-map)
9. [Re-using a saved map](#9-re-using-a-saved-map)
10. [Parameter reference](#10-parameter-reference)
11. [Troubleshooting](#11-troubleshooting)
12. [Known limitations](#12-known-limitations)

---

## 1. Package layout

```
unitree_indoor_slam/
├── launch/
│   ├── unitree_slam_toolbox.launch.py   # pointcloud_to_laserscan + slam_toolbox (lifecycle)
│   ├── unitree_gmapping.launch.py       # pointcloud_to_laserscan + gmapper/gmap
│   └── unitree_rtabmap.launch.py        # rtabmap_slam (LiDAR-only, external odometry)
├── config/
│   ├── slam_toolbox_params.yaml         # async mapper, Ceres solver, indoor tuning
│   └── gmapping_params.yaml             # RBPF particles, motion model, grid extents
├── maps/
│   ├── cave_indoor_map_slamtoolbox.{yaml,pgm}   # cave_world.sdf, recorded with slam_toolbox
│   └── cave_indoor_map_gmapping.{yaml,pgm}      # cave_world.sdf, recorded with gmapping
├── rviz/                                 # (reserved for indoor_slam.rviz — see §12)
├── CMakeLists.txt
└── package.xml
```

RTAB-Map parameters are declared inline in `unitree_rtabmap.launch.py` (no YAML
file), because most of them are RTAB-Map "Key/Value" strings rather than typed
ROS parameters.

---

## 2. How it works

### 2.1 Data flow

The Go2 carries a simulated 16-beam Velodyne VLP-16 (`gpu_lidar` sensor,
`unitree_go2_description/urdf/velodyne.xacro`). Gazebo publishes it on the gz
topic `velodyne_points`, and `ros_gz_bridge` (started by `champ_gazebo`)
exposes it to ROS 2:

```
Gazebo gpu_lidar
      │  gz topic: velodyne_points
      ▼
ros_gz_bridge ──▶ /velodyne_points/points   (sensor_msgs/PointCloud2, 10 Hz, frame_id = velodyne)
                          │
        ┌─────────────────┴──────────────────────────────┐
        │ 2D back-ends                                    │ 3D back-end
        ▼                                                 ▼
pointcloud_to_laserscan_node                          rtabmap (scan_cloud)
 (slice −0.2…1.5 m, 360°, 0.5°)                       + /odom as external odometry
        │                                                 │
        ▼                                                 │
     /scan  (sensor_msgs/LaserScan, frame_id = velodyne)   │
        │                                                 │
        ▼                                                 ▼
slam_toolbox / gmapper ─────────────▶ /map  (nav_msgs/OccupancyGrid) + TF map→odom
```

`pointcloud_to_laserscan` is launched **inside** the slam_toolbox and gmapping
launch files, so you never have to start it separately. RTAB-Map consumes the
raw 3D cloud and does not need it.

### 2.2 TF tree

```
map ──────────────▶ odom ──────────────▶ base_link ──▶ velodyne_base_link ──▶ velodyne
 │                   │                       │
 │                   │                       └── robot_state_publisher (URDF, static)
 │                   └── Gazebo gz-sim-odometry-publisher-system,
 │                       bridged as /odom_tf and remapped to /tf by champ_gazebo
 └── published by the SLAM node you launch (slam_toolbox / gmapper / rtabmap)
```

Important consequences:

* **The SLAM node owns `map→odom` only.** If two SLAM back-ends run at the same
  time they will fight over that transform — run exactly one.
* **`odom→base_link` comes from the Gazebo odometry plugin**, i.e. it is
  essentially ground truth (see [§12](#12-known-limitations)). CHAMP's
  `state_estimation_node` publishes its own leg odometry on `/odom/raw`, and the
  two `robot_localization` EKFs in `champ_bringup` are currently commented out,
  so nothing else competes for `odom→base_link`.
* All three back-ends are configured with `base_frame: base_link`,
  `odom_frame: odom`, `map_frame: map`. There is no `base_footprint` in this
  chain — do not set it in the configs.

### 2.3 Topics

| Topic | Type | Direction | Produced by |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | in | Gazebo bridge (needed by `use_sim_time`) |
| `/velodyne_points/points` | `sensor_msgs/PointCloud2` | in | Gazebo bridge |
| `/odom` | `nav_msgs/Odometry` | in (RTAB-Map only) | Gazebo odometry plugin |
| `/scan` | `sensor_msgs/LaserScan` | internal | `pointcloud_to_laserscan` |
| `/map` | `nav_msgs/OccupancyGrid` | out | the SLAM node |
| `/map_metadata` | `nav_msgs/MapMetaData` | out | the SLAM node |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | in/out | RSP, Gazebo bridge, SLAM node |
| `/cmd_vel` | `geometry_msgs/Twist` | out (you) | teleop → `champ_base` |

---

## 3. Dependencies and build

### 3.1 System packages

```bash
sudo apt update
sudo apt install \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools
```

`ros-jazzy-rtabmap-ros` is only needed for `unitree_rtabmap.launch.py`; the
other two back-ends work without it.

### 3.2 gmapping (optional, only for `unitree_gmapping.launch.py`)

The `gmapper` / `openslam_gmapping` packages are **not** part of this repository
and are not available from apt. Clone them into the same workspace:

```bash
cd ~/unitree_ws/src
git clone https://github.com/khaledgabr77/ros2_gmapping.git
```

Without this you will get `Package 'gmapper' not found` when launching the
gmapping variant.

### 3.3 Build

```bash
cd ~/unitree_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` is convenient here: edits to the YAML files under
`config/` and to the maps take effect without rebuilding.

---

## 4. Step 1 — start the simulation

```bash
source ~/unitree_ws/install/setup.bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

Defaults that matter for SLAM:

| Argument | Default | Meaning |
|---|---|---|
| `world` | `unitree_go2_description/worlds/cave_world.sdf` | the indoor cave/tunnel world the shipped maps were recorded in |
| `world_init_x/y/z` | `-30.0 / 0.0 / 0.35` | spawn just outside the cave opening; `z=0.35` matches the crouched stand pose so the dog does not flip on spawn |
| `use_sim_time` | `true` | everything downstream must match this |
| `rviz` | `false` | set `rviz:=true` for the sim's own RViz config |
| `gui` | `true` | set `gui:=false` for headless mapping runs |

Wait until the controllers are loaded and the robot is standing before starting
SLAM. Typical console evidence: `joint_states_controller` and
`joint_group_effort_controller` spawners exiting with success.

---

## 5. Step 2 — pre-flight checks (before you move the robot)

Do **all** of these in a fresh terminal. Skipping them is the most common cause
of a smeared or empty map.

**1. The clock is running** (otherwise every node with `use_sim_time:=true`
blocks forever):

```bash
ros2 topic hz /clock
```

**2. The LiDAR is publishing at ~10 Hz:**

```bash
ros2 topic hz /velodyne_points/points
ros2 topic echo /velodyne_points/points --field header.frame_id --once   # -> velodyne
```

If it is silent, your GPU cannot run Gazebo's `gpu_lidar`; switch the sensor
type to CPU `ray` in `unitree_go2_description/urdf/velodyne.xacro`.

**3. Odometry and TF are alive and complete:**

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link velodyne
```

Both `tf2_echo` calls must print transforms continuously. A one-shot picture of
the whole tree:

```bash
ros2 run tf2_tools view_frames        # writes frames.pdf in the current directory
```

**4. The robot is standing still and settled.** Let the gait controller
stabilise for a few seconds after spawn. Starting the mapper while the dog is
still bouncing from the spawn drop injects a bad first scan into the graph.

**5. Nothing else is publishing `map→odom`.** No other SLAM node, no AMCL, no
`map_server` + `static_transform_publisher`:

```bash
ros2 topic echo /tf --once | grep -A2 'child_frame_id'
```

**6. After launching SLAM (§6), check `/scan` and `/map` before driving:**

```bash
ros2 topic hz /scan      # ~10 Hz
ros2 topic hz /map       # ~1 Hz (map_update_interval)
```

---

## 6. Step 3 — start a SLAM back-end

Run exactly **one** of the following, in its own terminal, with
`~/unitree_ws/install/setup.bash` sourced.

### 6.1 slam_toolbox (recommended)

```bash
ros2 launch unitree_indoor_slam unitree_slam_toolbox.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | must match the simulation |
| `cloud_topic` | `/velodyne_points/points` | 3D cloud to flatten into `/scan` |
| `target_frame` | `velodyne` | frame the `LaserScan` is expressed in |
| `rviz` | `false` | see [§12](#12-known-limitations) — the RViz config is not shipped yet |

What it starts:

* `pointcloud_to_laserscan_node` → `/scan`
* `slam_toolbox/async_slam_toolbox_node` as a **lifecycle node**. It is brought
  to `active` automatically (`autostart: true` in
  `config/slam_toolbox_params.yaml`, plus configure/activate events in the
  launch file). You do not need `ros2 lifecycle set ...`.

Verify it is active:

```bash
ros2 lifecycle get /slam_toolbox        # -> active [3]
```

### 6.2 gmapping

Requires [§3.2](#32-gmapping-optional-only-for-unitree_gmappinglaunchpy).

```bash
ros2 launch unitree_indoor_slam unitree_gmapping.launch.py
```

Same four arguments as slam_toolbox. It starts `pointcloud_to_laserscan` plus
`gmapper/gmap` (node name `slam_gmapping`), loading
`config/gmapping_params.yaml`. `use_sim_time` from the launch argument
overrides the value inside the YAML.

gmapping is a particle filter: it is heavier (50 particles) and produces a
noisier grid than slam_toolbox, and it has no loop closure. It is kept as a
reference/comparison implementation.

### 6.3 RTAB-Map (3D)

```bash
ros2 launch unitree_indoor_slam unitree_rtabmap.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | must match the simulation |
| `cloud_topic` | `/velodyne_points/points` | 3D scan cloud (`scan_cloud`) |
| `odom_topic` | `/odom` | external odometry — RTAB-Map does **not** compute LiDAR odometry here |
| `frame_id` | `base_link` | robot base frame |
| `odom_frame_id` | `odom` | odometry frame |
| `delete_db_on_start` | `true` | start a fresh map; set `false` to append to the existing DB |
| `rviz` | `false` | starts `rtabmap_viz` (RTAB-Map's own GUI, not RViz) |

Configuration highlights (all inline in the launch file):
`Reg/Strategy=1` (ICP), `Reg/Force3DoF=true` (planar indoor motion),
`Icp/PointToPlane=true` with a 0.1 m voxel, `Grid/3D=false` +
`Grid/CellSize=0.05` so a 2D occupancy grid is published on `/map`,
`Optimizer/Strategy=1` (g2o) for graph optimisation.

The database is written to RTAB-Map's default location, `~/.ros/rtabmap.db`.
**With `delete_db_on_start:=true` (the default) that file is deleted on every
launch** — copy it elsewhere before relaunching if you want to keep it.

---

## 7. Step 4 — drive the robot and record the map

In another terminal:

```bash
source ~/unitree_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`teleop_twist_keyboard` publishes `geometry_msgs/Twist` on `/cmd_vel`, which is
what `champ_base` consumes (`champ_bringup` remaps `/cmd_vel/smooth → /cmd_vel`
internally). Keys: `i` forward, `,` backward, `j`/`l` rotate, `k` stop,
`q`/`z` change speed.

Mapping technique that gives clean grids in the cave world:

1. **Reduce speed first.** Press `z` a few times until linear speed is
   ~0.2–0.3 m/s. The gait makes the body pitch and roll, and the `LaserScan` is
   taken in the (tilted) `velodyne` frame — fast walking smears walls.
2. **Drive in straight segments, stop, then turn.** Turning while translating is
   the worst case for the scan matcher.
3. **Pause 1–2 s after each turn** so the scan buffer refills at the new
   heading.
4. **Keep walls in view.** In wide-open parts of the cave the matcher has
   little geometry to lock onto; hug one side.
5. **Close the loop.** Return along a corridor you already mapped and revisit
   the start area — slam_toolbox and RTAB-Map will optimise the graph and pull
   the drift out. gmapping will not.
6. **Watch `/map` live** while driving:

   ```bash
   rviz2
   # Fixed Frame: map ; add displays: Map (/map), LaserScan (/scan), TF, RobotModel
   ```

   If walls start doubling, stop, back up over known territory, and let the
   matcher re-lock before continuing.

---

## 8. Step 5 — save the map

Save **before** shutting anything down: `/map` only exists while the SLAM node
runs.

### 8.1 slam_toolbox — built-in saver (preferred)

`use_map_saver: true` is set in `config/slam_toolbox_params.yaml`, so the node
exposes its own services:

```bash
# Occupancy grid -> <name>.pgm + <name>.yaml, written to the CWD of the node
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'cave_indoor_map_slamtoolbox'}}"
```

To also keep the **pose graph** (so mapping can be resumed later, or used in
slam_toolbox's `localization` mode instead of AMCL):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/$USER/unitree_ws/src/unitree_go2_ros2/unitree_indoor_slam/maps/cave_indoor_map_slamtoolbox'}"
```

That produces `cave_indoor_map_slamtoolbox.posegraph` and `.data`.

### 8.2 Any back-end — `nav2_map_server`

Works for slam_toolbox, gmapping and RTAB-Map alike (they all publish
`nav_msgs/OccupancyGrid` on `/map`). Run it **from the directory you want the
files in**:

```bash
cd ~/unitree_ws/src/unitree_go2_ros2/unitree_indoor_slam/maps

ros2 run nav2_map_server map_saver_cli \
  -f cave_indoor_map_slamtoolbox \
  -t /map \
  --occ 0.65 --free 0.196 \
  --ros-args -p use_sim_time:=true
```

This writes `<name>.pgm` (the image) and `<name>.yaml` (metadata). Use
`--fmt pgm|png` to change the image format.

### 8.3 What the `.yaml` contains

```yaml
image: cave_indoor_map_slamtoolbox.pgm   # MUST match the .pgm file name next to it
mode: trinary
resolution: 0.050                        # m / cell
origin: [-39.694, -34.301, 0]            # world coords of the image's bottom-left pixel
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

If you rename the `.pgm`, rename it inside the `.yaml` too — `map_server` looks
the image up by that field, relative to the `.yaml`'s directory.

### 8.4 Committing a new map

Maps live in `maps/` and are installed to
`share/unitree_indoor_slam/maps` by `CMakeLists.txt`. Naming convention:
`<world>_<backend>.{yaml,pgm}`, e.g. `cave_indoor_map_gmapping.pgm`. Keep the
pair together and rebuild (`colcon build --symlink-install`) so the installed
share directory picks up new files.

---

## 9. Re-using a saved map

Any consumer that takes a map YAML can use these files. Resolve the installed
path with:

```bash
ros2 pkg prefix unitree_indoor_slam
# -> /home/<user>/unitree_ws/install/unitree_indoor_slam
```

Quick standalone check that a map is loadable:

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$(ros2 pkg prefix unitree_indoor_slam)/share/unitree_indoor_slam/maps/cave_indoor_map_slamtoolbox.yaml \
  -p use_sim_time:=true
# then, in another terminal:
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
ros2 topic echo /map --field info --once
```

The Nav2 bringup in `unitree_indoor_nav2` defaults to
`cave_indoor_map_slamtoolbox.yaml` from this package.

---

## 10. Parameter reference

### 10.1 `pointcloud_to_laserscan` (identical in both 2D launch files)

| Parameter | Value | Why |
|---|---|---|
| `target_frame` | `velodyne` | scan is emitted in the LiDAR frame |
| `min_height` / `max_height` | `-0.2` / `1.5` m | vertical slice taken around the sensor |
| `angle_min` / `angle_max` | `±π` | full 360° |
| `angle_increment` | `0.0087` rad (~0.5°) | ~720 beams per scan |
| `scan_time` | `0.1` s | matches the 10 Hz sensor |
| `range_min` / `range_max` | `0.5` / `20.0` m | `0.5` drops self-returns; `20` keeps the grid sane indoors |
| `use_inf` | `true` | out-of-range beams become `+inf` (free space) instead of being dropped |
| `transform_tolerance` | `0.01` s | tight; raise it if you see `Transform failure` warnings |

### 10.2 `config/slam_toolbox_params.yaml` — knobs worth touching

| Parameter | Value | Effect |
|---|---|---|
| `resolution` | `0.05` | grid cell size (m) |
| `max_laser_range` | `20.0` | must be ≥ `range_max` above, else scans get truncated |
| `minimum_travel_distance` / `minimum_travel_heading` | `0.2` / `0.2` | how far the dog must move before a new node is added |
| `scan_buffer_size` | `10` | scans kept for matching; raise in feature-poor areas |
| `do_loop_closing` | `true` | leave on |
| `loop_search_maximum_distance` | `3.0` | how far away a loop closure candidate may be |
| `loop_match_minimum_response_fine` | `0.45` | raise if you get false loop closures |
| `transform_publish_period` | `0.02` | `map→odom` publish rate (50 Hz) |
| `mode` | `mapping` | switch to `localization` to run against a serialized pose graph |

### 10.3 `config/gmapping_params.yaml` — knobs worth touching

| Parameter | Value | Effect |
|---|---|---|
| `particles` | `50` | more particles = more robust, more CPU |
| `delta` | `0.05` | grid resolution (m/cell) |
| `maxUrange` / `maxRange` | `16.0` / `20.0` | usable vs. maximum sensor range |
| `minimumScore` | `50.0` | scan matches below this are rejected; lower it in open areas |
| `linearUpdate` / `angularUpdate` | `0.2` / `0.2` | motion needed before a scan is processed |
| `srr, srt, str, stt` | `0.1, 0.2, 0.1, 0.2` | odometry noise model — lower them, since sim odometry is near-perfect, if the filter looks over-dispersed |
| `xmin…ymax` | `±20.0` | initial grid extent; it grows automatically |
| `map_update_interval` | `1.0` | seconds between full grid regenerations |

---

## 11. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Nothing happens, no `/map`, no logs after startup | `use_sim_time:=true` but `/clock` is not being published | start the simulation first; check `ros2 topic hz /clock` |
| `Package 'gmapper' not found` | `ros2_gmapping` not cloned/built | see [§3.2](#32-gmapping-optional-only-for-unitree_gmappinglaunchpy) |
| `/scan` empty or `Transform failure`/`Could not transform` | TF not ready or `transform_tolerance` too tight | check `tf2_echo base_link velodyne`; raise `transform_tolerance` to `0.1` |
| Map builds but walls double / ghost corridors | driving too fast, or turning while translating | slow down (`z`), stop before turning, revisit mapped areas to trigger loop closure |
| Thin phantom obstacles that follow the robot | the LiDAR sees the dog's own legs; the SLAM launches have **no** self-filter (Nav2's launch does) | keep `range_min: 0.5`, or add a `pcl_ros` CropBox like `unitree_indoor_nav2` does |
| Map drifts and never snaps back | gmapping (no loop closure) or too few particles | use slam_toolbox, or raise `particles` / lower `minimumScore` |
| `map_saver_cli` times out | `/map` not published yet, or `use_sim_time` mismatch | confirm `ros2 topic hz /map`, pass `--ros-args -p use_sim_time:=true` |
| `map_server` fails to load a saved map | `image:` in the `.yaml` does not match the `.pgm` filename | fix the `image:` field ([§8.3](#83-what-the-yaml-contains)) |
| Two conflicting `map→odom` transforms, robot teleports | two SLAM nodes (or SLAM + AMCL) running | run exactly one |
| slam_toolbox stays `unconfigured` | lifecycle transition failed | `ros2 lifecycle get /slam_toolbox`; check the launch log for the configure error |
| RTAB-Map map is empty on restart | `delete_db_on_start:=true` wiped `~/.ros/rtabmap.db` | relaunch with `delete_db_on_start:=false` |

---

## 12. Known limitations

* **`rviz:=true` does not work yet.** Both 2D launch files point at
  `rviz/indoor_slam.rviz`, which is not shipped in this package. Until it is
  added, run `rviz2` manually (Fixed Frame `map`; displays: Map `/map`,
  LaserScan `/scan`, TF, RobotModel). The RTAB-Map launch's `rviz` argument
  starts `rtabmap_viz` instead and is unaffected.
* **Odometry is simulation ground truth.** `odom→base_link` comes from Gazebo's
  `gz-sim-odometry-publisher-system` plugin, and the `robot_localization` EKFs
  in `champ_bringup` are commented out. SLAM therefore runs on nearly
  drift-free odometry — results here are optimistic compared to the real robot,
  where CHAMP's leg odometry (`/odom/raw`) fused with the IMU would be used.
* **No self-filter in the SLAM launches.** `unitree_indoor_nav2` inserts a
  `pcl_ros` CropBox around `base_link` to reject returns from the dog's own
  legs; the SLAM launches rely on `range_min: 0.5` alone.
* **The scan is taken in the tilted `velodyne` frame.** The body pitches and
  rolls with the gait, so the 2D slice is not perfectly horizontal. Driving
  slowly is the mitigation.
* **RTAB-Map output is not committed.** Only slam_toolbox and gmapping maps are
  in `maps/`; RTAB-Map results live in `~/.ros/rtabmap.db` until exported with
  `map_saver_cli`.
* **`gmapper` is not declared in `package.xml`.** It is an out-of-tree
  dependency, so `rosdep install` will not fetch it — clone it manually.
