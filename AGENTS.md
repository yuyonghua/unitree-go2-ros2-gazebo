# AGENTS.md — go2_ws (Unitree GO2 L1 Gazebo + SLAM/Nav2 repro)

ROS 2 Humble colcon workspace. Namespace `robot1`, DDS `rmw_cyclonedds_cpp`, `use_sim_time:=true` everywhere.
Reference (read-only, runs OK): `/home/user/git/ros2-learning/ROS2-Gazebo-GO2`. Full procedure:
`GO2-L1-复现指南.md` (canonical, keep in sync with real files). Prior session: `handoff-go2-l1-2026-09-04.md`.
Status: P1 description ✅, P2 bringup in testing, `go2_slam`/`go2_nav` are empty dirs (not started — don't jump ahead).

## Env (every terminal)

```bash
source env.sh   # order matters: /opt/ros/humble + install/setup.bash FIRST, then exports
```

- Known drift (verified 2026-09-04): `env.sh` sets `CYCLONEDDS_URI=file://$HOME/git/yyh/go2_ws/cyclonedds.xml`
  but that file does **not** exist — the real copy is `src/cyclonedds.xml`. Fix the path before blaming DDS.
- `GZ_SIM_RESOURCE_PATH` must append (colon) `src/go2_gazebo_bringup/worlds` + `models`, else Fuel download stall.
- All terminals must share the same `RMW_IMPLEMENTATION` or bridge gets no data.

## Packages (`src/`)

- `go2_description` — static model only, never touches Gazebo. Entry `xacro/robot.xacro`
  (includes `gazebo.xacro` + `lidar_external.xacro`); preview via `launch/display.launch.py`.
- `go2_gazebo_bringup` — everything simulation. Entry `launch/launch.py` (world → Gazebo,
  `sleep 6`, then `l1.launch.py`: rsp → spawn → bridge → spawners → quadropted nodes → EKF).
  RViz via `rviz:=true` (default off; config auto-selected: `rviz/go2_l1.rviz` vs `go2_l1_external.rviz`).
- `quadropted_controller` + `quadropted_msgs` — reuse as-is, never rewrite (comments already translated to Chinese).
- `go2_slam` / `go2_nav` — empty, future work (slam_toolbox, not cartographer).

## Build / verify

```bash
colcon build --packages-select go2_description go2_gazebo_bringup && source install/setup.bash
ros2 launch go2_description display.launch.py            # static check, RViz Fixed Frame = base_link
ros2 launch go2_gazebo_bringup launch.py                 # sim; add use_external_lidar:=true for 360° lidar
xacro src/go2_description/xacro/robot.xacro robot_name:=robot1 -o /tmp/go2l1.urdf && check_urdf /tmp/go2l1.urdf
ros2 topic hz /robot1/scan   # ~10Hz, angle_min≈1.39 angle_max≈4.88 (front ~200°, rear blind)
```

- `install/` goes stale after file removals — delete manually, then rebuild.
- `go2_gazebo_bringup/package.xml` has no `exec_depend`s yet; add when touching deps.

## Gotchas (all verified, don't relitigate)

- `spawn -topic` must be `/robot1/robot_description` (rsp is namespaced); wrong topic = world with no robot.
- `xacro.process_file` needs `OpaqueFunction` + `LaunchConfiguration(...).perform(context)` into
  `mappings`; `use_external_lidar` bridge topics must be appended conditionally (Gazebo has no velodyne topics when off).
- `gazebo.xacro` `<ros2_control>` must list all 12 leg joints fully — an empty/abbreviated block makes both spawners `exit 1`.
- Spawners in `l1.launch.py` must be serial (`joint_state_broadcaster` first, `joint_group_controller`
  on its exit) with BOTH `--controller-manager-timeout 60` AND `--service-call-timeout 60`
  (two independent 10s defaults); any timeout causes `already loaded` FATAL or
  loaded-but-unconfigured → missing leg TFs (broadcaster) or dog flips (group controller).
- Upstream Humble bug (verified in `/opt/ros/humble/.../controller_manager/spawner.py`):
  the `load_controller` call never passes `service_call_timeout`, so load always uses a
  hardcoded 10s. Hence spawners are wrapped in `TimerAction(20.0)` — Gazebo's controller_manager
  only answers load ~17s after gz start. Increase the delay on slower machines. If a spawner
  still dies, recover without restarting via `configure_controller` + `switch_controller`
  service calls (see guide §5.4).
- TF remaps `(/tf→tf, /tf_static→tf_static, /scan→scan, /odom→odometry/filtered)` required on rsp/spawners/controller/odom/ekf;
  a missing one = split TF tree or SLAM with no scan. Base frame is `base_link` (EKF/SLAM/Nav2 agree); never use `velodyne` as RViz Fixed Frame.
- Each TF edge must have exactly one publisher: `odom→base_link` only from EKF (`enable_odom_tf:=false`
  on the odom node — upstream has both on, causing RViz flicker); `map→odom` from EITHER slam_toolbox
  OR amcl, never both (don't run `slam.launch.py` together with `nav2.launch.py`).
- L1 ranges: Gazebo `max 131m` penetrates walls — clamp to 10m in SLAM/Nav2 params. `inflation_radius ≥ 0.5` (rear blind).
- `leg.xacro` upstream typo (`fixed">0`) is tolerated — leave it. No `dae/` dir (duplicate of `meshes/`);
  `calf_mirror.dae`/`foot.dae` unreferenced; `Vlp16.dae` only needed when external lidar on.
  `grep -rn velodyne src --include=*.xacro` must hit only `lidar_external.xacro` by default.

## Session conventions

- Work piece-by-piece; don't start slam/nav2 unprompted. Don't run long sims unprompted; if killing sim,
  `pkill` patterns can match your own shell — use the bracket trick (`pkill -f '[g]z sim'`) or exact PIDs.
- New launch/xacro code: heavily commented in Chinese. Only `go2_l1*.rviz` are allowed in repo (bringup exception). Keep `GO2-L1-复现指南.md` in sync (paste full file contents, never "copy omitted lines").
