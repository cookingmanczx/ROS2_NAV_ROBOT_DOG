# Unitree A2 Gazebo 仿真：SLAM 建图与 Nav2 导航

基于 **ROS 2 Humble + Gazebo Classic 11** 的四足机器狗（Unitree A2）仿真工作区，包含激光雷达建图（slam_toolbox）与自主导航（Nav2）的完整流程。

## 功能

- Unitree A2 四足模型（官方 URDF + STL 网格），站立姿态由 `joint_defaults.yaml` 固定
- Gazebo Classic 场景 `small_house`，机身搭载 2D 激光雷达（`/scan`）
- 静态模型 + `cmd_vel_driver` 方案：将 `/cmd_vel` 积分后通过 `SetEntityState` 移动模型，只发布里程计与 TF，避免四足动力学调参
- slam_toolbox 在线建图，可保存栅格地图
- Nav2 导航（map_server + AMCL + NavFn + DWB），目标点合法性校验（`goal_guard`）
- 启动时用当前里程计位姿初始化 AMCL，避免定位起点错位

## 包结构

| 包 | 类型 | 说明 |
| --- | --- | --- |
| `a2_description` | ament_cmake | A2 的 URDF 与 STL 网格 |
| `a2_gazebo_sim` | ament_cmake | Gazebo 场景、雷达 xacro、launch、Nav2/slam 参数、地图、驱动脚本 |
| `mybot` | ament_python | 教学用的最小 Python 节点示例（`circle_mover`） |

`a2_gazebo_sim` 主要目录：

```
launch/   sim / slam / nav / bringup 四组 launch
config/   nav2.yaml、slam.yaml、slam.rviz、joint_defaults.yaml
maps/     room1、my_room、test_auto 三张已建好的地图（.pgm + .yaml）
scripts/  cmd_vel_driver.py、goal_guard.py、initial_pose_from_odom.py
urdf/     a2_lidar.urdf.xacro（在官方模型上追加雷达）
worlds/   small_house.world
```

## 环境依赖

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox \
                 ros-humble-nav2-bringup ros-humble-xacro \
                 ros-humble-joint-state-publisher ros-humble-teleop-twist-keyboard
```

## 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

**1. 启动仿真**

```bash
ros2 launch a2_gazebo_sim sim.launch.py world:=small_house
# 可选参数：x:=0.0 y:=0.0 yaw:=0.0 gui:=false
```

手动控制（另一个终端）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**2. 建图（slam_toolbox）**

```bash
ros2 launch a2_gazebo_sim slam.launch.py
# 建完后保存地图
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/a2_gazebo_sim/maps/my_room
```

**3. 导航（Nav2）**

```bash
ros2 launch a2_gazebo_sim nav.launch.py map:=$HOME/ros2_ws/src/a2_gazebo_sim/maps/room1.yaml
```

RViz 中用 `2D Pose Estimate` 校正初始位姿，再用 `Nav2 Goal` 下发目标点。

**4. 一键启动（仿真 + 导航）**

```bash
ros2 launch a2_gazebo_sim bringup.launch.py world:=small_house
```

`bringup` 会延迟 `nav_delay` 秒（默认 12s）再启动 Nav2，等待 TF 与 `/odom` 就绪。

## 实现要点

- **运动链**：`nav2 → /cmd_vel → cmd_vel_driver → SetEntityState → Gazebo`。模型是静态的，位姿由驱动节点积分得到，`/odom` 与 `map→odom→base_link` TF 由该节点发布。
- **goal_guard**：接管 `navigate_to_pose` 动作，转发到重映射后的 `navigate_to_pose_impl`。目标点越界、落在未知区域、占用栅格，或离障碍物小于 `min_clearance`（默认 0.30 m）时直接 abort，避免 NavFn 容差导致“到了但没真正到达”。
- **initial_pose_from_odom**：启动时把 AMCL 初始位姿对齐到当前里程计，避免硬编码原点造成的整张地图错位。
- **雷达**：`a2_lidar.urdf.xacro` 在 A2 机身上追加 `ray` 传感器；`sim.launch.py` 会把 `package://` 网格 URI 改写为 `file://`，否则运行时 spawn 的模型在 gzclient 里不显示。

## 注意事项

- `sim` 与 `slam`/`nav` 必须分终端启动，且同一时间只运行其中一个定位源：建图时 AMCL 必须关闭，导航时 slam_toolbox 必须关闭，否则 `map→odom` 会冲突。
- 仿真中请保持 `use_sim_time:=true`。

## License

BSD-3-Clause（`a2_description`、`a2_gazebo_sim`）；`mybot` 为 Apache-2.0。
