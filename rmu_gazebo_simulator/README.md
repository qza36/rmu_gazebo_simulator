# rmu_gazebo_simulator

## 1. Introduction

rmu_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmu_gazebo_simulator 提供以下功能：

- rmul_2024, rmuc_2024, rmul_2025, rmuc_2025, rmuc_2026 仿真世界模型

- 网页端局域网联机对战

- 机器人底盘、云台、射击控制

- 地图模型一键导出 PCD 点云（离线，不用起仿真）

| rmul_2024 | rmuc_2024 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

## 2. Quick Start

<del>
### 2.1 ~~Option 1: Docker~~

#### 2.1.1 Setup Environment

- [Docker](https://docs.docker.com/engine/install/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- 允许本地的 Docker 容器访问主机的 X11 显示

    ```bash
    xhost +local:docker
    ```

#### 2.1.2 Create Container

```bash
docker run -it --rm --name rmu_gazebo_simulator \
  --network host \
  --runtime nvidia \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  ghcr.io/smbu-polarbear-robotics-team/rmu_gazebo_simulator:1.0.0
```
</del>

### 2.2 Option 2: Build From Source

#### 2.2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

#### 2.2.2 Create Workspace

```bash
sudo pip install xmacro
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git https://github.com/qza36/rmu_gazebo_simulator.git src/rmu_gazebo_simulator
```

#### 2.2.3 Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.3 Running

启动仿真环境

```sh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

默认会一起启动 **RViz2**（配置：`rviz/visualize.rviz`，Fixed Frame = `odom`，显示 `/livox/lidar`、`/rplidar_a2/scan`、RobotModel、TF）。
不需要 RViz 时加 `use_rviz:=false`；换配置用 `rviz_config_file:=<绝对路径>`。

> [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

#### 2.3.1 Test Commands

控制机器人移动

```sh
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
#根据提示进行输入，支持平移与自旋
```

机器人云台

```sh
ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入，支持绝对角度控制
```

机器人射击

```sh
ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入
```

#### 2.3.2 网页端控制

支持局域网内联机操作，只需要将 localhost 改为主机 ip 即可。

操作手端

<http://localhost:5000/>

```sh
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py
```

裁判系统端

<http://localhost:2350/>

```sh
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_web/main.py
```

#### 2.3.3 切换仿真世界

修改 [gz_world.yaml](./rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`, `rmuc_2026`（默认 `rmuc_2026`）

#### 2.3.4 地图模型转 PCD 点云（离线，不用起仿真）

不启动仿真、也不跑 FAST-LIO 建图，直接把世界模型（`resource/models/<world>/meshes/*.stl`）采样成 PCD 点云地图。
用途：给 small_gicp / NDT 重定位当先验地图，或喂给 pcd2pgm 生成 2D / 2.5D 栅格地图。

依赖：`numpy`（必装）；`open3d` 可选（`--sampler poisson` 泊松盘采样，不装则自动用 numpy 面积均匀采样）。

```bash
# 1) 当前 gz_world.yaml 选中的世界（默认 rmuc_2026）→ resource/maps/rmuc_2026.pcd
ros2 run rmu_gazebo_simulator mesh_to_pcd.py --preview

# 2) 指定世界（直接写世界名，可选: rmul_2024 / rmuc_2024 / rmul_2025 / rmuc_2025 / rmul_2026 / rmuc_2026）
ros2 run rmu_gazebo_simulator mesh_to_pcd.py --world rmul_2026

# 3) 只要墙面（裁掉地面）+ 0.02 m 体素 + 原点对齐机器人出生点
ros2 run rmu_gazebo_simulator mesh_to_pcd.py \
    --world rmul_2026 \
    --z-min 0.1 --voxel 0.02 --align-to-spawn \
    --out ~/maps/rmul_2026.pcd --preview
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `--world` | 世界名（如 `rmul_2026`）或 world.sdf 路径；默认读 `gz_world.yaml` 里选中的世界 |
| `--out` | 输出路径，默认 `resource/maps/<world>.pcd` |
| `--density` | 采样密度（点/m²），默认 1000 |
| `--points` | 直接指定总点数，覆盖 `--density` |
| `--voxel` | 体素降采样尺寸（m），默认 0.02，`0` = 不降采样 |
| `--z-min` / `--z-max` | 按高度裁剪，裁掉地面常用 `--z-min 0.1` |
| `--align-to-spawn` | 把原点平移到 `gz_world.yaml` 里第一台机器人的出生点（对应 `map→odom = -出生点` 的约定） |
| `--offset X Y Z` | 手动平移 |
| `--sampler` | `numpy`（默认）/ `open3d` / `poisson` |
| `--preview` | 额外输出 `*_preview.png`（左=全部点，右=墙面带，黑=占据） |
| `--ascii` | 输出 ascii 格式 pcd（默认二进制） |

输出：`resource/maps/<world>.pcd`（二进制 `x y z` float32）+ 可选的 `*_preview.png`。

> [!NOTE]
> - 点云默认在 **Gazebo 世界坐标系**。用哪种坐标系要和下游 `map` 帧对齐（例如 `nav_bringup` 里 `slam.launch.py` 的 `map→odom` 静态变换）；`--align-to-spawn` 给出的就是"map 原点 = 机器人出生点"那一支。
> - 采样的是**完整表面**（含墙背面、底壳、雷达看不到的面），喂 `pcd2pgm` 前建议先 `--z-min 0.1`。
> - 只支持 `mesh`(stl/obj) / `box` / `cylinder` / `sphere` / `plane`，`.dae` 等格式会跳过并告警。
> - 查看成果：`pcl_viewer resource/maps/rmuc_2026.pcd`。若报 `undefined symbol: libusb_set_option`，是 `LD_LIBRARY_PATH` 里混进了旧版 libusb（如海康 MVS SDK），临时绕过：`env -u LD_LIBRARY_PATH pcl_viewer ...`，或把 MVS 的路径从 `~/.bashrc` / `~/.profile` / `/etc/profile` 里删掉。

## 配套导航仿真仓库

- 2025 SMBU PolarBear Sentry Navigation

    [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git)

    ![cmu_nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmu_gazebo_simulator is provided under Apache License 2.0.
