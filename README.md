 <h1>ECE 780 Project</h1>

`dobot_planning_scene` 是在Dobot官方ROS2包上新添加的包， 当前实现通过 `/apply_planning_scene` 服务一次性添加 **10 个随机球形障碍物**。

## 目录结构

```text
dobot_planning_scene/
├── config/
│   └── obstacles.yaml          # 预留的箱体、网格障碍物配置
├── dobot_planning_scene/
│   ├── random_scene.py        # 生成随机场景并调用 MoveIt 服务
│   └── spheres_generator.py   # 构造球形 CollisionObject
├── launch/
│   └── planning_scene.launch.py
├── meshes/
│   └── support.STL            # 预留的支架模型
├── package.xml
├── setup.cfg
└── setup.py
```

## 环境与编译

以下命令以 ROS 2 Humble、工作空间 `~/dobot_ws` 和 CR10 模型为例。需要提前安装 ROS 2、MoveIt 2、RViz 2、Git 和 colcon。

### 获取源码

在新环境中创建工作空间，并克隆本项目的 `2026_10_27` 分支（当前开发分支，包含 `dobot_planning_scene`）：

```bash
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src
git clone --branch 2026_10_27 https://github.com/harryshi0920/dobot_ws.git DOBOT_6Axis_ROS2_V4
```

仓库本身包含各个 ROS 2 功能包，因此放在工作空间的 `src/DOBOT_6Axis_ROS2_V4` 下。本文档位于仓库根目录，规划场景包位于 `dobot_planning_scene/` 子目录。

### 编译工作空间
在工作空间根目录编译：

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果已经完成整个工作空间的编译，后续只修改本包时可以使用：

```bash
colcon build --symlink-install --packages-select dobot_planning_scene
source install/setup.bash
```

新增模型或配置文件后，需要重新构建本包以更新安装目录中的资源。

## 启动方法

### 一键启动 MoveIt 演示和障碍物

```bash
cd ~/dobot_ws
source install/setup.bash
ros2 launch dobot_planning_scene planning_scene.launch.py
```

此 launch 文件会依次包含以下启动动作：

1. 通过 `dobot_moveit/moveit_demo.launch.py` 启动 `${DOBOT_TYPE}_moveit/demo.launch.py`。
2. 启动随机场景节点，等待 `/apply_planning_scene` 服务可用后提交障碍物。

两者的初始化没有固定延时，场景节点通过等待服务来配合 MoveIt 启动。此入口使用 MoveIt 演示配置，不会启动本仓库的真机通信驱动。

成功后会看到日志：

```text
Random spheres generate successfully
```

场景节点在服务返回后退出，这是正常行为；MoveIt 和 RViz 仍继续运行。在 RViz 的 MotionPlanning 显示中启用场景几何体显示，即可观察球形障碍物并尝试规划。

### 向已经运行的 MoveIt 添加障碍物

如果 MoveIt 已经启动且提供 `/apply_planning_scene` 服务，只需在已加载 ROS 2 和工作空间环境的终端执行：

```bash
ros2 run dobot_planning_scene random_scene
```

独立运行时默认节点名为 `random_scene`；使用本包 launch 启动时，节点名为 `planning_scene_loader`。

## 随机场景的生成规则
`spheres_generator.py`为生成球体function，`random_scene.py`包含随即算法

所有球体都使用 `base_link` 坐标系，长度单位为米。

| 配置项 | 当前值 | 代码位置 |
| --- | --- | --- |
| 球体数量 | `10` | `random_scene.py` 中的 `main()` |
| 随机种子 | `None`，每次随机生成 | `random_scene.py` 中的 `main()` |
| 半径 | `0.05`～`0.2` | `generate_random_spheres()` |
| 球心 X | `-1.3`～`1.3` | `generate_random_spheres()` |
| 球心 Y | `-0.5`～`0.5` | `generate_random_spheres()` |
| 球心 Z | `0.1`～`0.5` | `generate_random_spheres()` |
| 障碍物 ID | `sphere_0`～`sphere_9` | `generate_random_spheres()` |
| 坐标系 | `base_link` | `spheres_generator.py` |

这些设置目前是 Python 代码中的常量，**尚未提供对应的 ROS 参数或 launch 参数**。如需固定场景，可将 `main()` 中的 `random_seed` 改为固定整数；如需改变数量或分布范围，修改对应代码后重新构建并加载工作空间环境。

随机生成过程不会检查球体之间或球体与机器人之间是否相交，也不会保证目标点一定可达。场景导致规划失败时，可调整分布范围或数量，或重新运行节点生成场景。

## LOG
### 2026-10-27
1. 更改ompl_planning.yaml为github上 moveit/moveit_resource/blob/humble/panda_moveit_config/config/ompl_planning.yaml中的结构。原ompl_planning.yaml会使MoveitConfigsBuilder自动在外面包一层ompl
2. 更改URDF的joint limit，原代码中的joint limit设置为+-360。更改为+-180

### 2026-10-28
1. 增加dobot_planning_scene/dobot_planning_scene/spheres_generator.py和random_scene.py

## Future Work
1. 更改random_scene.py中的生成算法，相比较与提取小球的物理碰撞的方式检测个规避小球之间相互叠加和小球与机械臂的碰撞。建议使用算法使小球分布均匀的同时，规避生成在机械臂现有区域直接穿模。


