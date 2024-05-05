## 11.时间参数化

### MoveIt 确实利用后处理来对速度和加速度值的运动轨迹进行时间参数化

```
max_position, min_position, max_velocity, max_acceleration
has_velocity_limits, has_acceleration_limits
```

### 速度控制

MoveIt 将关节轨迹的速度和加速度设置为机器人 URDF 或 `joint_limits.yaml`

`joint_limits.yaml`是从设置助手生成的，最初是 URDF 中值的精确副本。如果需要特殊约束，用户可以将这些值修改为小于原始 URDF 值。可以使用按键更改特定的关节属性
`max_position, min_position, max_velocity, max_acceleration`
关节限制可以通过按键打开或关闭
`has_velocity_limits, has_acceleration_limits`

### 运行时期间

参数化运动轨迹的速度也可以在运行时修改为配置值中设置的最大速度和加速度的一部分（0-1 之间的值）

要更改每个运动计划的速度，您可以设置两个缩放因子，[MotionPlanRequest.msg](http://docs.ros.org/melodic/api//moveit_msgs/html/msg/MotionPlanRequest.html)所述并且[MoveIt MotionPlanning RViz 插件](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)中还提供了用于设置这两个因素的旋转框。

### 时间参数化

MoveIt 可以支持不同的算法来对运动轨迹进行后处理，以添加时间戳和速度/加速度值。目前 MoveIt 中默认提供三种：

* [迭代抛物线时间参数化](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp)
* [迭代样条参数化](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/trajectory_processing/src/iterative_spline_parameterization.cpp)
* [时间最优轨迹生成](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp)

默认情况下，[迭代抛物线时间参数化算法在运动规划管道](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/motion_planning_pipeline/motion_planning_pipeline_tutorial.html)用作规划请求适配器

> 注意存在的问题
> 默认情况下，迭代抛物线时间参数化算法在运动规划管道中用作规划请求适配器，如本教程中所述。尽管 MoveIt 使用的迭代抛物线时间参数化算法多年来已被数百个机器人使用，但它存在已知的错误。

迭代样条参数化算法与PR 382合并，作为处理这些问题的方法。虽然初步实验非常有希望，但在完全替换迭代抛物线时间参数化算法之前，我们正在等待社区的更多反馈。

PR #809和#1365中引入的时间最优轨迹生成可生成具有非常平滑和连续速度曲线的轨迹。该方法基于将路径段拟合到原始轨迹，然后从优化路径中采样新的航路点。这与严格的时间参数化方法不同，因为所得航路点可能在一定容差内偏离原始轨迹。因此，使用此方法时可能需要额外的碰撞检查。
