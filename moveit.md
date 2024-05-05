# #MOVEIT学习  

## 1.moveit安装
>🔗链接  


## 2.moveit的介绍
>🔗链接  


## 3.moveit接口  
### 3.1moveit的C++接口  
>代码
```c
#include<iostream> 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h> 
using namespace std;
int main(){

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


/////////////////start////////////////////

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group.move();

return 0;
}

```
>🔗链接  
https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp


### 3.2moveit的python接口
>代码
```python


```
>🔗链接  
https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py#L81

## 4.1moveit的命令脚本
脚本1
```
source ~/ws_moveit/devel/setup.bash
roslaunch panda_moveit_config demo.launch
```
脚本2
```
source ~/ws_moveit/devel/setup.bash

rosrun moveit_commander moveit_commander_cmdline.py

```
脚本2  
组名
>use group name    
>use panda_arm  

当前组状态  
>current  

记录当前状态  
>rec c  

机器人移动  
```
goal = c
goal[0] = 0.2
go goal
```
>🔗链接  
https://blog.csdn.net/qq_33328642/article/details/116902401

## 5.1机器人模型RobotModel和状态RobotState

RobotModel类包含所有链接和关节之间的关系，包括从 URDF 加载的关节限制属性。 RobotModel还将机器人的连杆和关节分成 SRDF 中定义的规划组

RobotState包含有关机器人及时快照的信息，存储关节位置的向量以及可选的速度和加速度，可用于获取有关机器人的运动学信息（取决于机器人的当前状态）



---
> 附件  
1.MAKEDWON的学习
https://blog.csdn.net/hyupeng1006/article/details/129590409  
2.moveit学习  
https://blog.csdn.net/qq_33328642/category_11150832.html  
3.官网学习 
https://github.com/moveit/moveit_tutorials/tree/melodic-devel  


