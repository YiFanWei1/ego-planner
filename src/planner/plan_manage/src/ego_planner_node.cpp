/**
 * @file ego_planner_node.cpp
 * @brief EGO-Planner 主节点入口文件
 *
 * 该文件是 EGO-Planner 规划系统的 ROS 节点入口点。
 * 主要职责：初始化 ROS 节点、创建 EGOReplanFSM 有限状态机实例并启动。
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  // 初始化 ROS 节点，节点名为 "ego_planner_node"
  ros::init(argc, argv, "ego_planner_node");

  // 创建私有节点句柄（可通过 ~param_name 在 launch 文件中覆盖参数）
  ros::NodeHandle nh("~");

  // 创建 EGOReplanFSM 类的实例（有限状态机规划器）
  EGOReplanFSM rebo_replan;

  // 调用 init() 完成所有模块的初始化（订阅者、发布者、定时器、规划器等）
  rebo_replan.init(nh);

  // 等待系统完全启动（1秒），确保所有节点和服务都已就绪
  ros::Duration(1.0).sleep();

  // 进入 ROS 事件循环，处理回调函数（里程计回调、FSM 定时器回调、碰撞检测回调等）
  ros::spin();

  return 0;
}
