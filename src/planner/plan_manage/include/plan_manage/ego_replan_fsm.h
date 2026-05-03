/**
 * @file ego_replan_fsm.h
 * @brief EGOReplanFSM 有限状态机类的声明
 *
 * EGOReplanFSM 是 EGO-Planner 的核心状态机类，负责管理规划系统的整个生命周期。
 * 它整合了规划管理器（EGOPlannerManager）和可视化模块（PlanningVisualization），
 * 通过订阅里程计和航点话题，驱动有限状态机在六个状态之间转换，并触发相应的规划逻辑。
 *
 * 状态定义：
 *   - INIT          : 初始化状态，等待里程计和目标触发
 *   - WAIT_TARGET  : 等待目标（已收到目标点但尚未开始规划）
 *   - GEN_NEW_TRAJ : 生成全新轨迹（从无人机当前位置规划）
 *   - REPLAN_TRAJ  : 重规划（在当前轨迹基础上调整）
 *   - EXEC_TRAJ    : 执行轨迹（同时进行偏差和碰撞检测）
 *   - EMERGENCY_STOP: 紧急停止（检测到障碍且无法重规划）
 *
 * 目标类型：
 *   - MANUAL_TARGET : 用户通过 RViz 手动指定目标点
 *   - PRESET_TARGET : 使用 launch 文件中预设的航点
 *   - REFENCE_PATH  : 跟随参考路径
 */

#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- 状态枚举 ---------- */
    // FSM_EXEC_STATE: 规划状态机的执行状态枚举
    // 系统启动时处于 INIT，完成初始化后根据目标类型进入对应流程
    enum FSM_EXEC_STATE
    {
      INIT,           // 初始化：等待里程计和触发信号
      WAIT_TARGET,    // 等待目标：已收到目标但尚未开始规划
      GEN_NEW_TRAJ,   // 生成新轨迹：从当前无人机位置重新生成完整轨迹
      REPLAN_TRAJ,    // 重规划：在当前执行轨迹基础上调整（起点为轨迹上当前时刻位置）
      EXEC_TRAJ,      // 执行轨迹：发布 Bspline 并等待执行完毕或触发重规划
      EMERGENCY_STOP  // 紧急停止：发现障碍且来不及重规划，生成悬停轨迹
    };

    // TARGET_TYPE: 目标点获取方式枚举
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,   // 手动模式：用户在 RViz 中点击 2D Nav Goal
      PRESET_TARGET = 2,  // 预设模式：从 launch 文件读取固定航点序列
      REFENCE_PATH = 3    // 参考路径模式：跟随预定义路径（代码中未完整实现）
    };

    /* ---------- 规划工具（智能指针管理生命周期） ---------- */
    EGOPlannerManager::Ptr planner_manager_;    // 核心规划管理器（封装 GridMap、A*、B-spline 优化器）
    PlanningVisualization::Ptr visualization_;  // 可视化管理器（用于在 RViz 中显示路径、控制点、目标点）
    ego_planner::DataDisp data_disp_;           // 规划数据显示器消息（包含调试信息）

    /* ---------- 参数（从 launch 文件读取） ---------- */
    int target_type_;            // 目标类型：1=手动, 2=预设, 3=参考路径
    double no_replan_thresh_;   // 停止重规划的阈值：当接近目标 < 此距离时不再触发重规划
    double replan_thresh_;      // 触发重规划的阈值：当偏离轨迹起点 > 此距离时触发重规划
    double waypoints_[50][3];   // 预设航点坐标（最多 50 个，每个包含 xyz）
    int waypoint_num_;          // 预设航点数量
    double planning_horizen_;   // 局部规划的空间视野范围（米）
    double planning_horizen_time_; // 局部规划的时间视野（秒）
    double emergency_time_;     // 紧急停止前的缓冲时间（秒）

    /* ---------- 规划数据（运行时状态） ---------- */
    bool trigger_;              // 是否已收到目标触发信号（用户在 RViz 指定目标）
    bool have_target_;         // 是否有有效目标（全局轨迹已生成）
    bool have_odom_;           // 是否已收到里程计数据
    bool have_new_target_;     // 是否为新目标（用于强制使用多项式初始化而非轨迹延伸）
    FSM_EXEC_STATE exec_state_; // 当前执行状态
    int continously_called_times_; // 同一状态被连续调用的次数（用于检测异常卡死）

    Eigen::Vector3d odom_pos_;    // 里程计位置 (x, y, z)
    Eigen::Vector3d odom_vel_;    // 里程计速度 (vx, vy, vz)
    Eigen::Vector3d odom_acc_;    // 里程计加速度（当前未使用，可通过 IMU 估计）
    Eigen::Quaterniond odom_orient_; // 里程计姿态（四元数 w, x, y, z）

    Eigen::Vector3d init_pt_;   // 触发时刻的位置（用于重规划参考）
    Eigen::Vector3d start_pt_;  // 当前规划起点位置（GEN_NEW_TRAJ 时为 odom_pos_，REPLAN_TRAJ 时为轨迹上当前点）
    Eigen::Vector3d start_vel_; // 当前规划起点速度
    Eigen::Vector3d start_acc_; // 当前规划起点加速度
    Eigen::Vector3d start_yaw_; // 当前规划起点偏航角（当前未使用）
    Eigen::Vector3d end_pt_;   // 全局目标点位置
    Eigen::Vector3d end_vel_;   // 全局目标点速度
    Eigen::Vector3d local_target_pt_;  // 局部目标点位置（从全局轨迹上提取）
    Eigen::Vector3d local_target_vel_; // 局部目标点速度
    int current_wp_;            // 当前正在执行的航点索引

    bool flag_escape_emergency_; // 紧急脱困标志（避免重复调用紧急停止逻辑）

    /* ---------- ROS 通信工具 ---------- */
    ros::NodeHandle node_;                      // ROS 节点句柄
    ros::Timer exec_timer_;                    // FSM 主循环定时器（100Hz）
    ros::Timer safety_timer_;                  // 碰撞检测定时器（20Hz）
    ros::Subscriber waypoint_sub_;             // 航点话题订阅者（/waypoint_generator/waypoints）
    ros::Subscriber odom_sub_;                // 里程计话题订阅者（/odom_world）
    ros::Publisher replan_pub_;               // 预留重规划发布器
    ros::Publisher new_pub_;                  // 预留新轨迹发布器
    ros::Publisher bspline_pub_;              // B-spline 轨迹发布器（/planning/bspline）
    ros::Publisher data_disp_pub_;            // 调试数据发布器（/planning/data_display）

    /* ---------- 辅助函数 ---------- */
    // callReboundReplan: 调用回弹式重规划算法（前端 A* + 后端 B-spline 优化）
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);

    // callEmergencyStop: 生成紧急悬停轨迹
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    // planFromCurrentTraj: 从当前轨迹位置重规划
    bool planFromCurrentTraj();

    // changeFSMExecState: 切换 FSM 状态（带日志和连续调用计数）
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    // timesOfConsecutiveStateCalls: 获取连续状态调用信息
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    // printFSMExecState: 打印当前 FSM 状态
    void printFSMExecState();

    // planGlobalTrajbyGivenWps: 根据预设航点生成全局轨迹
    void planGlobalTrajbyGivenWps();

    // getLocalTarget: 从全局轨迹上确定局部目标点
    void getLocalTarget();

    /* ---------- ROS 回调函数 ---------- */
    // execFSMCallback: FSM 主循环回调（定时器驱动）
    void execFSMCallback(const ros::TimerEvent &e);

    // checkCollisionCallback: 碰撞检测回调（定时器驱动）
    void checkCollisionCallback(const ros::TimerEvent &e);

    // waypointCallback: 航点消息回调（收到 /waypoint_generator/waypoints 时触发）
    void waypointCallback(const nav_msgs::PathConstPtr &msg);

    // odometryCallback: 里程计回调（更新无人机当前状态）
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    // checkCollision: 预留的碰撞检测函数（当前未使用，由 checkCollisionCallback 代替）
    bool checkCollision();

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    // init: 初始化函数，由 main() 调用，完成所有模块的启动
    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
