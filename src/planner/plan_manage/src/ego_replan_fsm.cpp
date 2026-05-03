/**
 * @file ego_replan_fsm.cpp
 * @brief EGO-Planner 有限状态机（FSM）实现
 *
 * EGOReplanFSM 类负责管理 EGO-Planner 的核心规划流程。
 * 系统共包含 6 个状态，通过定时器驱动状态机，根据当前状态决定是否调用重规划和轨迹生成逻辑。
 *
 * 状态转换图:
 *   INIT ──(收到里程计+触发)──► WAIT_TARGET ──(收到目标点)──► GEN_NEW_TRAJ
 *                                                                            │
 *   EXEC_TRAJ ◄──────────────────────────┘                                   │
 *       │                                                                        │
 *       │       (偏离轨迹/碰撞检测)                                             │
 *       ▼                                                                        │
 *   REPLAN_TRAJ ──(规划失败+未脱困)──► EMERGENCY_STOP                         │
 *       │                                                                        │
 *       │       (重规划成功)                                                    │
 *       └──────────────────────► EXEC_TRAJ                                    │
 */

#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  /**
   * @brief FSM 初始化函数
   * @details 从 ROS 参数服务器读取配置，创建订阅者/发布者，设置定时器，
   *          并根据目标类型（手动/预设）决定启动方式。
   * @param nh ROS 节点句柄（从 main() 中传入的私有节点句柄）
   */
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    // ---------- 初始化状态变量 ----------
    current_wp_ = 0;                          // 当前航点索引，从第 0 个开始执行
    exec_state_ = FSM_EXEC_STATE::INIT;       // 初始状态为 INIT
    have_target_ = false;                     // 尚未收到目标点
    have_odom_ = false;                        // 尚未收到里程计数据

    /* ---------- 读取 FSM 相关参数（从 launch 文件或 yaml 配置） ---------- */
    // 飞行模式: 1=手动指定目标, 2=预设航点, 3=参考路径（见 TARGET_TYPE 枚举）
    nh.param("fsm/flight_type", target_type_, -1);

    // 触发重规划的偏差阈值：当无人机偏离原轨迹超过此距离时，触发重规划
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);

    // 停止重规划的距离阈值：当无人机接近目标点小于此距离时，停止重规划
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

    // 局部规划的空间视野范围（米）：在当前轨迹前方多少距离内寻找局部目标点
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);

    // 局部规划的时间视野（秒）：规划多久时长的轨迹（目前代码中主要使用 planning_horizen_）
    nh.param("fsm/planning_horizon_time", planning_horizen_time_, -1.0);

    // 紧急停止的缓冲时间（秒）：发现障碍物后，在多少秒内强制悬停后再尝试重规划
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    // 预设航点的数量（仅在 target_type_ == PRESET_TARGET 时使用）
    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    // 读取每个预设航点的三维坐标 (x, y, z)
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* ---------- 初始化核心模块 ---------- */
    // PlanningVisualization: 用于在 RViz 中可视化规划结果（路径、控制点、目标点等）
    visualization_.reset(new PlanningVisualization(nh));

    // EGOPlannerManager: 核心规划管理器，封装了 GridMap、A* 搜索和 B-spline 优化算法
    planner_manager_.reset(new EGOPlannerManager);
    // 调用 initPlanModules 初始化所有规划子模块
    planner_manager_->initPlanModules(nh, visualization_);

    /* ---------- 创建定时器（驱动状态机） ---------- */
    // exec_timer_: 以 100Hz (0.01s) 的频率执行 execFSMCallback，推动状态机运转
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);

    // safety_timer_: 以 20Hz (0.05s) 的频率执行 checkCollisionCallback，检测轨迹是否与障碍物碰撞
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    /* ---------- 订阅话题 ---------- */
    // 订阅里程计数据（位置 + 速度），是所有规划决策的基础输入
    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    /* ---------- 发布话题 ---------- */
    // 发布优化后的 B-spline 轨迹，供 traj_server 转换为位置指令
    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);

    // 发布规划调试数据（当前状态、规划耗时等），供数据监控使用
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    /* ---------- 根据目标类型启动规划流程 ---------- */
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      // 手动模式：订阅 waypoint_generator 发出的航点路径消息
      // 当用户在 RViz 中点击 "2D Nav Goal" 时，waypoint_generator 会发布此话题
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // 预设航点模式：等待里程计就绪后，直接使用 launch 文件中预设的航点启动规划
      ros::Duration(1.0).sleep();  // 等待 1 秒让其他节点（如里程计）完成初始化
      while (ros::ok() && !have_odom_)
        ros::spinOnce();           // 阻塞等待，直到收到第一条里程计消息
      planGlobalTrajbyGivenWps();   // 根据预设航点生成全局轨迹
    }
    else
    {
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
    }
  }

  /**
   * @brief 根据预设航点列表生成全局轨迹
   * @details 在 PRESET_TARGET 模式下调用。读取 init() 中加载的 waypoints_ 数组，
   *          调用 planner_manager_->planGlobalTrajWaypoints() 生成全局参考轨迹，
   *          然后触发状态机进入 GEN_NEW_TRAJ 状态。
   */
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    // 构建航点向量（从 ROS 参数中读取的 waypoints_）
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();  // 记录最后一个航点作为最终目标点
    }

    // 调用规划管理器，生成从当前位置出发、经过所有航点、到达最终目标的全局轨迹
    // 参数：起始位置、起始速度(零)、起始加速度(零)、航点列表、终止速度(零)、终止加速度(零)
    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 可视化：依次显示所有航点（每个间隔 1ms，避免可视化消息过多导致拥塞）
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {
      // 计算全局轨迹上的离散采样点，用于在 RViz 中显示全局路径
      constexpr double step_size_t = 0.1;  // 采样间隔 0.1 秒
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();       // 终止速度设为零（到达目标后悬停）
      have_target_ = true;      // 标记已有目标，开始规划流程
      have_new_target_ = true;   // 标记新目标，强制使用多项式初始化（而非从当前轨迹重规划）

      // 触发状态机：从 INIT 直接跳转到 GEN_NEW_TRAJ
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");

      ros::Duration(0.001).sleep();
      // 在 RViz 中显示全局路径线
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /**
   * @brief 航点回调函数（MANUAL_TARGET 模式）
   * @details 订阅 /waypoint_generator/waypoints，当用户通过 RViz 指定目标后被调用。
   *          从消息中提取目标点位置，调用全局轨迹规划，并触发状态机转换。
   * @param msg 来自 waypoint_generator 的路径消息（通常只包含一个目标点）
   */
  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    // 如果目标点高度低于 -0.1m（无效高度），直接忽略
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;             // 标记已收到触发信号（用于 INIT -> WAIT_TARGET 转换）
    init_pt_ = odom_pos_;       // 记录触发时刻的无人机位置，作为规划起点参考

    bool success = false;

    // 提取目标点坐标（固定高度为 1.0m，即预设飞行高度）
    end_pt_ << msg->poses[0].pose.position.x,
               msg->poses[0].pose.position.y,
               1.0;

    // 调用全局轨迹规划：当前状态 -> 目标点（速度和加速度均为零）
    success = planner_manager_->planGlobalTraj(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 可视化目标点（青色圆点，半径 0.3m）
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      // 采样全局轨迹上的离散点，用于 RViz 可视化
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      // 根据当前状态决定触发哪种规划流程：
      // - 如果正在悬停等待目标 (WAIT_TARGET)，生成全新轨迹 (GEN_NEW_TRAJ)
      // - 如果正在执行轨迹 (EXEC_TRAJ)，触发重规划 (REPLAN_TRAJ)
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "FSM");

      // 在 RViz 中显示全局路径
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /**
   * @brief 里程计回调函数
   * @details 订阅 /odom_world，从 Odometry 消息中提取无人机的位置、速度和姿态（ quaternion ），
   *          并更新类成员变量 odom_pos_、odom_vel_、odom_orient_。
   *          这些数据在 execFSMCallback() 和所有规划决策中被持续使用。
   * @param msg nav_msgs/Odometry 消息，来自 VIO/LIO 等定位系统
   */
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // 提取位置 (x, y, z)
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 提取线速度 (vx, vy, vz)
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 提取姿态四元数 (w, x, y, z)
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;  // 标记已收到有效里程计，使状态机可以退出 INIT 状态
  }

  /**
   * @brief 切换 FSM 执行状态
   * @details 更新 exec_state_，并记录状态转换日志（来源 + 转换前后状态名）。
   *          同时追踪同一状态被连续调用的次数（用于检测异常卡死）。
   * @param new_state 目标状态
   * @param pos_call 调用来源标识（如 "FSM"、"TRIG"、"SAFETY"）
   */
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    // 如果切换到与当前相同的状态，递增连续调用计数；否则重置为 1
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    // 状态名称映射表（用于日志输出）
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    int pre_s = int(exec_state_);  // 记录当前状态（用于日志）
    exec_state_ = new_state;        // 执行状态切换

    // 打印状态转换日志，格式：[调用来源]: from 旧状态 to 新状态
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  /**
   * @brief 获取连续状态调用的统计信息
   * @return pair(连续调用次数, 当前状态)
   */
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  /**
   * @brief 打印当前 FSM 状态（供 execFSMCallback 定时调用）
   */
  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  /**
   * @brief FSM 主循环回调（10ms 定时器，每 100Hz 执行一次）
   * @details 这是状态机的核心驱动函数。根据当前 exec_state_ 执行对应状态的处理逻辑。
   *          每 100 次调用（约每秒）打印一次状态信息。
   * @param e ROS 定时器事件（包含触发时间戳）
   */
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    static int fsm_num = 0;
    fsm_num++;

    // 每 100 次调用（约每秒）打印一次状态和健康信息
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;   // 尚无里程计数据
      if (!trigger_)
        cout << "wait for goal." << endl;  // 等待目标触发
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      // INIT 状态：等待里程计和目标触发信号，二者就绪后转入 WAIT_TARGET
      if (!have_odom_)
      {
        return;  // 尚无里程计，保持 INIT 状态
      }
      if (!trigger_)
      {
        return;  // 尚无目标触发，保持 INIT 状态
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      // WAIT_TARGET 状态：等待 have_target_ 被置为 true（即 waypointCallback 中已规划好全局轨迹）
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // GEN_NEW_TRAJ 状态：从当前无人机状态出发，调用 callReboundReplan() 生成新的局部轨迹
      // 更新规划起点状态（当前位置、当前速度、加速度设为零）
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // 判断是否为首次调用 GEN_NEW_TRAJ：
      // - 首次调用（timesOfConsecutiveStateCalls().first == 1）：使用确定性的多项式初始化
      // - 重复调用（规划失败后重试）：使用随机多项式初始化，增加探索性
      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      // 调用重规划算法（前端初始化 + 后端优化），生成新的局部 B-spline 轨迹
      bool success = callReboundReplan(true, flag_random_poly_init);

      if (success)
      {
        // 规划成功，进入轨迹执行状态，并重置紧急脱困标志
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        // 规划失败，保持 GEN_NEW_TRAJ 状态，等待下一次回调重试
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // REPLAN_TRAJ 状态：在当前执行的轨迹基础上进行重规划（而非从当前位置重新生成）
      // 适用于轨迹执行过程中发现偏差或障碍，需要在线调整的情形
      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // 重规划失败，继续保持在 REPLAN_TRAJ 状态等待重试
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ:
    {
      // EXEC_TRAJ 状态：执行当前轨迹，同时判断是否需要触发重规划
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();

      // 计算当前时刻（相对于轨迹起始时间）
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);  // 不超过轨迹总时长

      // 获取当前时刻无人机沿轨迹应该到达的位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 条件1：轨迹执行完毕（当前时刻超过轨迹时长），回到 WAIT_TARGET 等待新目标
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // 条件2：当前位置已非常接近目标点（< no_replan_thresh_），停止重规划
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        return;
      }
      // 条件3：当前位置离轨迹起点太近（< replan_thresh_），暂时不重规划（避免起点附近频繁触发）
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        return;
      }
      // 条件4：偏离原轨迹超过阈值，触发重规划
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      // EMERGENCY_STOP 状态：发现障碍物且来不及重规划时的紧急处理
      // flag_escape_emergency_ 用于避免重复调用紧急停止逻辑
      if (flag_escape_emergency_)
      {
        // 调用 EmergencyStop()，生成一个所有控制点都是当前位置的悬停轨迹
        callEmergencyStop(odom_pos_);
      }
      else
      {
        // 如果速度已经降得很低（< 0.1 m/s），认为无人机已停稳，尝试重新生成轨迹
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    // 周期性发布调试数据（/planning/data_display）
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  /**
   * @brief 从当前轨迹的当前位置开始重规划
   * @details 在 REPLAN_TRAJ 状态中被调用。与 GEN_NEW_TRAJ 不同的是：
   *          起点是当前轨迹上 t_cur 时刻的位置/速度/加速度，而非无人机的实时里程计状态。
   *          这保证了重规划轨迹与原轨迹在空间上平滑衔接。
   * @return 规划是否成功
   */
  bool EGOReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();

    // 计算当前时刻相对于轨迹起始时间的偏移
    double t_cur = (time_now - info->start_time_).toSec();

    // 从当前轨迹中采样当前位置、速度和加速度，作为重规划的起点约束
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 第一次尝试：使用非随机多项式初始化（确定性）
    bool success = callReboundReplan(false, false);

    if (!success)
    {
      // 第二次尝试：仍使用确定性初始化（重置起点）
      success = callReboundReplan(true, false);

      if (!success)
      {
        // 第三次尝试：使用随机多项式初始化，增加探索性（可能在障碍物密集区域找到替代路径）
        success = callReboundReplan(true, true);

        if (!success)
        {
          return false;  // 三次尝试均失败，返回失败
        }
      }
    }

    return true;
  }

  /**
   * @brief 碰撞检测回调（50ms 定时器，每 20Hz 执行一次）
   * @details 沿着当前轨迹前瞻，检测是否有控制点落在障碍物占据的栅格中。
   *          发现碰撞时根据剩余安全时间决定触发重规划或紧急停止。
   * @param e ROS 定时器事件
   */
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 如果尚未开始规划（WAIT_TARGET）或轨迹尚未初始化，直接返回
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- 沿轨迹前瞻检查碰撞 ---------- */
    constexpr double time_step = 0.01;  // 沿轨迹以 10ms 步长采样
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;  // 轨迹的后 1/3 被视为不关键区域

    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      // 如果当前时刻已经过了 2/3 处，后续区域被视为已执行完毕，不再检查
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      // 调用 GridMap 的膨胀占据查询（考虑安全膨胀半径）
      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        // 发现碰撞，尝试从当前位置重规划
        if (planFromCurrentTraj())
        {
          // 重规划成功，立即进入轨迹执行状态
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          // 重规划失败，判断是否需要紧急停止
          if (t - t_cur < emergency_time_)
          {
            // 发现障碍物时距离过近，预判来不及重规划，触发紧急停止
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            // 有足够的安全时间，触发常规重规划
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  /**
   * @brief 调用回弹式重规划算法（前端 + 后端）
   * @details 这是 EGO-Planner 的核心规划接口，整合了 A* 路径搜索和 B-spline 轨迹优化。
   *          流程：确定局部目标点 -> 调用 planner_manager_->reboundReplan() -> 发布 Bspline 消息
   * @param flag_use_poly_init 是否使用多项式轨迹初始化（首次规划/重规划失败时为 true）
   * @param flag_randomPolyTraj 是否使用随机扰动的多项式初始化（增加探索性）
   * @return 规划是否成功
   */
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 确定局部目标点：从全局轨迹上找到 planning_horizen_ 范围内的最优点
    getLocalTarget();

    // 调用 EGOPlannerManager 的核心重规划函数
    // 参数：起点状态（位置、速度、加速度）、局部目标（位置、速度）、初始化标志
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_,
                                        local_target_pt_, local_target_vel_,
                                        (have_new_target_ || flag_use_poly_init),
                                        flag_randomPolyTraj);
    have_new_target_ = false;  // 重置新目标标志

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;

      /* ---------- 将优化后的 B-spline 轨迹打包为 ROS 消息并发布 ---------- */
      ego_planner::Bspline bspline;
      bspline.order = 3;  // 三阶（阶数=度数=3），即三次 B-spline
      bspline.start_time = info->start_time_;  // 轨迹起始时间（用于 traj_server 计算相对时间）
      bspline.traj_id = info->traj_id_;         // 轨迹唯一 ID（用于轨迹验证）

      // 提取 B-spline 控制点（3 x N 矩阵），转换为 geometry_msgs::Point 数组
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      // 提取 B-spline 节点向量，转换为 double 数组
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      // 发布 Bspline 消息给 traj_server
      bspline_pub_.publish(bspline);

      // 在 RViz 中可视化优化后的控制点（绿色）
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  /**
   * @brief 紧急停止：生成悬停轨迹
   * @details 当检测到障碍物且来不及重规划时调用。生成一个所有控制点
   *          都固定在当前位置的 B-spline，使无人机立即悬停。
   * @param stop_pos 悬停位置（通常是当前里程计位置）
   * @return 是否成功
   */
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    // 调用规划管理器的紧急停止接口
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* ---------- 将悬停轨迹打包为 Bspline 消息并发布 ---------- */
    // （与 callReboundReplan 中相同的打包逻辑）
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  /**
   * @brief 确定局部目标点（Local Target）
   * @details 在全局轨迹上搜索离当前无人机位置"最近的前方点"，
   *          作为 B-spline 优化的局部目标。搜索范围受 planning_horizen_ 限制。
   *          局部目标点的选取直接影响局部轨迹的形状和安全性。
   */
  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    // 计算沿全局轨迹搜索时的步长：基于最大速度和规划视野
    // 步长 = planning_horizon / 20 / max_vel，确保在规划视野内有足够的采样点
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;

    // 从上一次进度时间开始，沿全局轨迹向前搜索
    for (t = planner_manager_->global_data_.last_progress_time_;
         t < planner_manager_->global_data_.global_duration_;
         t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      // 错误处理：如果搜索起点落后于 last_progress_time_ 且距离超过规划视野，报错
      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }

      // 记录离无人机最近的点及其时间
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      // 当距离首次超过规划视野时，取当前点作为局部目标点
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        // 更新进度时间，下次从当前位置继续搜索
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }

    // 如果遍历完整个全局轨迹仍没有找到超出规划视野的点，
    // 说明目标点本身就在规划视野内，取最终目标点作为局部目标
    if (t > planner_manager_->global_data_.global_duration_)
    {
      local_target_pt_ = end_pt_;
    }

    // 计算局部目标速度：
    // - 如果离目标很近（小于 2a/v 的制动距离），速度设为零（平滑减速）
    // - 否则使用全局轨迹上该点处的切向速度
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
