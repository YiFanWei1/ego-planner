/**
 * @file planner_manager.cpp
 * @brief EGO-Planner 核心规划管理器实现
 *
 * EGOPlannerManager 封装了 EGO-Planner 的所有底层算法模块：
 *   1. GridMap（栅格地图）：管理占据栅格，用于碰撞检测
 *   2. A*（动态路径搜索）：在栅格地图中搜索可通行的初始路径
 *   3. BsplineOptimizer（B样条优化器）：将初始路径优化为满足动力学约束的 B-spline 轨迹
 *
 * 核心接口为 reboundReplan()，整合了上述三个模块，完成从起点到局部目标的重规划。
 */

#include <plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{

  // ========== 构造函数与析构函数 ==========

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  /**
   * @brief 初始化所有规划子模块
   * @details 从参数服务器读取配置，创建 GridMap（地图管理）、A* 搜索器、
   *          B-spline 优化器，并绑定可视化模块。
   * @param nh ROS 节点句柄
   * @param vis 可视化模块指针（用于在 RViz 中显示规划结果）
   */
  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* ---------- 读取算法参数 ---------- */

    // 最大速度 (m/s)，用于时间参数化和可行性检查
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);

    // 最大加速度 (m/s^2)，用于时间参数化和可行性检查
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);

    // 最大加加速度 (m/s^3)，用于轨迹平滑约束
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);

    // 动力学可行性容差：允许速度/加速度超出限制的比例（0.0 表示严格约束）
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);

    // B-spline 控制点之间的期望距离 (m)，影响轨迹的分辨率和计算效率
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

    // 局部规划的空间视野范围 (m)，定义了每次规划的空间窗口大小
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    // 初始化轨迹 ID（每次规划成功后递增，用于轨迹去重和验证）
    local_data_.traj_id_ = 0;

    // ---------- 创建并初始化 GridMap ----------
    // GridMap 负责：接收深度/点云数据 → 更新栅格占据状态 → 提供碰撞查询接口
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // ---------- 创建并初始化 B-spline 优化器 ----------
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);           // 从参数服务器读取优化器参数
    bspline_optimizer_rebound_->setEnvironment(grid_map_); // 将地图模块注入优化器（用于计算碰撞代价）

    // ---------- 创建并初始化 A* 搜索器 ----------
    // A* 搜索器在 BsplineOptimizer 内部使用，用于生成初始可行路径
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION 接口初始化

  // ========== 核心重规划算法 ==========

  /**
   * @brief 回弹式重规划算法（Rebound Replan）
   *
   * 这是 EGO-Planner 的核心规划函数，整合了前端路径搜索和后端轨迹优化。
   * 采用"前端-后端"（Front-End + Back-End）架构：
   *
   *   前端（Front-End）：快速生成一条初始可行路径
   *     ├─ 方案A：多项式轨迹（起点→目标点的最短时间 5次多项式）
   *     └─ 方案B：从当前执行轨迹延伸（保证与原轨迹平滑衔接）
   *     → A* 搜索器对前端路径进行扰动，使其通过无障碍区域
   *
   *   后端（Back-End）：数值优化，将初始路径精炼为满足所有约束的最优轨迹
   *     ├─ B-spline 参数化 + L-BFGS 梯度下降优化
   *     └─ 时间重分配（当速度/加速度超限时延长轨迹时长）
   *
   * 算法流程分为三个阶段：
   *   STEP 1 INIT   ：初始化（多项式/轨迹采样 → 参数化为 B-spline 控制点 → A* 搜索扰动）
   *   STEP 2 OPT    ：B-spline 优化（最小化代价函数，满足安全和动力学约束）
   *   STEP 3 REFINE ：时间重分配（如优化后速度/加速度超限，则重新分配时间参数）
   *
   * @param start_pt     起点位置
   * @param start_vel    起点速度
   * @param start_acc    起点加速度
   * @param local_target_pt   局部目标位置（来自全局轨迹的 planning_horizon 边缘点）
   * @param local_target_vel  局部目标速度（全局轨迹在目标点处的切向速度）
   * @param flag_polyInit     是否使用多项式初始化（首次规划或强制重新初始化时为 true，
   *                          即 FSM 中 have_new_target_==true 或规划失败重试时）
   * @param flag_randomPolyTraj 是否使用随机多项式（多次重试时增加探索性，
   *                            在起点和目标之间插入随机扰动中间点）
   * @return 规划是否成功
   */
  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose()
         << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    // ========== 距离检查：起终点过近则直接返回失败 ==========
    // 如果起点与目标点距离小于 0.2m，说明无人机已经非常接近目标，
    // 视为"已到达"，无需再规划。返回 false 让 FSM 等待目标更新。
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    // 记录各阶段耗时，用于性能分析
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /****************************************************************************
     * STEP 1: 初始化 —— 生成初始路径（Front-End）
     *
     * 目标：快速生成一条从起点到局部目标的初始可行路径。
     * 这条路径不需要最优，只需要"大致合理"且"不穿过障碍物"。
     *
     * 两种初始化策略：
     *   策略A（多项式初始化）：适用于首次规划（have_new_target_=true）
     *     - 直接用 5次多项式连接起点和目标点，速度/加速度连续
     *     - 如 flag_randomPolyTraj=true，在中间插入随机扰动点（增加探索性）
     *
     *   策略B（轨迹延伸初始化）：适用于执行中的重规划
     *     - 从当前正在执行的轨迹的当前位置开始延伸
     *     - 保证新轨迹与原轨迹在空间上平滑衔接，无突兀跳跃
     *
     * 无论哪种策略，最终都会得到一组均匀采样的路径点 point_set，
     * 然后用 B-spline 逆参数化（给定路径点和端点导数，反求控制点）
     ****************************************************************************/

    // 计算基础时间步长 ts（B-spline 离散化时的采样间隔）
    // ts 决定了 B-spline 的"密度"：ts 越小，控制点越密，轨迹分辨率越高
    //
    // 推导：
    //   最小采样时间 = 控制点间距 / 最大速度 = ctrl_pt_dist / max_vel
    //   乘以 1.2 系数，留出安全余量，保证采样点足够密集
    //
    //   近距离（<0.1m）时：ts * 5，因为此时路径很短，采样点数量本身就少，
    //   需要扩大 ts 来避免控制点过密导致计算开销增加
    double ts = (start_pt - local_target_pt).norm() > 0.1
                    ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2
                    : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;

    // point_set：初始路径上的采样点序列（按时间顺序排列）
    // start_end_derivatives：端点导数约束（顺序：起点速度、终点速度、起点加速度、终点加速度）
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;

    // 异常恢复循环：如果初始路径异常（如控制点数量异常多），重新生成
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      /*** 初始化策略分支 ***/

      // ========== 分支A：多项式初始化 ==========
      // 触发条件：首次调用（flag_first_call）或 FSM 要求强制使用多项式（flag_polyInit）
      if (flag_first_call || flag_polyInit || flag_force_polynomial)
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        /* 计算最短时间（梯形速度曲线）：
         *
         * 梯形速度曲线的物理约束：
         *   - 以 max_acc_ 匀加速启动，从 0 加速到 max_vel_
         *   - 以 max_vel_ 匀速巡航
         *   - 以 max_acc_ 匀减速到 0
         *
         * 加速距离 = 减速距离 = v^2 / (2*a)
         * 如果总距离 dist <= 2 * v^2 / (2*a) = v^2/a，则存在匀速段
         *
         *   dist > v^2/a 时（长距离，有匀速段）：
         *     t = (dist - v^2/a) / v + 2 * v/a
         *       = dist/v + v/a
         *
         *   dist <= v^2/a 时（短距离，仅加减速，无匀速段）：
         *     t = sqrt(4 * dist / a)  （匀加速到中点 + 匀减速到终点）
         *       = 2 * sqrt(dist / a)
         *
         * 代码实现：
         *   if (v^2/a > dist)  t = sqrt(dist / a) * 2
         *   else                t = (dist - v^2/a) / v + 2 * v / a
         *                      = dist/v + v/a
         */
        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist
                          ? sqrt(dist / pp_.max_acc_)
                          : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        /* 生成多项式轨迹 */
        if (!flag_randomPolyTraj)
        {
          // ===== 确定性多项式：直接连接起点和目标点 =====
          // 使用 one_segment_traj_gen() 生成一段 5次多项式：
          //   - 约束：起点/终点的位置、速度、加速度（共12个约束，正好确定6个系数）
          //   - 中间点约束设为零向量（不起作用，仅为函数签名兼容）
          gl_traj = PolynomialTraj::one_segment_traj_gen(
              start_pt, start_vel, start_acc,
              local_target_pt, local_target_vel,
              Eigen::Vector3d::Zero(), time);
        }
        else
        {
          // ===== 随机多项式：在起点和目标之间插入随机扰动中间点 =====
          //
          // 目的：在障碍物密集或之前规划失败的区域，通过引入随机扰动来增加探索性
          //
          // 随机扰动方向构造：
          //   horizen_dir  = (start→target) × (0,0,1)  → 水平面上的横向单位向量
          //   vertical_dir = (start→target) × horizen_dir → 垂直于横向的另一个水平方向
          //   两个方向互相正交，且都在水平面内，覆盖了所有水平面内的扰动方向
          //
          // 扰动幅度随连续失败次数调整：
          //   - 初始时（failures=0）：amplitude ≈ 0.8 * dist * 0.011（很小）
          //   - 失败越多：amplitude 逐渐增大，最大可达 0.8 * dist * 0.8（很大）
          //   这是一个反比例映射：(-0.978/(n+0.989) + 0.989) 在 n=0 时 ≈ 0.011，在 n→∞ 时趋近 0.989
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

          // 三点：起点、随机中间点、目标点，组成两段多项式
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        /* 从多项式轨迹上均匀采样路径点 */
        // 目的：将连续的多项式轨迹离散化为点集，作为 B-spline 的几何约束
        double t;
        bool flag_too_far;
        ts *= 1.5;  // 预扩大，在下面的二分搜索中逐步缩小
        do
        {
          ts /= 1.5;  // 二分搜索：逐步缩小步长，直到采样点间距满足要求
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            // 质量检查：相邻采样点的空间距离不能超过 ctrl_pt_dist * 1.5
            // 如果超过，说明步长太大导致"跳点"，需要继续细分
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7);
        // 退出条件：
        //   - flag_too_far == false：所有采样点间距都合格
        //   - point_set.size() >= 7：保证有足够的点（3阶 B-spline 需要至少 4+阶数 个点才能定义）
        t -= ts;

        // 提取端点导数约束，用于 B-spline 参数化
        // B-spline 的起点/终点行为由首尾控制点决定，
        // 但通过设置"虚拟"端点值，可以控制起点/终点处的速度/加速度
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));        // 起点速度：多项式在 t=0 处的一阶导
        start_end_derivatives.push_back(local_target_vel);               // 终点速度：全局轨迹在目标点的切向速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));        // 起点加速度：多项式在 t=0 处的二阶导
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));         // 终点加速度：多项式在终点处的一阶导（来自导数）
      }

      // ========== 分支B：轨迹延伸初始化 ==========
      // 触发条件：非首次调用重规划（REPLAN_TRAJ 状态），且未强制使用多项式
      // 特点：从当前执行轨迹的"当前位置"开始延伸，保证新轨迹与原轨迹平滑衔接
      else
      {
        double t;
        // 计算当前时刻（相对于当前轨迹起始时间的偏移量）
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;  // 伪弧长数组：每个采样点到起点的累积弧长
        vector<Eigen::Vector3d> segment_point;  // 从当前轨迹上采样的位置点

        // 沿当前轨迹均匀采样，从 t_cur 采样到轨迹终点
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          // evaluateDeBoorT(t)：在 B-spline 轨迹上求 t 时刻的位置
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            // 累积弧长：当前点与上一个点的距离 + 之前的总弧长
            pseudo_arc_length.push_back(
                (segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        // 计算从当前轨迹末端（last时刻）到局部目标点的过渡时间
        // 公式：距离 / 最大速度 * 2（给予足够的时间裕度，避免速度超限）
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          // 生成多项式过渡段：从轨迹末端状态（位置+速度+加速度）过渡到目标状态
          // 这里将端点加速度设为零，表示希望到达目标时速度为零（平滑停止）
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
              local_data_.position_traj_.evaluateDeBoorT(t),
              local_data_.velocity_traj_.evaluateDeBoorT(t),
              local_data_.acceleration_traj_.evaluateDeBoorT(t),
              local_target_pt, local_target_vel,
              Eigen::Vector3d::Zero(), poly_time);

          // 将多项式过渡段的采样点追加到 segment_point
          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back(
                  (segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        /* 伪弧长均匀重采样：
         *
         * 问题：segment_point 中的点是在固定时间步长 ts 下采样的，
         *       但 B-spline 优化需要"均匀间距"的控制点。
         *
         * 解决：使用伪弧长参数化：
         *       将 segment_point 按累积弧长重新均匀采样，得到 point_set
         *
         * 原理：
         *   pseudo_arc_length[i] = 弧长从起点到 segment_point[i]
         *   目标：在累积弧长轴上均匀取样（每 cps_dist 米取一个点）
         *   对于累积弧长 L，落在 [pseudo_arc_length[i], pseudo_arc_length[i+1]] 之间时，
         *   用线性插值得到对应位置：
         *     point = segment_point[i] + (L - length[i]) / (length[i+1] - length[i]) * (segment_point[i+1] - segment_point[i])
         */
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5;  // 目标控制点间距（均匀采样间隔）
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;  // 二分搜索：逐步缩小目标间距，直到控制点数量足够
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              // 线性插值：在弧长轴上均匀取样，对应到空间位置
              point_set.push_back(
                  (sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);  // 强制将目标点加入（保证终点精确）
        } while (point_set.size() < 7);

        // 端点导数约束：继承当前轨迹的状态，到达目标时为零加速度
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // 异常检测：如果控制点数量异常多（说明伪弧长过大或采样异常），强制切换到多项式初始化
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // ========== STEP 1 收尾：B-spline 参数化 + A* 搜索扰动 ==========

    /* B-spline 逆参数化（给定路径点和导数约束，反求控制点）
     *
     * parameterizeToBspline() 的数学原理：
     *   给定 K+2 个路径点 {P_0, P_1, ..., P_{K+1}} 和端点导数约束，
     *   求出 N=K+6 个控制点 {C_0, ..., C_{N-1}}，
     *   使得 3阶均匀 B-spline 恰好通过所有路径点，且起点/终点处满足指定的速度/加速度
     *
     * 约束条件数量分析：
     *   - 路径点约束：K+2 个点
     *   - 起点速度约束：1 个（3维）
     *   - 起点加速度约束：1 个（3维）
     *   - 终点速度约束：1 个（3维）
     *   - 终点加速度约束：1 个（3维）
     *   共计：(K+2) + 12 = K+14 个约束
     *   3阶 B-spline 的控制点数 N = (K+2) + 6 = K+8
     *   每个控制点 3 个自由度 → 3N = 3(K+8) = 3K+24 个变量
     *   约束数 K+14 < 3K+24，当 K>=5 时系统是超定的，有唯一解
     */
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    /* A* 搜索器扰动（A* Path Searching）
     *
     * 问题：初始路径（多项式或轨迹延伸）完全基于几何，不考虑障碍物。
     *       如果路径穿过障碍物，直接用于优化会失败。
     *
     * 解决：调用 initControlPoints()，其内部：
     *   1. 沿初始路径逐个检查控制点是否在自由空间
     *   2. 对碰撞的控制点，以其为起点运行 A* 搜索，找到一条到目标点的无碰撞路径
     *   3. 用 A* 路径上的点替换/扰动原始控制点，使其通过无障碍区域
     *
     * 注意：A* 搜索只在局部区域（控制点附近）运行，开销很小
     *       它不要求 A* 路径完全替代原路径，只需要"拉拽"碰撞的控制点
     */
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;

    // 可视化调试信息（RViz 中显示）
    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_pathes, vis_id);

    t_start = ros::Time::now();

    /****************************************************************************
     * STEP 2: 优化 —— B-spline 后端优化（Back-End）
     *
     * 目标：在前端路径的基础上，通过数值优化找到一个"最优"的 B-spline 轨迹，
     *       同时满足：避障安全、动力学可行、轨迹平滑
     *
     * 优化算法：L-BFGS（Limited-memory BFGS）
     *   - BFGS 是一个拟牛顿方法，通过迭代更新 Hessian 矩阵的近似来求解优化问题
     *   - L-BFGS 限制只存储最近 m 次迭代的信息，适合大规模优化
     *   - 每次迭代 O(n)，其中 n 是控制点数量（通常十几到几十个），非常高效
     *
     * 优化变量：所有中间控制点的坐标（起点和终点的控制点固定）
     *   x = [c_1.x, c_1.y, c_1.z, c_2.x, c_2.y, c_2.z, ..., c_{N-2}.x, ...]  (3*(N-2) 维)
     *
     * 代价函数（总代价 = 各分量的加权和）：
     *   J_total = λ1 * J_smooth + λ2 * J_safe + λ3 * J_feasibility
     *
     *   (1) 平滑代价 J_smooth（最小化 Jerk / 加加速度）
     *       - 定义：轨迹三阶导数（加加速度）的平方积分
     *       - 物理意义：Jerk 越小，轨迹越"顺滑"，无人机运动越自然
     *       - 公式：J_smooth = ∫|p'''(t)|^2 dt
     *       - 特性：Jerk 仅依赖于控制点，与速度/加速度约束解耦
     *       - 实现：对 B-spline 的控制点做三阶差分，计算相邻控制点的三阶差分平方和
     *
     *   (2) 安全代价 J_safe（与障碍物保持距离）
     *       - 定义：每个控制点到最近障碍物的加权距离
     *       - 公式：J_safe = Σ max(0, d_safe - dist(cp_i, obstacle))^2
     *       - 其中 d_safe 是安全膨胀半径，dist() 是到最近障碍物的欧氏距离
     *       - 特性：代价在安全距离之外为零，进入安全距离后迅速增长
     *
     *   (3) 动力学可行性代价 J_feasibility（速度/加速度不超限）
     *       - 定义：对超出速度/加速度限制的控制点施加惩罚
     *       - 公式：J_feas = Σ (max(0, |v_i| - v_max)^2 + max(0, |a_i| - a_max)^2)
     *       - 其中 v_i = (cp_{i+1} - cp_i) / ts（控制点差分近似速度）
     *         a_i   = (cp_{i+2} - 2*cp_{i+1} + cp_i) / ts^2（控制点差分近似加速度）
     *       - 特性：仅在超限时产生代价，不影响正常范围内的控制点
     *
     * 梯度计算：
     *   L-BFGS 需要代价函数的梯度 ∂J/∂cp_i
     *   通过自动微分或解析求导计算，效率很高
     *
     * 终止条件：
     *   - 最大迭代次数（通常 500 次）
     *   - 最大运行时间（通常 0.15 秒）
     *   - 梯度范数 < 阈值（收敛判断）
     *
     * 退出机制：
     *   BsplineOptimizeTrajRebound() 内部在每次迭代时调用 check_collision_and_rebound()，
     *   如果发现优化过程中某控制点进入障碍物，会触发早期退出（STOP_FOR_REBOUND），
     *   此时返回 false，表示前端路径与障碍物过于接近，优化失败
     ****************************************************************************/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // 优化失败：通常意味着 A* 扰动后的路径仍然"太差"（大部分穿过障碍物）
      // 此时增加连续失败计数，FSM 会触发更多重试（使用随机多项式）
      continous_failures_count_++;
      return false;
    }

    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /****************************************************************************
     * STEP 3: 细化 —— 时间重分配（Feasibility Refinement）
     *
     * 问题：STEP 2 的优化过程改变了控制点位置，但时间参数 ts 保持不变。
     *       优化后的轨迹可能出现速度/加速度超出限制的情况（尤其是"拉直"绕过障碍物后）。
     *
     * 解决：检查并修复动力学可行性
     *
     *   checkFeasibility() 的原理：
     *     沿 B-spline 密集采样速度/加速度，找到全局最大超限比例 ratio
     *     ratio > 1 表示超限，ratio = 1.3 表示速度/加速度达到了限制的 1.3 倍
     *
     *   时间重分配（reparamBspline）的原理：
     *     核心思想：时间与速度成反比
     *       v ∝ 1/t  →  如果 v 超限 1.3 倍，将时间延长 1.3 倍即可
     *
     *     具体做法：
     *       1. 调用 bspline.lengthenTime(ratio)，将 B-spline 的时长乘以 ratio
     *          （通过重新计算节点向量实现，不改变控制点数量）
     *       2. 用新的时间跨度重新均匀采样控制点（保持控制点数量不变）
     *       3. 调用 BsplineOptimizeTrajRefine() 重新优化（细化版，λ4 曲线拟合代价权重更高）
     *       4. 曲线拟合代价 J_fitness：使新控制点尽量接近原始控制点，避免过度变形
     *
     *     为什么需要重新优化？
     *       因为延长时长后，采样得到的控制点位置会略有变化（由于 B-spline 的节点跨度变了）
     *       需要重新优化来调整控制点，消除碰撞并恢复平滑性
     ****************************************************************************/

    // 用优化后的控制点构建 B-spline，设置物理极限
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;  // 超限比例：velocity_max_actual / velocity_max_desired
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      // 动力学约束检查失败：需要重新分配时间参数
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);  // 用细化后的控制点重建轨迹
    }

    if (!flag_step_2_success)
    {
      // 细化失败：通常是因为重新分配时间后，延长后的轨迹仍然穿过障碍物
      // 这可能偶尔发生（障碍物在飞行中途突然出现）
      // 打印警告但不判定为完全失败，等待下一轮 FSM 重规划
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appears occasionally. "
             "But if continuously appearing, increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return false;
    }

    t_refine = ros::Time::now() - t_start;

    // ========== 保存结果：更新局部轨迹数据 ==========
    // updateTrajInfo() 将位置 B-spline 存入 local_data_，
    // 并同时计算派生量：速度 B-spline（位置 B-spline 的一阶导）、加速度 B-spline（二阶导）
    // 这些派生量供 traj_server（轨迹执行器）使用：速度 → 期望推力方向，加速度 → 期望姿态
    updateTrajInfo(pos, ros::Time::now());

    // 打印耗时统计（绿色背景高亮）
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m, "
         << "optimize:" << (t_init + t_opt).toSec() << ", "
         << "refine:" << t_refine.toSec() << endl;

    // 成功：重置连续失败计数，下次重试时使用确定性初始化
    continous_failures_count_ = 0;
    return true;
  }

  /**
   * @brief 紧急停止
   * @details 生成一个所有控制点都固定在 stop_pos 的悬停 B-spline 轨迹，
   *          使无人机立即停止运动并保持当前位置悬停。
   * @param stop_pos 悬停位置
   * @return 是否成功（始终返回 true）
   */
  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }
    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());
    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();
      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }
    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
    {
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    }
    else if (pos.cols() == 2)
    {
      gl_traj = PolynomialTraj::one_segment_traj_gen(
          start_pos, start_vel, start_acc,
          pos.col(1), end_vel, end_acc,
          time(0));
    }
    else
    {
      return false;
    }

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();
      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }
    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
    {
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    }
    else if (pos.cols() == 2)
    {
      gl_traj = PolynomialTraj::one_segment_traj_gen(
          start_pos, start_vel, start_acc,
          end_pos, end_vel, end_acc,
          time(0));
    }
    else
    {
      return false;
    }

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }

    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
