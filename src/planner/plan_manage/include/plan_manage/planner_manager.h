/**
 * @file planner_manager.h
 * @brief EGOPlannerManager 类的声明
 *
 * EGOPlannerManager 是 EGO-Planner 的核心管理器，负责整合所有底层规划算法模块，
 * 并提供统一的对内（FSM）和对外（其他节点）规划接口。
 *
 * 它管理以下核心组件：
 *   - GridMap：栅格地图，管理环境的占据状态，提供碰撞查询
 *   - AStar：动态 A* 路径搜索器，在栅格地图中搜索初始可行路径
 *   - BsplineOptimizer：B-spline 轨迹优化器，将初始路径优化为满足约束的平滑轨迹
 *
 * 核心规划流程（reboundReplan）：
 *   1. 初始化：从多项式/当前轨迹采样生成初始路径点集
 *   2. A* 搜索：调整初始路径使其通过无障碍区域
 *   3. B-spline 优化：最小化代价函数（碰撞 + 平滑 + 动力学约束）
 *   4. 时间细化：如速度/加速度超限，重新分配时间参数
 */

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  /**
   * @class EGOPlannerManager
   * @brief EGO-Planner 核心规划管理器
   *
   * 封装了栅格地图（A）、A* 路径搜索（B）和 B-spline 轨迹优化（C）三个核心模块。
   * 通过 plan_container.hpp 中定义的数据结构（PlanParameters、LocalTrajData、GlobalTrajData）
   * 存储规划参数和轨迹数据。
   */
  class EGOPlannerManager
  {
    // SECTION stable
  public:
    /** @brief 默认构造函数 */
    EGOPlannerManager();

    /** @brief 析构函数 */
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* 主规划接口 */

    /**
     * @brief 回弹式重规划算法（核心）
     *
     * 整合 A* 搜索和 B-spline 优化的前端-后端规划算法。
     * 输入起点和局部目标，输出优化后的 B-spline 轨迹。
     *
     * @param start_pt     起点位置
     * @param start_vel    起点速度
     * @param start_acc    起点加速度
     * @param end_pt       局部目标位置
     * @param end_vel      局部目标速度
     * @param flag_polyInit     是否使用多项式初始化
     * @param flag_randomPolyTraj 是否使用随机多项式
     * @return 规划是否成功
     */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);

    /**
     * @brief 紧急停止
     * @param stop_pos 悬停位置
     */
    bool EmergencyStop(Eigen::Vector3d stop_pos);

    /**
     * @brief 根据多个航点生成全局多项式轨迹
     * @param start_pos  起始位置
     * @param start_vel  起始速度
     * @param start_acc  起始加速度
     * @param waypoints  航点列表
     * @param end_vel    终止速度
     * @param end_acc    终止加速度
     */
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /**
     * @brief 初始化所有规划子模块
     * @param nh ROS 节点句柄
     * @param vis 可视化管理器（可选）
     */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    // ========== 公共数据成员 ==========

    /** @brief 规划算法参数（速度/加速度/控制点距离/规划视野等） */
    PlanParameters pp_;

    /** @brief 当前正在执行的局部轨迹数据（包含位置/速度/加速度 B-spline） */
    LocalTrajData local_data_;

    /** @brief 全局参考轨迹数据（由 planGlobalTraj/planGlobalTrajWaypoints 生成） */
    GlobalTrajData global_data_;

    /** @brief 栅格地图管理器（接收点云/深度图，维护占据状态） */
    GridMap::Ptr grid_map_;

  private:
    /* 内部规划算法模块（智能指针管理） */

    PlanningVisualization::Ptr visualization_;  // 可视化管理器

    /** @brief B-spline 优化器（负责代价函数定义和优化求解） */
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    /** @brief 连续规划失败的次数（用于自适应调整探索策略） */
    int continous_failures_count_{0};

    /**
     * @brief 更新局部轨迹信息
     * @param position_traj 优化后的位置 B-spline
     * @param time_now 轨迹起始时间
     */
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    /**
     * @brief B-spline 时间重参数化
     * @details 当轨迹超出动力学约束时，延长时长并重新采样控制点
     */
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    /**
     * @brief 轨迹细化算法
     * @details 时间重分配 + 重新优化，用于处理速度/加速度超限情况
     */
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    /** @brief 智能指针类型别名，便于外部创建实例 */
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
