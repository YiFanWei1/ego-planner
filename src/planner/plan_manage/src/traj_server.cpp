/**
 * @file traj_server.cpp
 * @brief 轨迹服务器节点实现
 *
 * traj_server 是 EGO-Planner 与下游控制器之间的桥梁。
 * 主要职责：
 *   1. 订阅 ego_planner_node 发布的 /planning/bspline（B-spline 轨迹消息）
 *   2. 以 100Hz 的频率将 B-spline 轨迹转换为 quadrotor_msgs::PositionCommand 位置指令
 *   3. 动态计算偏航角（yaw），使无人机机头朝向轨迹前方方向
 *
 * 输出话题 /position_cmd 被 so3_control 节点订阅，最终转换为电机控制指令。
 */

#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

// 发布器：向 so3_control 节点发送位置指令
ros::Publisher pos_cmd_pub;

// quadrotor_msgs::PositionCommand 消息结构：
//   position (Vector3): 目标位置
//   velocity (Vector3): 目标速度
//   acceleration (Vector3): 目标加速度
//   yaw (float): 目标偏航角
//   yaw_dot (float): 偏航角变化率
//   kx[3], kv[3]: 位置/速度增益（用于上层控制，此处均设为零，下层自行计算）
quadrotor_msgs::PositionCommand cmd;

// 位置/速度增益参数（此处硬编码为零，实际由下层控制器计算）
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

// ========== 轨迹接收状态 ==========
bool receive_traj_ = false;    // 是否已接收到有效轨迹
vector<UniformBspline> traj_;    // 存储 B-spline 轨迹集合：[0]=位置, [1]=速度(导数), [2]=加速度(二次导数)
double traj_duration_;          // 轨迹总时长（秒）
ros::Time start_time_;          // 轨迹开始时间（ROS absolute time）
int traj_id_;                   // 轨迹唯一 ID（用于验证）

// ========== 偏航角控制相关变量 ==========
double last_yaw_, last_yaw_dot_;  // 上一次的偏航角和偏航角速度（用于增量式 yaw 计算）
double time_forward_;             // 前向看时间（秒）：沿轨迹向前看多远来决定偏航方向

/**
 * @brief B-spline 轨迹回调函数
 * @details 收到 /planning/bspline 消息后，将 ROS 消息解析为 UniformBspline 对象，
 *          并计算位置/速度/加速度的 B-spline 表示（通过求导）。
 * @param msg ego_planner::Bspline 消息，包含控制点坐标和节点向量
 */
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // 从消息中提取控制点矩阵（3 行，pos_pts.size() 列）
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  // 提取节点向量（knot vector）
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  // 提取控制点坐标
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  // 使用控制点和节点向量构造位置 B-spline（order=3 即为三次 B-spline）
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);  // 显式设置节点向量

  // 记录轨迹元信息（起始时间、ID）
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // 清空轨迹容器，并依次存入：位置轨迹、速度轨迹（一次导数）、加速度轨迹（二次导数）
  traj_.clear();
  traj_.push_back(pos_traj);                        // 位置
  traj_.push_back(traj_[0].getDerivative());      // 速度 = 位置的一阶导数
  traj_.push_back(traj_[1].getDerivative());      // 加速度 = 速度的一阶导数

  traj_duration_ = traj_[0].getTimeSum();  // 获取轨迹总时长
  receive_traj_ = true;                     // 标记已接收到有效轨迹，使 cmdCallback 可以开始发布
}

/**
 * @brief 计算当前时刻的目标偏航角及偏航角速度
 *
 * 采用"前向看"策略：沿轨迹向前看 time_forward_ 秒，计算无人机应朝向的目标方向角。
 * 偏航角变化率受 YAW_DOT_MAX_PER_SEC 限制，防止偏航角突变导致无人机剧烈翻滚。
 * 支持 -PI 到 PI 的角度跨越（通过判断是否穿越了正负 PI 边界）。
 *
 * @param t_cur    相对于轨迹起始时间的当前时刻（秒）
 * @param pos      当前位置（用于计算前向方向）
 * @param time_now 当前 ROS 时间
 * @param time_last 上一次调用的 ROS 时间（用于计算时间差）
 * @return pair(yaw, yaw_dot) 目标偏航角和偏航角速度
 */
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;  // 最大偏航角速度：每秒 PI 弧度（180°/s）
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  // 计算前向目标点：
  //   - 如果当前时刻 + 前向时间 < 轨迹总时长，取轨迹上 t_cur + time_forward_ 处的位置
  //   - 否则取轨迹终点
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                            : traj_[0].evaluateDeBoorT(traj_duration_) - pos;

  // 计算目标偏航角：使用 atan2(y, x) 从方向向量的 xy 分量得到航向角
  // 如果方向向量过短（< 0.1m），使用上一次的偏航角（避免除零或噪声）
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;

  // 计算单次允许的最大偏航角变化量：max_yaw_change = 最大偏航角速度 * 时间间隔
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();

  // 偏航角从 -PI 到 PI 是连续的，需要处理角度跨越问题
  // 以下逻辑分为三个分支：
  //   1. yaw_temp - last_yaw_ > PI：目标角从 last_yaw_ 正向跨越了 PI（如从 +179° 到 -179°）
  //   2. yaw_temp - last_yaw_ < -PI：目标角从 last_yaw_ 负向跨越了 -PI
  //   3. 其他：正常情况，直接计算变化量并限幅

  if (yaw_temp - last_yaw_ > PI)
  {
    // 正向跨越 PI（例如从 179° 到 -179°，实际变化量只有 -2°）
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      // 实际需要减少的角度超过了允许的最大值，只能按最大值减少
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;  // 允许跨越
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    // 负向跨越 -PI（例如从 -179° 到 +179°，实际变化量只有 +2°）
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    // 正常情况（无 PI 跨越）
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      // 需要顺时针旋转超过限制，按最大值顺时针旋转
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      // 需要逆时针旋转超过限制，按最大值逆时针旋转
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      // 变化量在允许范围内，直接跟随
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  // 低通滤波器（LPF）：平滑偏航角和偏航角速度的变化
  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // 简单均值滤波
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;  // 偏航角速度也做滤波

  // 更新历史值，供下次调用使用
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

/**
 * @brief 指令定时回调（100Hz，每 10ms 执行一次）
 * @details 从 B-spline 轨迹中采样当前位置/速度/加速度，
 *          计算目标偏航角，组装 PositionCommand 消息并发布。
 * @param e ROS 定时器事件
 */
void cmdCallback(const ros::TimerEvent &e)
{
  // 如果尚未收到轨迹，不发布任何指令（下游控制器将维持上一条指令）
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  // 计算当前时刻相对于轨迹起始时间的偏移
  double t_cur = (time_now - start_time_).toSec();

  // 初始化状态变量
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
      acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    // 轨迹正常执行区间：沿 B-spline 采样当前状态

    // 使用 De Boor 算法从 B-spline 中求取任意时刻的位置/速度/加速度
    // traj_[0]: 位置轨迹，traj_[1]: 速度轨迹，traj_[2]: 加速度轨迹
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** 计算偏航角 ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** 计算偏航角结束 ***/

    // 计算前向目标点（用于可视化或调试）
    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    // 轨迹执行完毕：悬停在轨迹终点
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;   // 保持最后偏航角
    yaw_yawdot.second = 0;          // 偏航角速度为零

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  // ========== 组装 PositionCommand 消息 ==========
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";  // 所有位置均为世界坐标系

  // 轨迹状态标志：READY 表示轨迹已就绪，可供控制器执行
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;     // 轨迹 ID，用于控制器验证

  // 填充目标位置/速度/加速度（xyz 分量）
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  // 填充偏航角和偏航角速度
  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  // 发布 PositionCommand 给下游 so3_control 节点
  pos_cmd_pub.publish(cmd);
}

/**
 * @brief 主函数：初始化 traj_server 节点
 */
int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;       // 全局节点句柄
  ros::NodeHandle nh("~");    // 私有节点句柄（用于读取本节点的参数）

  // ========== 订阅/发布设置 ==========

  // 订阅优化后的 B-spline 轨迹（来自 ego_planner_node）
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  // 发布 quadrotor 位置指令（给 so3_control 节点）
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  // 创建 100Hz 定时器，每 10ms 调用一次 cmdCallback
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  /* 控制增益参数（此处设为零，so3_control 节点自行计算增益） */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  // 从参数服务器读取前向看时间参数（秒）
  // 前向看时间越大，无人机越提前转向；越小则越"盯着"当前运动方向
  nh.param("traj_server/time_forward", time_forward_, -1.0);

  last_yaw_ = 0.0;       // 初始偏航角（朝向 x 轴正方向）
  last_yaw_dot_ = 0.0;   // 初始偏航角速度

  // 等待 1 秒确保所有节点连接就绪
  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  // 进入 ROS 主循环
  ros::spin();

  return 0;
}
