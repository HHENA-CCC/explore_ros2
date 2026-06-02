#ifndef EXPLORE_H_
#define EXPLORE_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "explore_lite_msgs/msg/explore_status.hpp"
#include "explore/frontier_search.h"
#include <explore/costmap_client.h>

namespace explore
{

// 定义导航监控状态
enum class NavState {
  IDLE,      // 刚启动或无目标状态
  ON_GOAL,   // 正在前往目标的路上
  NEAR,      // 已到达目标附近
  STUCK      // 被困在原地
};

class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

private:
  void makePlan();
  void visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);
  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void returnToInitialPose();
  bool is_returning_home_ = false;

  // --- 重构引入的新函数 ---
  void publishGoal(const geometry_msgs::msg::Point& target);
  geometry_msgs::msg::Point getCurrentRobotPose();
  void stateWatchdogCallback();

  rclcpp::Logger logger_;

  // TF 相关
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string robot_base_frame_;

  frontier_exploration::FrontierSearch search_;

  // --- 重构引入的 Topic 发布器与定时器 ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  
  // 原有的发布与订阅
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;

  // --- 状态机监控变量 ---
  NavState current_state_;
  geometry_msgs::msg::Point current_goal_;
  geometry_msgs::msg::Point last_check_pose_;
  rclcpp::Time last_check_time_;
  Costmap2DClient costmap_client_;

  // 参数与控制变量
  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  bool visualize_;
  bool return_to_init_;
  bool resuming_ = false;
  float planner_frequency_;
  float potential_scale_, orientation_scale_, gain_scale_;
  size_t last_markers_count_;
  
  geometry_msgs::msg::Pose initial_pose_;
};

}  // namespace explore

#endif