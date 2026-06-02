#include "explore/explore_normal.h"
#include <thread>
#include <cmath>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  return (dx * dx + dy * dy) < 0.0001; // < 0.01m
}

namespace explore
{

Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , current_state_(NavState::IDLE)
  , last_markers_count_(0)
{
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  //this->declare_parameter<std::string>("robot_base_frame", "base_link");

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  search_ = frontier_exploration::FrontierSearch(
      costmap_client_.getCostmap(), potential_scale_, gain_scale_, min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/frontiers", 10);
  }
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", rclcpp::QoS(10).transient_local()); 
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10, std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  // --- 初始化目标点发布器 ---
  goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  // --- 初始化 10Hz 状态监控看门狗 ---
  watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), 
      std::bind(&Explore::stateWatchdogCallback, this));

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      auto transformStamped = tf_buffer_.lookupTransform(map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform: %s", ex.what());
      return_to_init_ = false;
    }
  }

  // 立即开始探索
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

// ---------------------------------------------------------
// 核心监控逻辑：利用 TF 实现闭环状态机
// ---------------------------------------------------------
geometry_msgs::msg::Point Explore::getCurrentRobotPose()
{
  geometry_msgs::msg::Point pose;
  try {
    auto transform = tf_buffer_.lookupTransform(
        costmap_client_.getGlobalFrameID(), robot_base_frame_, tf2::TimePointZero);
    pose.x = transform.transform.translation.x;
    pose.y = transform.transform.translation.y;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 2000, "TF Lookup Error in Watchdog: %s", ex.what());
  }
  return pose;
}

void Explore::stateWatchdogCallback()
{
  if (current_state_ != NavState::ON_GOAL) {
    return;
  }

  geometry_msgs::msg::Point current_pose = getCurrentRobotPose();
  
  // 1. 判定是否到达目标附近 (NEAR)
  double dist_to_goal = std::hypot(current_pose.x - current_goal_.x, 
                                   current_pose.y - current_goal_.y);
  
  if (dist_to_goal < 0.2) { 
    if (is_returning_home_) {
      // 如果是回家的任务，到达后直接下班，不要重新规划
      RCLCPP_INFO(logger_, "[Watchdog] Robot has successfully returned to initial pose. Shutting down.");
      current_state_ = NavState::IDLE;
      // 可选：如果希望到家后直接退出节点，可以加上 rclcpp::shutdown();
      is_returning_home_ = false; // 重置回家标记
      return; 
    } else {
      // 如果是正常的探索任务，到达后继续寻找下一个点
      RCLCPP_INFO(logger_, "[Watchdog] Robot is NEAR the goal (%.2fm). Triggering new plan.", dist_to_goal);
      current_state_ = NavState::NEAR;
      makePlan(); 
      return;
    }
  }

  // 2. 判定是否被困 (STUCK) - 每 3 秒检查一次累计位移
  auto current_time = this->now();
  if ((current_time - last_check_time_).seconds() > 3.0) {
    
    double dist_moved = std::hypot(current_pose.x - last_check_pose_.x, 
                                   current_pose.y - last_check_pose_.y);
                                   
    if (dist_moved < 0.15) { 
      // 如果 3 秒内移动不到 15cm，触发 STUCK 处理逻辑
      if (dist_to_goal < 0.4) {
        RCLCPP_INFO(logger_, "[Watchdog] Robot stuck near target (%.2fm). Considered REACHED.", dist_to_goal);
      } else {
        RCLCPP_WARN(logger_, "[Watchdog] Robot is STUCK! Blacklisting current goal and re-planning.");
        frontier_blacklist_.push_back(current_goal_); 
      }
      
      current_state_ = NavState::STUCK;
      makePlan();
      return;
    }

    // 正常移动，重置锚点时间戳与坐标
    last_check_pose_ = current_pose;
    last_check_time_ = current_time;
  }
}

// ---------------------------------------------------------
// 目标发布逻辑：封装 Topic
// ---------------------------------------------------------
void Explore::publishGoal(const geometry_msgs::msg::Point& target)
{
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.frame_id = costmap_client_.getGlobalFrameID();
  goal_msg.header.stamp = this->now();
  goal_msg.pose.position = target;
  goal_msg.pose.orientation.w = 1.0; 

  RCLCPP_INFO(logger_, "Publishing new target to /goal_pose: (%.2f, %.2f)", target.x, target.y);
  goal_publisher_->publish(goal_msg);
  
  // 重置状态机参数，将控制权交回给 Watchdog
  current_goal_ = target;
  last_check_pose_ = getCurrentRobotPose();
  last_check_time_ = this->now();
  current_state_ = NavState::ON_GOAL; 
}

// ---------------------------------------------------------
// 前沿探索与规划逻辑
// ---------------------------------------------------------
void Explore::makePlan()
{
  auto pose = costmap_client_.getRobotPose();
  auto frontiers = search_.searchFrom(pose.position);

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // 寻找不在黑名单中的前沿
  auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                                   [this](const frontier_exploration::Frontier& f) {
                                     return goalOnBlacklist(f.centroid);
                                   });
                                   
  if (frontier == frontiers.end()) {
    RCLCPP_INFO(logger_, "All frontiers traversed/tried out (blacklisted), stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  if (resuming_) {
    resuming_ = false;
  }

  // 如果目标发生实质性改变，调用发布逻辑
  if (!same_point(current_goal_, frontier->centroid) || current_state_ != NavState::ON_GOAL) {
    publishGoal(frontier->centroid);
  }
}

// ---------------------------------------------------------
// 其他辅助与控制逻辑 (简化展示)
// ---------------------------------------------------------
void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  // 加上这个标记，告诉全系统我们正在回家
  is_returning_home_ = true; 
  publishGoal(initial_pose_.position);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) resume();
  else stop();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  current_state_ = NavState::IDLE;
  is_returning_home_ = false; // 确保新任务开始前是干净的
  makePlan();
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  current_state_ = NavState::IDLE;

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  is_returning_home_ = false; // 确保新任务开始前是干净的
  makePlan();
}

void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{
  // 保持原有渲染逻辑不变
  // ...
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore::Explore>());
  rclcpp::shutdown();
  return 0;
}