#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath> 

using namespace std::chrono_literals;

class ComparePoseNode : public rclcpp::Node{
  bool flat =false;
  public:
  ComparePoseNode() : Node("compare_pose_node"){
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&ComparePoseNode::goal_callback, this, std::placeholders::_1));

    // amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //     "/amcl_pose", 10, std::bind(&ComparePoseNode::amcl_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ComparePoseNode::odom_callback, this, std::placeholders::_1));
       
    load_map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_goal_pose_ = msg->pose;
    flat = true;
    RCLCPP_INFO(this->get_logger(), "Goal Pose: (%.2f, %.2f)", current_goal_pose_.position.x, current_goal_pose_.position.y);
  }

  // void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  // {
  //   current_amcl_pose_ = msg->pose.pose;
  //   compare_poses();
  //   RCLCPP_INFO(this->get_logger(), "AMCL Position: (%.2f, %.2f)", current_amcl_pose_.position.x, current_amcl_pose_.position.y);
  // }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_pose_ = msg->pose.pose;
    compare_poses();
    RCLCPP_INFO(this->get_logger(), "Odom Position: (%.2f, %.2f)", current_odom_pose_.position.x, current_odom_pose_.position.y);
  }

  void compare_poses()
  {
    if (flat)
    {
      double distance;
      distance = sqrt(pow(current_odom_pose_.position.x-current_goal_pose_.position.x,2)+pow(current_odom_pose_.position.y-current_goal_pose_.position.y,2));
      RCLCPP_INFO(this->get_logger(), "Distance: (%.2f)",distance);
      if (distance < 1.99)
      {
        load_map();
        RCLCPP_INFO(this->get_logger(), "Map switched");
      }
    }
    
  }

  void load_map()
  {
    RCLCPP_INFO(this->get_logger(), "Loading new map");
    auto client = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = "/home/keyu/map2.yaml";
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service load_map");
      return;
    }
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Result: %d", result->result);
  }

  // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;

  // geometry_msgs::msg::Pose current_amcl_pose_;
  geometry_msgs::msg::Pose current_odom_pose_;
  geometry_msgs::msg::Pose current_goal_pose_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ComparePoseNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
