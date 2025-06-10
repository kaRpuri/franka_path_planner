#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_path_node");
  
  // Try the new recommended group first
  moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");
  
  // Rest of your code stays the same...
  move_group.setMaxVelocityScalingFactor(0.5);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.4;
  target_pose1.orientation.w = 1.0;
  waypoints.push_back(target_pose1);

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = 0.3;
  target_pose2.position.y = 0.2;
  target_pose2.position.z = 0.6;
  target_pose2.orientation.x = 0.707;
  target_pose2.orientation.w = 0.707;
  waypoints.push_back(target_pose2);

  move_group.setMaxVelocityScalingFactor(0.2);
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if(fraction >= 0.9) {
    move_group.execute(trajectory);
    RCLCPP_INFO(node->get_logger(), "Path execution completed");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed (%.2f%% achieved)", fraction * 100.0);
  }

  rclcpp::shutdown();
  return 0;
}
