// Provide (x,y,z) for inverse kinematics of the arm.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  // auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // print current pose
  // geometry_msgs::msg::Pose current_pose =
  //   move_group_interface.getCurrentPose().pose;

  // // Print the current pose of the end effector
  // RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
  //   current_pose.position.x,
  //   current_pose.position.y,
  //   current_pose.position.z,
  //   current_pose.orientation.x,
  //   current_pose.orientation.y,
  //   current_pose.orientation.z,
  //   current_pose.orientation.w);
//   move_group_interface.setRandomTarget();
//Plan the motion and then move the group to the sampled target
  // move_group_interface.move();

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.562170;
    msg.orientation.y = -0.414440;
    msg.orientation.z = 0.453448;
    msg.orientation.w = 0.553705;
    msg.position.x = -0.167553;
    msg.position.y = -0.064641;
    msg.position.z = 0.365909;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
    
  // Execute the plan
  if(success) {
    // move_group_interface.execute(plan);
    move_group_interface.move();
    RCLCPP_INFO(node->get_logger(), "Entered success loop");
  } else {
    // RCLCPP_ERROR(logger, "Planning failed!");
    RCLCPP_INFO(node->get_logger(), "Entered fail loop");

  }
  RCLCPP_INFO(node->get_logger(), "Entered after if loop");

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}


