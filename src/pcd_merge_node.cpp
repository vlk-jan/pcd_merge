#include "pcd_merge/pcd_merge.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      rclcpp::Node::make_shared("pcd_merge_node", rclcpp::NodeOptions());

  PCDMerge pcd_merge(node);
  rclcpp::Rate rate(pcd_merge.getHz());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    pcd_merge.update();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
