#pragma once
#ifndef PCD_MERGE_HPP
#define PCD_MERGE_HPP

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

class PCDMerge {
public:
  PCDMerge(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    num_clouds_ = nh_->declare_parameter<int>("num_clouds", 2);
    if (num_clouds_ < 2) {
      RCLCPP_ERROR(nh_->get_logger(), "num_clouds must be >= 2");
    }
    int queue_size = nh_->declare_parameter<int>("queue_size", 2);
    queue_size = std::max(1, queue_size);
    target_frame_ =
        nh_->declare_parameter<std::string>("target_frame", "base_link");
    hz_ = nh_->declare_parameter<double>("hz", 10.0);

    clouds_.resize(num_clouds_);

    for (int i = 0; i < num_clouds_; ++i) {
      std::stringstream ss;
      ss << "input_cloud_" << i;
      input_cloud_subs_.push_back(
          nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
              ss.str(), queue_size,
              [this,
               i](const std::shared_ptr<const sensor_msgs::msg::PointCloud2>
                      &msg) { this->receiveCloud(msg, i); }));
    }

    cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "out_cloud", queue_size);
  }

  ~PCDMerge() {}

  void update() {
    if (cloud_pub_->get_subscription_count() == 0 || num_clouds_ < 2) {
      return;
    }

    if (clouds_.size() == 0) {
      RCLCPP_WARN_THROTTLE(
          nh_->get_logger(), *nh_->get_clock(), 5000,
          "No clouds received yet, waiting for data on input topics.");
      return;
    }

    // Merge clouds
    cloud_lock_ = true;
    sensor_msgs::msg::PointCloud2 merged_cloud;
    for (int i = 0; i < num_clouds_; ++i) {
      if (clouds_[i].data.size() == 0) { // Cloud not received
        RCLCPP_WARN(nh_->get_logger(), "Cloud %d not received", i);
        continue;
      }

      if (merged_cloud.data.size() == 0) { // First valid cloud
        merged_cloud = clouds_[i];
      } else {
        pcl::concatenatePointCloud(merged_cloud, clouds_[i], merged_cloud);
      }
      clouds_[i] = sensor_msgs::msg::PointCloud2(); // Clear cloud after merging
    }
    cloud_lock_ = false;

    // Publish merged cloud
    merged_cloud.header.stamp = nh_->get_clock()->now();
    merged_cloud.header.frame_id = target_frame_;
    cloud_pub_->publish(merged_cloud);
  }

  double getHz() { return hz_; }

  void receiveCloud(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      const int index) {
    sensor_msgs::msg::PointCloud2 temp_cloud;
    bool ret =
        pcl_ros::transformPointCloud(target_frame_, *input, temp_cloud, *tf_);
    if (!ret || cloud_lock_) {
      return;
    }
    clouds_[index] = temp_cloud;
  }

protected:
  rclcpp::Node::SharedPtr nh_;

  std::shared_ptr<tf2_ros::Buffer> tf_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      input_cloud_subs_;

  int num_clouds_;
  std::vector<sensor_msgs::msg::PointCloud2> clouds_;
  std::string target_frame_;
  double hz_;
  bool cloud_lock_ = false;
};

#endif // PCD_MERGE_HPP
