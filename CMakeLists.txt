cmake_minimum_required(VERSION 3.9)
project(pcd_merge)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_executable(pcd_merge src/pcd_merge_node.cpp)
ament_target_dependencies(
    pcd_merge
        rclcpp
        tf2_ros
        pcl_ros
        sensor_msgs
        pcl_conversions
)

install(
    TARGETS
        pcd_merge
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
        launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
