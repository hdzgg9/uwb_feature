cmake_minimum_required(VERSION 3.8)
project(uwb_feature_gtsam_sim)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CircleFeature.msg"
  "msg/CircleFeatureArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(PROGRAMS
  scripts/uwb_range_sim
  scripts/circle_feature_detector
  scripts/gtsam_fusion
  scripts/scan_wander
  scripts/alignment_waiter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
