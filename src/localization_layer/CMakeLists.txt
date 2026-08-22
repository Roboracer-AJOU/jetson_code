cmake_minimum_required(VERSION 3.8)
project(localization_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install rviz config
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install helper scripts
  install(PROGRAMS
  scripts/map_auto_saver.py
  scripts/localization_initial_pose_setter.py
  scripts/vesc_wheel_odom.py
  scripts/scan_range_filter.py
  scripts/static_map_publisher.py
  scripts/setup_lidar_network.sh
  scripts/install_lidar_network.sh
  scripts/run_localization_rviz.sh
  scripts/localization_rviz.sh
  scripts/install_rviz_desktop_access.sh
  scripts/fix_rviz_x11_access.sh
  scripts/rviz-xhost-at-boot.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
