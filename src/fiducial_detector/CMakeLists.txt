cmake_minimum_required(VERSION 3.8)
project(fiducial_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -march=native)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED
  COMPONENTS core imgproc highgui aruco calib3d)
find_package(Eigen3 REQUIRED)

# ── aruco_node executable ─────────────────────────────────────────────────────
add_executable(aruco_node
  src/main.cpp
  src/aruco.cpp
  src/visualization.cpp
  src/pose_estimator.cpp
  src/fps_monitor.cpp
  src/detector_parameters.cpp
  src/dictionary_manager.cpp
)

target_include_directories(aruco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(aruco_node
  rclcpp rclcpp_components
  std_msgs sensor_msgs geometry_msgs
  tf2 tf2_geometry_msgs
  cv_bridge image_transport
)

target_link_libraries(aruco_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS aruco_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
