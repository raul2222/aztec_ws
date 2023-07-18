cmake_minimum_required(VERSION 3.5)
project(synexens_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(camera_info_manager REQUIRED)

############################################
############ Synexens SDK start ############
############################################
# find_package(OpenCV REQUIRED)
# set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_videoio opencv_calib3d)
# set(OPENCV_LIB_DIR ${SYN_SDK_PATH}/opencv-4.4.0)
set(SYN_LIBS csreconstruction SonixCamera synexens3)
set(SYN_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk)
set(SYN_INCLUDE_DIR ${SYN_SDK_PATH}/include)
set(SYN_LIB_DIR ${SYN_SDK_PATH}/lib)

link_directories(
  ${SYN_LIB_DIR}
)

##########################################
############ Synexens SDK END ############
##########################################

add_executable(synexens_ros_node src/synexens_ros_node.cpp src/synexens_ros_device.cpp src/synexens_ros_params.cpp src/synexens_calibration_transform_data.cpp)

target_include_directories(synexens_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  synexens_ros_node
  "geometry_msgs"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "angles"
  "camera_info_manager"
)

target_link_libraries(
  synexens_ros_node
  ${SYN_LIBS}
)

# install(
#   DIRECTORY lib/
#   DESTINATION lib
# )

install(
  DIRECTORY ${SYN_LIB_DIR}/
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(TARGETS synexens_ros_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
