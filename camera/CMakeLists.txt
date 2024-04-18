cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Add executable
add_executable(WebcamPub src/WebcamPub.cpp)
add_executable(ImageServiceServer src/ImageServiceServer.cpp)
add_executable(CannyEdgeNode src/CannyEdgeNode.cpp)
add_executable(StillShotClient src/StillShotClient.cpp)

# Include directories
target_include_directories(WebcamPub PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(ImageServiceServer PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(CannyEdgeNode PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(StillShotClient PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
# Link libraries
ament_target_dependencies(WebcamPub
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
)

# Link libraries
ament_target_dependencies(ImageServiceServer
    rclcpp
    sensor_msgs
    std_srvs
    cv_bridge
    OpenCV
)

# Link libraries
ament_target_dependencies(CannyEdgeNode
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
)

# Link libraries
ament_target_dependencies(StillShotClient
    rclcpp
    std_srvs 
    sensor_msgs
    cv_bridge
    OpenCV
)

# Install
install(TARGETS
    WebcamPub
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    ImageServiceServer
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    CannyEdgeNode
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    StillShotClient
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
