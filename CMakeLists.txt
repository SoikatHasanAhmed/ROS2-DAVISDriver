cmake_minimum_required(VERSION 3.8)
project(davisdriver)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libcaer REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(CURL REQUIRED)  # For curl-related linking (e.g., GDAL, netCDF)

# Declare all local .msg files
set(msg_files
  "msg/Event.msg"
  "msg/EventArray.msg"
  "msg/ImageWithTimestamp.msg"
)

# Generate interface code from message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

# Export runtime dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs sensor_msgs builtin_interfaces)

# Build the driver node
add_executable(davis_driver src/davis_driver.cpp)

ament_target_dependencies(davis_driver
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  libcaer
  rosidl_typesupport_cpp
)

# Link generated interface support
rosidl_target_interfaces(davis_driver
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Include directories
target_include_directories(davis_driver PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${libcaer_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(davis_driver
  ${OpenCV_LIBS}
  ${libcaer_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  caer
  ${CURL_LIBRARIES}
)

# Install executable
install(TARGETS
  davis_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: add linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark as ament package
ament_package()
