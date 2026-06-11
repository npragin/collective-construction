cmake_minimum_required(VERSION 3.20)
project(cc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/Stockpiles.msg"
  "msg/StructurePlan.msg"
  "action/RetrievalTask.action"
  "action/ManipulationTask.action"
  "srv/CheckPlacement.srv"
  "srv/LogPickedTag.srv"
  "action/CorrectionTask.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
