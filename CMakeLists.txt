cmake_minimum_required(VERSION 3.8)
project(cobot_pjt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ServeBeverage.srv"
)

ament_python_install_package(peter)

install(
  PROGRAMS
    scripts/beverage_test
    scripts/beverage_test_srv
    scripts/get_current_pose
    scripts/go_home
    scripts/gripper_test
    scripts/jog_complete
    scripts/move_plate
    scripts/paddle_ocr_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
