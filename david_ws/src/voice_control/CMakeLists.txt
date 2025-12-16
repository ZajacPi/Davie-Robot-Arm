find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MovementCommand.msg"
)

ament_export_dependencies(rosidl_default_runtime)
