add_library(localization_quality_monitor_lib SHARED
  localization_quality_state_machine.cpp
  localization_quality_monitor.cpp
)

ament_target_dependencies(localization_quality_monitor_lib
  rclcpp
  sensor_msgs
)

install(TARGETS
  localization_quality_monitor_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
