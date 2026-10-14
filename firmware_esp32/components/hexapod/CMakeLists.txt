idf_component_register(
    SRCS "serial_bridge.cpp" "posture_engine.cpp" "leg_kinematics.cpp" "leg_config.cpp" "locomotion_task.cpp" "gait_engine.cpp" "body_kinematics.cpp" "robot_controller.cpp" "diagnostic.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos driver log ax12
)
