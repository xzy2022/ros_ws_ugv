Parameter Principles

Launch args stay for architecture-only switches: namespacing (robot_name), resource paths (world_file, rviz_config), and binary toggles (start_*, paused, use_sim_time). Changing these reshapes the running system, so they belong in XML.
YAML holds runtime/business parameters that nodes can reread without restarting ROS (topics, limits, logging knobs). Load them under the robot namespace so every node can grab ~param cleanly.