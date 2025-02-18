ros2 run gazebo_ros spawn_entity.py \
  -file "$(ros2 pkg prefix mars_plan)/share/mars_plan/models/turtlebot3_waffle/model.sdf" \
  -entity tb1 \
  -robot_namespace /tb1 \
  -x -1.5 \
  -y -0.5 \
  -z 0.01 \
  -Y 0.0 \
  -unpause
2 pkg prefix mars_plan)/share/