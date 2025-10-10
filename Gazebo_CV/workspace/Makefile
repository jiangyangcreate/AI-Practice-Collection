.PHONY: world sim img cam vel bridge go left right stop rv node start

# 启动完整世界
world:
	gz sim -r world.sdf
	

# 桥接摄像头和控制话题 (simple.sdf 的 /camera和/cmd_vel)
bridge:
	ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist

start:
	colcon build --packages-select learning_node
	ros2 run learning_node node_helloworld

# 单独桥接摄像头 (simple.sdf 的 /camera)
cam:
	ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
# 启动 rviz2
rv:
	rviz2

# 单独桥接控制话题 (simple.sdf 的 /cmd_vel)
vel:
	ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist

# 前进
go:
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转
left:
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 右转
right:
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 停止
stop:
	ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


