cd ~/ros2_ws  
colcon build --packages-select emergency_stop_pkg  
source install/setup.bash  　
ros2 launch emergency_stop_pkg emergency_stop_launch.py  
#異なるターミナルで  
ros2 topic echo /rosout
