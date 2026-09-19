# Run Moveit

colcon build --packages-select arm_description arm_moveit_config arm_hardware_bridge

cd ~/robot_ws
colcon build --packages-select arm_hardware_bridge
source install/setup.bash
ros2 launch arm_hardware_bridge real_hardware.launch.py

# Give premission
sudo chmod 666 /dev/ttyACM0