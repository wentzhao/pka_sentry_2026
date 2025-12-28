#仿真模式


colcon build
source install/setup.sh
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
style:=RMUC3 \
use_sim_time:=False


#真实模式


# colcon build
# source install/setup.sh
# ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
# style:=RMUL1 \
# use_sim_time:=False


#跑真实模式时要修改rm_behavior_tree.launch.py的命名空间