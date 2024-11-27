colcon build
source install/setup.sh
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \
mode:=nav \
lio:=pointlio \
localization:=slam_toolbox \
lio_rviz:=False \
nav_rviz:=True 

# cmds=(
# "ros2 launch rm_nav_bringup bringup_real.launch.py \
#     world:=labtory4 \
#     mode:=nav \
#     lio:=pointlio \
#     localization:=slam_toolbox \
#     lio_rviz:=False \
#     nav_rviz:=True"
# "ros2 launch rm_nav_bringup bringup.launch.py"
# )

# for cmd in "${cmds[@]}";
# do
# 	echo Current CMD : "$cmd"
# 	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
# 	sleep 0.2
# done
