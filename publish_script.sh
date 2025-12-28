# !/bin/zsh
colcon build
source install/setup.sh

# ros2 topic pub -r 10 /decision_num rm_interfaces/msg/DecisionNum "{
#     decision_num: 1,
# }" &

# ros2 topic pub -r 10 /robot_status rm_interfaces/msg/NavigationReceive "{
#      current_hp: 600,
#      game_progress: 4, # 4:比赛开始
# }" &

ros2 topic pub -r 10 /red_standard_robot1/game_status rm_interfaces/msg/GameStatus "{
    game_progress: 4, # 4:比赛开始
}" &

ros2 topic pub -r 10 /red_standard_robot1/robot_status rm_interfaces/msg/RobotStatus "{
    current_hp: 600,
}" &

ros2 topic pub -r 10 /red_standard_robot1/rfid_status rm_interfaces/msg/RfidStatus "{
    friendly_supply_zone_non_exchange: 1,
}" &

# ros2 topic pub -r 3 /robot_hp rm_interfaces/msg/AllRobotHP "{
#     red_1_robot_hp: 100,
#     red_2_robot_hp: 100,
#     red_3_robot_hp: 100,
#     red_4_robot_hp: 100,
#     red_5_robot_hp: 200,
#     red_7_robot_hp: 200,
#     red_outpost_hp: 20,
#     red_base_hp: 1000,
#     blue_1_robot_hp: 100,
#     blue_2_robot_hp: 100,
#     blue_3_robot_hp: 100,
#     blue_4_robot_hp: 200,
#     blue_5_robot_hp: 200,
#     blue_7_robot_hp: 200,
#     blue_outpost_hp: 1000,
#     blue_base_hp: 1000
# }" &

# ros2 topic pub -r 5 /detector/armors rm_interfaces/msg/Armors "{
#   header: {
#     stamp: {sec: 0, nanosec: 0},
#     frame_id: 'my_frame'
#   },

#   armors: [ # 取消注释，代表识别到敌人
#     {
#       number: '1',
#       type: '0',
#       distance_to_image_center: 1.0,
#       pose: {
#         position: {x: 0.0, y: 0.0, z: 0.0},
#         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
#       }
#     }
#   ]
# }" &

wait
