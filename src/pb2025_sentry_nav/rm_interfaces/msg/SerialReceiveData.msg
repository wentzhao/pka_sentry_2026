#std_msgs/Header header
#uint8 mode
#float32 bullet_speed
#float32 roll 
#float32 yaw
#float32 pitch
#bool is_attacked # 是否受到攻击 0->未受到攻击 1->受到攻击
#JudgeSystemData judge_system_data

uint16 game_progress # 当前比赛阶段
uint8 friendly_supply_zone_non_exchange              # 己方与兑换区不重叠的补给区/RMUL 补给区
uint16 current_hp # 血量