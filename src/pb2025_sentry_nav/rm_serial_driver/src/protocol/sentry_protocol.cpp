// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_serial_driver/protocol/sentry_protocol.hpp"
#include<iostream>

// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rm_interfaces/msg/navigation_send.hpp>
using namespace std;
namespace pka::serial_driver::protocol {
ProtocolSentry::ProtocolSentry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<16>>(uart_transporter);
  packet_tool_test = std::make_shared<FixedPacketTool<8>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) 
{
//   packet_.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  packet_.loadData<unsigned char>(FireState::Fire, 1);

  // is_spin
  // packet_.loadData<unsigned char>(0x00, 2);
  // gimbal control
  // packet_.loadData<float>(static_cast<float>(data.pitch), 4);
  // packet_.loadData<float>(static_cast<float>(data.yaw), 8);
  // packet_.loadData<float>(static_cast<float>(data.distance), 12);
    packet_.loadData<float>(1.0, 4);
  packet_.loadData<float>(2.0, 8);
  packet_.loadData<float>(3.0, 12);
  // // chassis control
  // // linear x
  // packet_.loadData<float>(0, 16);
  // // linear y
  // packet_.loadData<float>(0, 20);
  // // angular z
  // packet_.loadData<float>(0, 24);
  // // useless data
  // packet_.loadData<float>(0, 28);
  packet_tool_->sendPacket(packet_);
}

// void ProtocolSentry::send(const rm_interfaces::msg::ChassisCmd &data) {
void ProtocolSentry::send(const geometry_msgs::msg::Twist &data) {

  FixedPacket<16> packet;
  // packet_.loadData<unsigned char>(0x00, 1);
  // is_spin
  // packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
  // packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
  //
  packet.loadData<unsigned char>(0x01, 1);
  // packet_.loadData<unsigned char>(0x01, 3);

  // gimbal control
  // packet_.loadData<float>(0, 4);
  // packet_.loadData<float>(0, 8);
  // packet_.loadData<float>(0, 12);
  // chassis control
  // linear x
  // packet_.loadData<float>(data.twist.linear.x, 16);
  // // linear y
  // packet_.loadData<float>(data.twist.linear.y, 20);
  // // angular z
  // packet_.loadData<float>(data.twist.angular.z, 24);
  float x = (data.linear.x)*0.2;
  float y = (data.linear.y)*0.2;

  packet.loadData<float>(x, 2);
  // linear y
  packet.loadData<float>(y, 6);
  // angular z
  packet.loadData<float>(data.angular.z, 10);
  // std::cout<<123<<std::endl;

  // packet.loadData<unsigned char>(0x01, 14);
  // // useless data
  // packets_.loadData<short>(1, 28);
  // packets_.loadData<unsigned char>(0x00, 30);
  std::cout<<x<<std::endl;
 
  packet_tool_->sendPacket(packet);
  // transporter_->write(packets_.buffer(), 16);
}


// void ProtocolSentry::send(const rm_interfaces::msg::NavigationSend &data) 
// {
//   packet_.loadData<unsigned char>(data.friction_wheel_scan, 14);
//   // packet_.loadData<unsigned char>(data.stop_gimbal_scan, 14);
//     std::cout<<data.friction_wheel_scan<<std::endl;
//   packet_tool_->sendPacket(packet_);
// }


bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) 
{
  FixedPacket<8> packet;
  // std::cout<<456<<std::endl;
  if (packet_tool_test->recvPacket(packet)) 
  {
    packet.unloadData(data.game_progress, 1);
    data.game_progress = (data.game_progress << 8) | (data.game_progress >> 8);
    // for(int i=0;i<2;i++){
    //   std::cout<<std::hex <<static_cast<int>(reinterpret_cast<unsigned char*>(&data.game_progress)[i])<< " ";
    // }
    // std::cout << "\n";
    packet.unloadData(data.friendly_supply_zone_non_exchange, 3);
    // packet.unloadData(data.friendly_supply_zone_non_exchange, 3);

    packet.unloadData(data.current_hp, 4);
     data.current_hp = (data.current_hp << 8) | (data.current_hp >> 8);
    //  packet.unloadData(data.current_hp, 4);
    // for(int i=0;i<2;i++){
    //   std::cout<<std::hex <<static_cast<int>(reinterpret_cast<unsigned char*>(&data.current_hp)[i])<< " ";
    // }
    // std::cout << "\n";

    // std::cout<<"data.game_progress:"<<data.game_progress<<std::endl;
    // std::cout<<"data.friendly_supply_zone_non_exchange:"<< data.friendly_supply_zone_non_exchange<<std::endl;
    // std::cout<<"data.current_hp:"<< data.current_hp<<std::endl;


    // std::cout<<"data.mode:"<< int(data.mode)<<" ";
    // std::cout<<"data.roll:"<< data.roll<<" ";
    // std::cout<<"data.pitch:"<< data.pitch<<" ";
    // std::cout<<"data.yaw:"<< data.yaw<<std::endl;
    // packet.unloadData(data.mode, 1);
    // packet.unloadData(data.roll, 2);
    // packet.unloadData(data.pitch, 6);
    // packet.unloadData(data.yaw, 10);
      // packet.unloadData(data.is_attacked,1);
  //    // game status
  //   uint8_t enemy_color;
  //   packet.unloadData(enemy_color, 1);
  //   data.mode = (enemy_color == ENEMY_BLUE ? 1 : 0);

    // packet.unloadData(data.pitch, 2);
    // packet.unloadData(data.yaw, 6);
    // packet.unloadData(data.roll,10);
  //   // 实际上是底盘角度
  //   // packet.unloadData(data.chassis_yaw, 10);
  //   // blood
  //   packet.unloadData(data.judge_system_data.blood, 14);
  //   // remaining time
  //   packet.unloadData(data.judge_system_data.remaining_time, 16);
  //   // outpost hp
  //   packet.unloadData(data.judge_system_data.outpost_hp, 20);
  //   // operator control message
  //   packet.unloadData(data.judge_system_data.operator_command.is_outpost_attacking, 22);
  //   packet.unloadData(data.judge_system_data.operator_command.is_retreating, 23);
  //   packet.unloadData(data.judge_system_data.operator_command.is_drone_avoiding, 24);

  //   packet.unloadData(data.judge_system_data.game_status, 25);

  //   packet.unloadData(data.bullet_speed,26);

  // cout<<"is_attacked:"<<data.is_attacked<<endl;
  // // cout<<"pitch:"<<data.pitch<<endl;
  // // cout<<"yaw:"<<data.yaw<<endl;
  // // cout<<"roll:"<<data.roll<<endl;
  // // cout<<"blood:"<<data.judge_system_data.blood<<endl;
  // // cout<<"remaining_time:"<<data.judge_system_data.remaining_time<<endl;
  // // cout<<"outpost_hp:"<<data.judge_system_data.outpost_hp<<endl;
  // // cout<<"is_outpost_attacking:"<<data.judge_system_data.operator_command.is_outpost_attacking<<endl;
  // // cout<<"is_retreating:"<<data.judge_system_data.operator_command.is_retreating<<endl;
  // // cout<<"is_drone_avoiding:"<<data.judge_system_data.operator_command.is_drone_avoiding<<endl;
  // // cout<<"game_status:"<<data.judge_system_data.game_status<<endl;
  // // cout<<"bullet_speed:"<<data.bullet_speed<<endl;

  //   // data.bullet_speed = 25;
  //   return true;
  } else {
    return false;
  }
  return true;
}




std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  // auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
  //   "armor_solver/cmd_gimbal",
  //   rclcpp::SensorDataQoS(),
  //   [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  // "armor_solver/cmd_gimbal",
  // auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
  //   "rune_solver/cmd_gimbal",
  //   rclcpp::SensorDataQoS(),
  //   [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });

  //
  // auto sub3 = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
  // "cmd_vel_chassis",
  // rclcpp::SensorDataQoS(),
  // [this](const rm_interfaces::msg::ChassisCmd::SharedPtr msg) { this->send(*msg); });
  // geometry_msgs/Twist
    auto sub3 = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->send(*msg); });

    //  auto sub4 = node->create_subscription<rm_interfaces::msg::NavigationSend>(
    // "navigation_send",  // 订阅 navigation_send 话题
    // rclcpp::SensorDataQoS(),
    // [this](const rm_interfaces::msg::NavigationSend::SharedPtr msg) { this->send(*msg); });
    
  
  // auto sub3 = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
  //   "/cmd_chassis",
  //   rclcpp::SensorDataQoS(),
  //   [this](const rm_interfaces::msg::ChassisCmd: :SharedPtr msg) { this->send(*msg); });
  return {sub3};
  // return {sub1, sub2, sub3};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolSentry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2};
}
}  // namespace pka::serial_driver::protocol