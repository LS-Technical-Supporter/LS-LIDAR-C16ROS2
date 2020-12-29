/*
 * This file is part of lslidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 */

#ifndef __LSLIDAR_INPUT_H_
#define __LSLIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
//#include <pcap.h>
#include <netinet/in.h>
#include "lslidar_c16_msgs/msg/lslidar_c16_packet.hpp"
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/time_reference.hpp"


namespace lslidar_c16_driver
{
static const uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
static const uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
/**
 *  从在线的网络数据（pcap文件）中提取出lidar的原始数据，即packet数据包
 * @brief The Input class,
     *
     * @param private_nh  一个NodeHandled,用于通过节点传递参数
     * @param port
     * @returns 0 if successful,
     *          -1 if end of file
     *          >0 if incomplete packet (is this possible?)
 */
class Input
{
public:
  Input(rclcpp::Node* private_nh, uint16_t port);

  virtual ~Input()
  {
  }

  virtual int getPacket(lslidar_c16_msgs::msg::LslidarC16Packet& pkt, const double time_offset, std::string pkg_name) = 0;

  int getRpm(void);
  int getReturnMode(void);
  bool getUpdateFlag(void);
  void clearUpdateFlag(void);
  volatile sig_atomic_t flag;
protected:
  rclcpp::Node* private_nh_;
  uint16_t port_;
  std::string devip_str_;
  int cur_rpm_;
  int return_mode_;
  bool npkt_update_flag_;
  bool add_multicast;
  std::string group_ip;

};

/** @brief Live lslidar input from socket. */
class InputSocket : public Input
{
public:
  InputSocket(rclcpp::Node* private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);

  virtual ~InputSocket();

  virtual int getPacket(lslidar_c16_msgs::msg::LslidarC16Packet& pkt, const double time_offset, std::string pkg_name);
  

private:
  int sockfd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timeout_pub;
  bool msop_timeout;
  bool difop_timeout;
  in_addr devip_;

};

}
#endif  // __LSLIDAR_INPUT_H
