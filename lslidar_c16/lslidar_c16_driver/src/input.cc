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
 */


#include "lslidar_c16_driver/input.h"

namespace lslidar_c16_driver
{
static const size_t packet_size = sizeof(lslidar_c16_msgs::msg::LslidarC16Packet().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
int Input_first_time = 0;
static void my_handler(int sig)
{
	//flag = 0;
	abort();
}

Input::Input(rclcpp::Node* private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
  npkt_update_flag_ = false;
  cur_rpm_ = 0;
  return_mode_ = 1;
  flag = 1;
  signal(SIGINT, my_handler);
  
  if(Input_first_time == 0)
  {
	  private_nh->declare_parameter("device_ip");
	  private_nh->declare_parameter("add_multicast");
	  private_nh->declare_parameter("group_ip");
	  Input_first_time++;
  }
  private_nh->get_parameter("device_ip", devip_str_);
  private_nh->get_parameter("add_multicast", add_multicast);
  private_nh->get_parameter("group_ip", group_ip);
  if (!devip_str_.empty())
	  RCLCPP_INFO(private_nh->get_logger(), "[driver][input] accepting packets from IP address: %s", devip_str_.c_str());
}

int Input::getRpm(void)
{
  return cur_rpm_;
}

int Input::getReturnMode(void)
{
  return return_mode_;
}

bool Input::getUpdateFlag(void)
{
  return npkt_update_flag_;
}

void Input::clearUpdateFlag(void)
{
  npkt_update_flag_ = false;
}
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(rclcpp::Node* private_nh, uint16_t port) : Input(private_nh, port)
{
  //timeout_pub = this->create_publisher<std_msgs::msg::String>("timeout_topic", 10);
  msop_timeout = false;
  difop_timeout = false;																				 
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  RCLCPP_INFO(private_nh_->get_logger(), "[driver][socket] Opening UDP socket: port %d", port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] create socket fail");
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] setsockopt fail");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] socket bind fail");
    return;
  }
	
	if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
			group.imr_interface.s_addr =  htonl(INADDR_ANY);
			//group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                RCLCPP_ERROR(private_nh_->get_logger(), "Adding multicast group error");
                close(sockfd_);
                exit(1);
            } else
				RCLCPP_INFO(private_nh_->get_logger(), "Adding multicast group...OK.\n");
        }
		
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] fcntl fail");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one lslidar packet. */
int InputSocket::getPacket(lslidar_c16_msgs::msg::LslidarC16Packet& pkt, const double time_offset, std::string pkg_name)
{
  double time1 = private_nh_->get_clock()->now().seconds();
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 3000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
 
  while (flag == 1)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
	  int retval;
	  retval = poll(fds, 1, POLL_TIMEOUT);
		
	  if(retval >0){
		time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
				
		if(msop_timeout){
			std_msgs::msg::String renew_msg;
			renew_msg.data = "0003";
			//timeout_pub->publish(renew_msg);
			RCLCPP_WARN(private_nh_->get_logger(), "renew_code: [%s] ,lslidar [%s] renew: %s", renew_msg.data.c_str(), pkg_name.c_str(), bufTime);
			msop_timeout = false;
		}
		if(difop_timeout){
			std_msgs::msg::String renew_msg;
			renew_msg.data = "0004";
			//timeout_pub->publish(renew_msg);
			RCLCPP_WARN(private_nh_->get_logger(), "renew_code: [%s] ,lslidar [%s] renew: %s",renew_msg.data.c_str(), pkg_name.c_str(), bufTime);
			difop_timeout = false;
		}
	  }

      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
			RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
		time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
		
		if(pkg_name == "msop_pkg"){
			std_msgs::msg::String err_msg;
			err_msg.data = "0001";
			//timeout_pub->publish(err_msg);
			RCLCPP_WARN(private_nh_->get_logger(), "error_code: [%s] ,lslidar [%s] poll() timeout: %s %s",err_msg.data.c_str(), pkg_name.c_str(), bufTime, devip_str_.c_str());
			msop_timeout = true;
		}
		if(pkg_name == "difop_pkg"){
			std_msgs::msg::String err_msg;
			err_msg.data = "0002";
			//timeout_pub->publish(err_msg);
			RCLCPP_WARN(private_nh_->get_logger(), "error_code: [%s] ,lslidar [%s] poll() timeout: %s %s",err_msg.data.c_str(), pkg_name.c_str(), bufTime, devip_str_.c_str());
			difop_timeout = true;
		}  
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] poll() reports Rslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt.data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
          RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break;  // done
    }
	RCLCPP_WARN(private_nh_->get_logger(), "[driver][socket] incomplete rslidar packet read: %d bytes", nbytes);
  }
  if (flag == 0)
  {
    abort();
  }

  if (pkt.data[0] == 0xA5 && pkt.data[1] == 0xFF && pkt.data[2] == 0x00 && pkt.data[3] == 0x5A)
  {
	 //difop
    int rpm = (pkt.data[8]<<8)|pkt.data[9];
      //RCLCPP_INFO(private_nh_->get_logger(), "[driver]rpm--= %d ",rpm);

    int mode= 1;
    if (cur_rpm_ != rpm || return_mode_ != mode)
    {
      cur_rpm_ = rpm;
      return_mode_ = mode;
      npkt_update_flag_ = true;
    }
    
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = private_nh_->get_clock()->now().seconds();
  pkt.stamp = rclcpp::Time(((time2 + time1) / 2.0 + time_offset)*1e9);
  return 0;
}

}
