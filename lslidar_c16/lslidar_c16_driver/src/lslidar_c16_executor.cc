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

#include "rclcpp/rclcpp.hpp"
#include "lslidar_c16_driver/lslidar_c16_driver.h"
#include "lslidar_c16_decoder/convert.hpp"

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  
  rclcpp::NodeOptions options(rclcpp::NodeOptions().use_intra_process_comms(true));

  auto driver = std::make_shared<lslidar_c16_driver::lslidarDriver>(options);
  exec.add_node(driver);
																					  
  //auto converter = std::make_shared<lslidar_c16_decoder::Convert>(options);
  //exec.add_node(converter);				

  exec.spin();

  rclcpp::shutdown();

  return 0;
}