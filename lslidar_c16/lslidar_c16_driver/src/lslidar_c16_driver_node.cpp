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


using namespace lslidar_c16_driver;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lslidar_c16_driver::lslidarDriver>();
  

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
