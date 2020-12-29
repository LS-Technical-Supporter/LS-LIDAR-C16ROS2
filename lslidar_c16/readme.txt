注意: 
	本版本适用于小c16雷达2.6版本以上雷达。

运行: 
       1.环境 ubuntu20.04上的ros2-foxy版本。
       2.终端编译和运行。
         新建ros工作空间：c16_ws 文件夹(文件名可改，不可有中文字符)。
         cd c16_ws
            mkdir src
            cd src
            把驱动拷贝到src文件夹下，并解压。
        cd c16_ws
	colcon build
	source install/setup.bash
	ros2 launch lslidar_c16_decoder lslidar_c16_launch.py

版本信息：

----------------------2020-12-25------------------
初始版本: LSLIDAR_C16_V1.0.1_201225_ROS2
修订日期: 2020-12-25






