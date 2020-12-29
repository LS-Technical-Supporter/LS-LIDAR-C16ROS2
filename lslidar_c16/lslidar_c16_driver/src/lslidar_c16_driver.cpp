/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */

#include "lslidar_c16_driver/lslidar_c16_driver.h"

namespace lslidar_c16_driver {
    static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;  //320000/16
    static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

    lslidarDriver::lslidarDriver() : lslidarDriver(rclcpp::NodeOptions()) {}

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions &options) :
            Node("lslidar_node", options)//,  diagnostics_(this)
    {
        //std::string tf_prefix = tf::getPrefixParam(private_nh);
        //ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
        //config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
        config_.frame_id = std::string("lslidar");
        config_.model = std::string("LSC16");
        config_.degree_mode = 2;
        config_.return_mode = 1;
        config_.rpm = 600.0;

        this->declare_parameter("frame_id");
        this->declare_parameter("model");
        this->declare_parameter("degree_mode");
        this->declare_parameter("return_mode");
        this->declare_parameter("rpm");
        this->get_parameter("frame_id", config_.frame_id);
        this->get_parameter("model", config_.model);
        this->get_parameter("degree_mode", config_.degree_mode);
        this->get_parameter("return_mode", config_.return_mode);
        this->get_parameter("rpm", config_.rpm);

        int packet_rate;  // packet frequency (Hz)
        double frequency = (config_.rpm / 60.0);  // expected Hz rate

        packet_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND / (BLOCKS_ONE_CHANNEL_PER_PKT * 2)) * config_.return_mode;
        config_.npackets = (int) ceil(packet_rate / frequency);
/*	this->declare_parameter("npackets");
	this->get_parameter("npackets", config_.npackets);*/
        RCLCPP_INFO(this->get_logger(), "[driver] return mode = %d ", config_.return_mode);
        RCLCPP_INFO(this->get_logger(), "[driver] publishing %d packets per scan", config_.npackets);
        printf("frequency = %f\n", frequency);


        int msop_udp_port = (int) MSOP_DATA_PORT_NUMBER;
        int difop_udp_port = (int) DIFOP_DATA_PORT_NUMBER;
        this->declare_parameter("msop_port");
        this->declare_parameter("difop_port");
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);

        scan_start = lslidar_c16_msgs::msg::LslidarC16ScanUnified();
        scan_start.packets.resize(1);


        // read data from live socket
        msop_input_.reset(new lslidar_c16_driver::InputSocket(this, msop_udp_port));
        difop_input_.reset(new lslidar_c16_driver::InputSocket(this, difop_udp_port));


        // raw packet output topic
        std::string output_packets_topic = std::string("lslidar_packet");
        std::string output_difop_topic = std::string("lslidar_packet_difop");
        time_synchronization_ = false;
        this->declare_parameter("output_packets_topic");
        this->declare_parameter("output_difop_topic");
        this->declare_parameter("time_synchronization");
        this->get_parameter("output_packets_topic", output_packets_topic);
        this->get_parameter("output_difop_topic", output_difop_topic);
        this->get_parameter("time_synchronization", time_synchronization_);

        msop_output_ = this->create_publisher<lslidar_c16_msgs::msg::LslidarC16ScanUnified>(output_packets_topic, 10);
        difop_output_ = this->create_publisher<lslidar_c16_msgs::msg::LslidarC16Packet>(output_difop_topic, 10);

        difop_thread_ = std::thread(std::bind(&lslidarDriver::difopPollThread, this));
        poll_thread_ = std::thread(std::bind(&lslidarDriver::pollThread, this));

        if (time_synchronization_) {
            output_sync_ = this->create_publisher<sensor_msgs::msg::TimeReference>("sync_header", 1);
        }
        scan_fill = false;
    }


    lslidarDriver::~lslidarDriver() {
        if (difop_thread_.joinable()) {
            difop_thread_.join();
        }
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
    }

    void lslidarDriver::pollThread(void) {
        while (rclcpp::ok()) {
            this->poll();
        }
    }

    void lslidarDriver::difopPollThread(void) {
        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            this->difopPoll();
        }
    }

/** poll the device
 *
 *  @returns true unless end of file reached
 */
    bool lslidarDriver::poll(void) {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_c16_msgs::msg::LslidarC16ScanUnified::UniquePtr scan(
                new lslidar_c16_msgs::msg::LslidarC16ScanUnified());

        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        int mode = config_.return_mode;
        int packet_rate;
        int azi1, azi2;
        if (difop_input_->getUpdateFlag()) {
            packet_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND / (BLOCKS_ONE_CHANNEL_PER_PKT * 2)) * mode;


            config_.rpm = difop_input_->getRpm();

            if (config_.rpm < 200) config_.rpm = 600;
            config_.npackets = ceil(packet_rate * 60 / config_.rpm);

            config_.npackets =
                    config_.npackets + config_.npackets / 10;//Increase the  size to find the next 0 degree packet
            difop_input_->clearUpdateFlag();
        }
        //scan->packets.clear();
        scan->packets.resize(config_.npackets);

        //Find the 0 degree packet, in order to fix the 0 degree angle position
        if (scan_fill) {
            scan->packets[0] = scan_start.packets[0];
            GPSCurrentTS = GPSCountingTS;
        } else {
            while (true) {
                while (true) {
                    int rc = msop_input_->getPacket(scan->packets[0], config_.time_offset, "msop_pkg");
                    if (rc == 0)
                        break;
                    if (rc < 0)
                        return false;
                }
                azi1 = 256 * scan->packets[0].data[3] + scan->packets[0].data[2];
                azi2 = 256 * scan->packets[0].data[1103] + scan->packets[0].data[1102];
                if (azi1 > 35000 && azi2 < 1000) break;
            }
        }
        scan_fill = false;

        // use in standard behaviour only
        for (int i = 1; i < config_.npackets; ++i) {
            while (true) {
                // keep reading until full packet received
                int rc = msop_input_->getPacket(scan->packets[i], config_.time_offset, "msop_pkg");
                if (rc == 0)
                    break;  // got a full packet?
                if (rc < 0)
                    return false;  // end of file reached?
            }


            azi1 = 256 * scan->packets[i].data[3] + scan->packets[i].data[2];
            azi2 = 256 * scan->packets[i].data[1103] + scan->packets[i].data[1102];

            //scan_fill:   The last packet in the previous frame acts as the first packet in the next frame
            if ((azi1 > 35000 && azi2 < 1000) || (azi1 < 500 && i > config_.npackets / 2)) {
                scan_fill = true;
                scan_start.packets[0] = scan->packets[i];
                break;
            }
        }

        if (time_synchronization_) {
            sensor_msgs::msg::TimeReference sync_header;

            // it is already the msop msg
            // use the first packets
            lslidar_c16_msgs::msg::LslidarC16Packet pkt = scan->packets[0];
            uint64_t packet_timestamp;
            packet_timestamp = (pkt.data[1200] +
                                pkt.data[1201] * pow(2, 8) +
                                pkt.data[1202] * pow(2, 16) +
                                pkt.data[1203] * pow(2, 24)) * 1e3; //ns

            timeStamp = rclcpp::Time(GPSCurrentTS, packet_timestamp);// s,ns
            sync_header.header.stamp = timeStamp;
            output_sync_->publish(sync_header);
        }


        // publish message using time of last packet read
        //  ROS_INFO("Publishing a full scan.");
        if (time_synchronization_) {
            scan->header.stamp = timeStamp;//get_clock()->now();
        } else {
            scan->header.stamp = get_clock()->now();
        }
        scan->header.frame_id = config_.frame_id;
        msop_output_->publish(std::move(scan));

        return true;
    }

    void lslidarDriver::difopPoll(void) {
        // reading and publishing scans as fast as possible.
        lslidar_c16_msgs::msg::LslidarC16Packet::UniquePtr difop_packet_ptr(
                new lslidar_c16_msgs::msg::LslidarC16Packet);
        lslidar_c16_msgs::msg::LslidarC16Packet difop_packet_msg;
        // keep reading
        //lslidar_c16_msgs::msg::LslidarC16Packet difop_packet_msg;
        int rc = difop_input_->getPacket(*difop_packet_ptr, config_.time_offset, "difop_pkg");
        if (rc == 0) {
            //RCLCPP_INFO(this->get_logger(), "timesdsadsadsadsadsa");
            //*difop_packet_ptr = difop_packet_msg;
            RCLCPP_DEBUG(this->get_logger(), "Publishing a difop data.");


            if (difop_packet_ptr->data[1202] == 0x03) {
                this->packetTimeStamp[4] = difop_packet_ptr->data[57];
                this->packetTimeStamp[5] = difop_packet_ptr->data[56];
                this->packetTimeStamp[6] = difop_packet_ptr->data[55];
                this->packetTimeStamp[7] = difop_packet_ptr->data[54];
                this->packetTimeStamp[8] = difop_packet_ptr->data[53];
                this->packetTimeStamp[9] = difop_packet_ptr->data[52];
            } else {
                this->packetTimeStamp[4] = difop_packet_ptr->data[41];
                this->packetTimeStamp[5] = difop_packet_ptr->data[40];
                this->packetTimeStamp[6] = difop_packet_ptr->data[39];
                this->packetTimeStamp[7] = difop_packet_ptr->data[38];
                this->packetTimeStamp[8] = difop_packet_ptr->data[37];
                this->packetTimeStamp[9] = difop_packet_ptr->data[36];
            }
            struct tm cur_time;
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_sec = this->packetTimeStamp[4] + 1;
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8] - 1;
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            this->pointcloudTimeStamp = mktime(&cur_time);
            difop_output_->publish(std::move(difop_packet_ptr));


            if (GPSCountingTS != this->pointcloudTimeStamp) {
                cnt_gps_ts = 0;
                GPSCountingTS = this->pointcloudTimeStamp;
            } else if (cnt_gps_ts == 3) {
                GPSStableTS = GPSCountingTS;
            } else {
                cnt_gps_ts++;
            }
        }
        if (rc < 0)
            return;  // end of file reached?
        //rclcpp::spin_some(this);

    }

// add for time synchronization
}  // namespace lslidar_c16_driver
