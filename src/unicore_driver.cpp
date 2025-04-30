/**
 * This file is part of UnicoreDriver.
 *
 * Copyright (C) 2025 Zhiliang Tan <zltan0906@gmail.com>
 *
 * UnicoreDriver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * UnicoreDriver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with UnicoreDriver.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include "serial_handler.h"
#include "unicore_message_processor.h"
#include <bitset>
#include "ntripclient_handler.h"

static std::atomic<bool> interrupted(false);

// variables for receiver config at start
std::mutex ack_m;
std::condition_variable ack_cv;
int ack_flag = 0;

void ctrl_c_handler(int s)
{
    interrupted = true;
}

// void print_data(const uint8_t* data, size_t len) 
// {
//     std::cout << "Received data: ";
//     for (size_t i = 0; i < len; ++i) {
//         std::cout << std::hex << static_cast<int>(data[i]) << " ";
//     }
//     std::cout << std::dec << std::endl;
// }
void print_data(const uint8_t* data, size_t len) 
{
    std::cout << "Received data: ";
    for (size_t i = 0; i < len; ++i) {
        // 使用bitset将字节转换为二进制，且输出8位
        std::cout << std::bitset<8>(data[i]) << " ";
    }
    std::cout << std::endl;
}



int main(int argc, char **argv)
{
    if (!interrupted.is_lock_free())  return 10;

    ros::init(argc, argv, "unicore_driver");
    ros::NodeHandle nh("~");

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);


    std::string port;
    int baud_rate;
    std::string gga_sentence;
    bool enable_rtk = false;
    std::string ntrip_host, ntrip_mountpoint, ntrip_user, ntrip_pass;
    int ntrip_port = 8002;

    nh.param<std::string>("port", port, "");
    nh.param("baud_rate", baud_rate, 921600);
    nh.param<std::string>("gga_sentence", gga_sentence, "");
    nh.param("enable_rtk", enable_rtk, false);
    nh.param<std::string>("ntrip_host", ntrip_host, "");
    nh.param("ntrip_port", ntrip_port, 8002);
    nh.param<std::string>("ntrip_mountpoint", ntrip_mountpoint, "");
    nh.param<std::string>("ntrip_user", ntrip_user, "");
    nh.param<std::string>("ntrip_pass", ntrip_pass, "");

    std::shared_ptr<SerialHandler> serial;
    std::shared_ptr<UnicoreMessageProcessor> unicore_msg_processor;

    if (!port.empty())
    {
        try 
        {
            unicore_msg_processor = std::make_shared<UnicoreMessageProcessor>(nh);
            serial = std::make_shared<SerialHandler>(port, static_cast<unsigned int>(baud_rate));

            if (!serial->isOpen()) 
            {
                ROS_ERROR("Serial port %s cannot be opened.", port.c_str());
                return 1;
            }

            serial->addCallback(std::bind(&UnicoreMessageProcessor::process_data, unicore_msg_processor.get(), std::placeholders::_1, std::placeholders::_2));
            serial->startRead();
            ROS_INFO("open port: %s", port.c_str());
        } catch (const std::exception& e) 
        {
            ROS_ERROR("Exception when opening serial port: %s", e.what());
            return 0;
        }
    }
    else
    {
        ROS_ERROR("No port specified. Please set the ~port1 parameter.");
        return 0;
    }


    // NTRIP init
    std::shared_ptr<NtripClientHandler> ntrip;
    if (enable_rtk && !gga_sentence.empty() && !ntrip_host.empty() && !ntrip_mountpoint.empty())
    {
        ntrip = std::make_shared<NtripClientHandler>(ntrip_host, static_cast<uint16_t>(ntrip_port), ntrip_mountpoint, ntrip_user, ntrip_pass);
        ntrip->setGGAProducer([gga_sentence]() { return gga_sentence; });
        ntrip->addCallback([serial](const uint8_t* data, size_t len) 
        {
            serial->writeRaw(data, len, 50);
        });
        ntrip->start();
        ROS_INFO("NTRIP client started for RTCM injection");
    }
    else if (enable_rtk)
    {
        ROS_WARN("NTRIP is enabled but required parameters are missing (host, mountpoint or gga)");
    }

    ros::Rate loop(50);  
    while (ros::ok() && !interrupted)
    {
        ros::spinOnce();
        loop.sleep();
    }

    if (serial)
        serial->close();
    if (ntrip)
        ntrip->stop();

    return 0;
}


