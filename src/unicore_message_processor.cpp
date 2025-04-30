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

#include "unicore_message_processor.h"

constexpr uint32_t UnicoreMessageProcessor::CRC32_TABLE[256];

UnicoreMessageProcessor::UnicoreMessageProcessor(const ros::NodeHandle& nh):nh_(nh)
{
    pub_bestnav_ecef_ = nh_.advertise<nav_msgs::Odometry>("best_nav_ecef", 100);
}
void UnicoreMessageProcessor::process_data(const uint8_t* data, size_t len) 
{
    if (len < 24) 
    {
        std::cerr << "Invalid data size!" << std::endl;
        return ;
    }

    if (data[SYNC_1_OFFSET] != UNICORE_SYNC_1 || 
        data[SYNC_2_OFFSET] != UNICORE_SYNC_2 || 
        data[SYNC_3_OFFSET] != UNICORE_SYNC_3) 
    {
        std::cerr << "Sync mismatch!" << std::endl;
        return ;
    }   

    // uint8_t timeRef = data[TIME_REF_OFFSET];    // 时间参考
    // uint8_t timeStatus = data[TIME_STATUS_OFFSET]; // 时间状态
    // uint16_t week = data[WEEK_OFFSET]  | (data[WEEK_OFFSET + 1]<< 8); // 时间周
    // uint32_t ms = data[MS_OFFSET]  | (data[MS_OFFSET + 1] << 8) | 
    //                 (data[MS_OFFSET + 2] << 16) | (data[MS_OFFSET + 3]<< 24); // 毫秒
    
    // uint32_t reserved = data[RESERVED_OFFSET]  | (data[RESERVED_OFFSET + 1] << 8) | 
    //                     (data[RESERVED_OFFSET + 2] << 16) | (data[RESERVED_OFFSET + 3]<< 24); // 保留字段
    // uint8_t version = data[VERSION_OFFSET];   // 版本号
    // uint8_t leapSec = data[LEAP_SEC_OFFSET];  // 闰秒
    // uint16_t delayMs = data[DELAY_MS_OFFSET]  | (data[DELAY_MS_OFFSET + 1]<< 8); // 数据输出延迟
    

    uint16_t messageId = *reinterpret_cast<const uint16_t*>(data + MSG_ID_OFFSET);
    uint16_t messageLength = *reinterpret_cast<const uint16_t*>(data + MSG_LENGTH_OFFSET);

    if (len != static_cast<size_t>(messageLength + 24 + 4))
    {
        std::cerr << "Invalid message length!" << std::endl;
        return ;
    }
    if (!check_crc32(data, len)) 
    {
        std::cerr << "Invalid CRC checksum!" << std::endl;
        return ;
    }

    // std::cout << "Message ID: " << messageId << std::endl;
    // std::cout << "Message Length: " << messageLength << std::endl;

    switch(messageId)
    {
        case UNICORE_OBSVM_ID:
        {
            this->parse_meas_msg(data, len);
            break;
        }
        case UNICORE_BDSION_ID:
        {
            this->parse_iono_msg(data, len);
            break;
        }
        case UNICORE_BDSEPH_ID:
        {
            this->parse_ephem_msg(data, len);
            break;
        }
        case UNICORE_BESTNAVXYZ_ID:
        {
            nav_msgs::Odometry best_nav_ecef_msg = this->parse_bestnav_ecef_msg(data, len);            
            
            if (!(best_nav_ecef_msg.pose.pose.position.x == 0.0 &&
                best_nav_ecef_msg.pose.pose.position.y == 0.0 &&
                best_nav_ecef_msg.pose.pose.position.z == 0.0))
            {
                pub_bestnav_ecef_.publish(best_nav_ecef_msg);  
            }
            else
            {
                ROS_WARN("Invalid best_nav_ecef_msg, not publishing.");
            }
            break;
        }

        default:
        {
            std::cout<<"Message ID not match..."<<std::endl;
            break;
        }

    }    

    return ;
}

nav_msgs::Odometry UnicoreMessageProcessor::parse_bestnav_ecef_msg(const uint8_t *msg_data, const uint32_t msg_len)
{
    nav_msgs::Odometry bestnav_ecef;
    if (msg_len < 112)
    {
        std::cerr << "unicore bestnavxyz length error: len=" << msg_len;
        return bestnav_ecef;
    }

    const uint8_t *p = msg_data + HEADER_LENGTH; 

    // GPS Time
    uint16_t gps_week = *reinterpret_cast<const uint16_t*>(msg_data + WEEK_OFFSET);
    uint32_t tow_ms = *reinterpret_cast<const uint32_t*>(msg_data + MS_OFFSET);
    uint16_t delay_ms = *reinterpret_cast<const uint16_t*>(msg_data + DELAY_MS_OFFSET);

    int32_t p_type = *reinterpret_cast<const int32_t*>(p + ECEF_P_TYPE_OFFSET);
    // std::cout << "P type: " << p_type << " (" << get_type_name(p_type) << ")" << std::endl;
    // int32_t v_type = *reinterpret_cast<const int32_t*>(p + ECEF_V_TYPE_OFFSET);
    // std::cout << "V type: " << p_type << " (" << get_type_name(p_type) << ")" << std::endl;

    bestnav_ecef.child_frame_id = get_type_name(p_type);

    double tow_sec = (tow_ms - delay_ms) * 0.001;
    double unix_time = gpsTimeToUnixTime(gps_week, tow_sec);

    bestnav_ecef.header.stamp = ros::Time(unix_time);
 
    bestnav_ecef.header.frame_id = "ecef";
    bestnav_ecef.pose.pose.position.x = *reinterpret_cast<const double*>(p + ECEF_PX_OFFSET); 
    bestnav_ecef.pose.pose.position.y = *reinterpret_cast<const double*>(p + ECEF_PY_OFFSET); 
    bestnav_ecef.pose.pose.position.z = *reinterpret_cast<const double*>(p + ECEF_PZ_OFFSET); 
    // pos cov
    float sigma_px = *reinterpret_cast<const float*>(p + ECEF_PX_SIGMA_OFFSET);
    float sigma_py = *reinterpret_cast<const float*>(p + ECEF_PY_SIGMA_OFFSET);
    float sigma_pz = *reinterpret_cast<const float*>(p + ECEF_PZ_SIGMA_OFFSET);
    bestnav_ecef.pose.covariance.fill(0.0);
    bestnav_ecef.pose.covariance[0] = sigma_px * sigma_px; 
    bestnav_ecef.pose.covariance[7] = sigma_py * sigma_py; 
    bestnav_ecef.pose.covariance[14] = sigma_pz * sigma_pz; 

    bestnav_ecef.twist.twist.linear.x = *reinterpret_cast<const double*>(p + ECEF_VX_OFFSET);
    bestnav_ecef.twist.twist.linear.y = *reinterpret_cast<const double*>(p + ECEF_VY_OFFSET);
    bestnav_ecef.twist.twist.linear.z = *reinterpret_cast<const double*>(p + ECEF_VZ_OFFSET);

    // vel cov
    float sigma_vx = *reinterpret_cast<const float*>(p + ECEF_VX_SIGMA_OFFSET);
    float sigma_vy = *reinterpret_cast<const float*>(p + ECEF_VY_SIGMA_OFFSET);
    float sigma_vz = *reinterpret_cast<const float*>(p + ECEF_VZ_SIGMA_OFFSET);

    bestnav_ecef.twist.covariance.fill(0.0);
    bestnav_ecef.twist.covariance[0] = sigma_vx * sigma_vx; 
    bestnav_ecef.twist.covariance[7] = sigma_vy * sigma_vy; 
    bestnav_ecef.twist.covariance[14] = sigma_vz * sigma_vz; 

    return bestnav_ecef;

}



// todo...  OBSVM  EPHEM  IONO   use gnss_comm
void append_raw_with_timestamp(const std::string& filename, const uint8_t* data, size_t len)
{
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S")
              << "." << std::setw(3) << std::setfill('0') << now_ms.count();

    std::ofstream ofs(filename, std::ios::app | std::ios::binary);
    ofs << timestamp.str() << " ";
    ofs.write(reinterpret_cast<const char*>(data), len);
    ofs << "\n";
    ofs.close();
}

void UnicoreMessageProcessor::parse_meas_msg(const uint8_t *msg_data, const uint32_t msg_len)
{
    append_raw_with_timestamp("obs_data.bin", msg_data, msg_len);
}

void UnicoreMessageProcessor::parse_iono_msg(const uint8_t *msg_data, const uint32_t msg_len)
{
    append_raw_with_timestamp("iono_data.bin", msg_data, msg_len);
}

void UnicoreMessageProcessor::parse_ephem_msg(const uint8_t *msg_data, const uint32_t msg_len)
{
    append_raw_with_timestamp("ephem_data.bin", msg_data, msg_len);
}



uint32_t UnicoreMessageProcessor::crc32(const uint8_t* data, size_t len) 
{
    int iIndex;
    uint32_t crc = 0; 
    for (iIndex=0; iIndex < len; iIndex++)
    {
        crc = CRC32_TABLE[(crc ^ data[iIndex]) & 0xff] ^ (crc >> 8);
    }
    return crc;
}


bool UnicoreMessageProcessor::check_crc32(const uint8_t *data, size_t len)
{
    uint32_t computed_crc = crc32(data, len-4 ); 
    uint32_t received_crc = data[len - 4]  | 
                            (data[len - 3] << 8) | 
                            (data[len - 2] << 16) | 
                            (data[len - 1] << 24); 
    
    return computed_crc == received_crc;
}

