/**
* This file is part of ublox-driver.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* ublox-driver is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ublox-driver is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ublox-driver. If not, see <http://www.gnu.org/licenses/>.

* Modified for UnicoreDriver, © 2025 Zhiliang Tan <zltan0906@gmail.com>
*/

#include "serial_handler.h"

SerialHandler::SerialHandler(std::string port, unsigned int baud_rate, unsigned int buf_size) : port_(port), 
                    data_buf_(new uint8_t[buf_size]), serial_(io_service_, port), keep_working_task_(io_service_), 
                    write_timeout_(timer_io_service_), MSG_HEADER_LEN(24)
{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    if (!serial_.is_open())
    {
        std::cerr << "Cannot open serial port " << port << ".\n";
        return;
    }
    boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service_));
}

SerialHandler::~SerialHandler()
{
    close();
    serial_.close();
}

void SerialHandler::addCallback(std::function<void (const uint8_t*, size_t)> callback)
{
    callbacks_.push_back(callback);
}

void SerialHandler::startRead()
{
    boost::asio::async_read(serial_, boost::asio::buffer(data_buf_.get(), MSG_HEADER_LEN), 
            boost::asio::transfer_all(),
            boost::bind(&SerialHandler::read_handler, this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));
}

void SerialHandler::stop_read()
{
    serial_.cancel();
}

void SerialHandler::write(const std::string &str, uint32_t timeout_ms)
{
    writeRaw((const uint8_t*)(str.c_str()), str.size(), timeout_ms);
}

void SerialHandler::writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    boost::asio::async_write(serial_, boost::asio::buffer(data, len), 
        boost::bind(&SerialHandler::write_callback, this, 
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
/*     // unblock these lines will cause write timeout for unknown reason  
    write_timeout_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    write_timeout_.async_wait(boost::bind(&SerialHandler::wait_callback, this, 
        boost::asio::placeholders::error));
    timer_io_service_.run();
    timer_io_service_.reset();
 */
}

void SerialHandler::close()
{
    serial_.cancel();
    write_timeout_.cancel();
    if (!io_service_.stopped())
        io_service_.stop();
    if (!timer_io_service_.stopped())
        timer_io_service_.stop();
}

void SerialHandler::read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error)
    {
        if (bytes_transferred < MSG_HEADER_LEN)
        {
            std::cerr << "Not received enough bytes: " << bytes_transferred << " while expecting " << MSG_HEADER_LEN << '\n';
            return;
        }

        uint32_t header_remains = MSG_HEADER_LEN;
        while (header_remains != 0)
        {
            uint32_t pream_idx = 0;
            // sync 0xAA 0x44 0xB5
            for (; pream_idx < MSG_HEADER_LEN - 2; ++pream_idx)
            {
                if (data_buf_[pream_idx] == 0xAA &&
                    data_buf_[pream_idx + 1] == 0x44 &&
                    data_buf_[pream_idx + 2] == 0xB5)
                    break;
            }
            header_remains = pream_idx;
            if(header_remains != 0)
            {
                memmove(data_buf_.get(), data_buf_.get() + pream_idx, MSG_HEADER_LEN - header_remains);

                boost::system::error_code receive_error;
                unsigned int bytes_num = boost::asio::read(serial_, 
                    boost::asio::buffer(data_buf_.get() + MSG_HEADER_LEN - pream_idx, header_remains), 
                    boost::asio::transfer_all(), receive_error);
                    
                if (bytes_num != header_remains)
                {
                    std::cerr << "Not received enough bytes: " << bytes_num << " while expecting " << header_remains << '\n';
                    return;
                }
            }           
            
        }

        uint16_t *msg_len_ptr = reinterpret_cast<uint16_t*>(data_buf_.get() + 6); 
        uint16_t msg_length = *msg_len_ptr;

        boost::system::error_code receive_error;
        unsigned int bytes_num = boost::asio::read(serial_, 
            boost::asio::buffer(data_buf_.get() + MSG_HEADER_LEN, msg_length + 4), 
            boost::asio::transfer_all(), receive_error); 
        if (bytes_num != static_cast<unsigned int>(msg_length + 4))
        {
            std::cerr << "Not received enough bytes: " << bytes_num << " while expecting " << (msg_length + 4) << '\n';
            return;
        }

        for (auto &f : callbacks_)
            f(data_buf_.get(), MSG_HEADER_LEN + msg_length + 4); 
        

        startRead();
    }
    else if (error == boost::asio::error::operation_aborted || error != boost::asio::error::bad_descriptor)
    {
        std::cerr << "Serial " << port_ <<  " read handler aborted due to serial shutdown.\n";
        return;
    }
    else
    {
        std::cerr << "Connection to serial " << port_ << " lost.\n";
    }
}


void SerialHandler::write_callback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (error || !bytes_transferred)
    {
        // no data was written
        std::cerr << "Serial write failed or operation timeout: " << port_ << '\n';
        return;
    }
    write_timeout_.cancel();
}

void SerialHandler::wait_callback(const boost::system::error_code& error)
{
    if (error)  return;                 // data was written and this timer was cancelled

    serial_.cancel();       // will cause write_callback to fire with an error
}
