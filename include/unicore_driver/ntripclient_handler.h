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


#ifndef NTRIP_CLIENT_HANDLER_H
#define NTRIP_CLIENT_HANDLER_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <functional>
#include <vector>
#include <string>
#include <iostream>
#include <atomic>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <sstream>

class NtripClientHandler
{
public:
    NtripClientHandler(const std::string& host,
                       uint16_t port,
                       const std::string& mountpoint,
                       const std::string& username,
                       const std::string& password);

    ~NtripClientHandler();

    void addCallback(std::function<void(const uint8_t*, size_t)> callback);

    void start();
    void stop();
    void setGGAProducer(std::function<std::string()> gga_provider);


private:
    void startRead();
    void sendGGA();
    std::string encodeBase64(const std::string& src);


    std::string host_, mountpoint_, username_, password_;
    uint16_t port_;

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;
    boost::asio::deadline_timer gga_timer_;

    std::vector<std::function<void(const uint8_t*, size_t)>> callbacks_;
    std::function<std::string()> gga_provider_;

    enum { BUFFER_SIZE = 4096 };
    std::array<uint8_t, BUFFER_SIZE> buffer_;
    std::atomic<bool> running_;
};

#endif // NTRIP_CLIENT_HANDLER_H


