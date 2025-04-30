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

#include "ntripclient_handler.h"
#include <ros/ros.h>

NtripClientHandler::NtripClientHandler(const std::string& host, uint16_t port,
                                       const std::string& mountpoint,
                                       const std::string& username,
                                       const std::string& password)
    : host_(host), port_(port), mountpoint_(mountpoint),
      username_(username), password_(password),
      socket_(io_service_), gga_timer_(io_service_), running_(false)
{
    ROS_INFO("[NTRIP] Initialized client for host %s:%u, mountpoint: %s", host.c_str(), port, mountpoint.c_str());
}

NtripClientHandler::~NtripClientHandler()
{
    stop();
}

void NtripClientHandler::addCallback(std::function<void(const uint8_t*, size_t)> callback)
{
    callbacks_.push_back(callback);
    ROS_INFO("[NTRIP] Callback registered.");
}

void NtripClientHandler::setGGAProducer(std::function<std::string()> gga_provider)
{
    gga_provider_ = gga_provider;
    ROS_INFO("[NTRIP] GGA provider set.");
}

void NtripClientHandler::start()
{
    using boost::asio::ip::tcp;
    ROS_INFO("[NTRIP] Resolving host...");
    try {
        tcp::resolver resolver(io_service_);
        tcp::resolver::query query(host_, std::to_string(port_));
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket_, endpoint_iterator);
        ROS_INFO("[NTRIP] TCP connection established to %s:%u", host_.c_str(), port_);
    } catch (const std::exception& e) {
        ROS_ERROR("[NTRIP] Failed to connect to NTRIP caster: %s", e.what());
        return;
    }

    std::string userpass = username_ + ":" + password_;
    ROS_INFO("[NTRIP] username_: %s", username_.c_str());
    ROS_INFO("[NTRIP] password_: %s", password_.c_str());
    ROS_INFO("[NTRIP] userpass: %s", userpass.c_str());
    std::string auth_base64 = encodeBase64(userpass);
    ROS_INFO("[NTRIP] Encoded auth: %s", auth_base64.c_str());


    std::ostringstream request;
    request << "GET /" << mountpoint_ << " HTTP/1.1\r\n";
    request << "User-Agent: NTRIP NTRIPClient/20181206\r\n";
    request << "Accept: */*\r\n";
    request << "Connection: close\r\n";
    request << "Authorization: Basic " << auth_base64 << "\r\n";
    request << "\r\n";

    ROS_INFO_STREAM("[NTRIP] Sending request:\n" << request.str());
    boost::asio::write(socket_, boost::asio::buffer(request.str()));
    ROS_INFO("[NTRIP] Sent NTRIP request to caster.");

    boost::asio::streambuf response;
    boost::system::error_code ec;
    std::size_t n = boost::asio::read_until(socket_, response, "\r\n\r\n", ec);
    if (ec) 
    {
        ROS_ERROR("[NTRIP] Failed to read HTTP response: %s", ec.message().c_str());
        return;
    }
    ROS_INFO("[NTRIP] Received HTTP header (%lu bytes)", n);

    std::istream response_stream(&response);
    std::string line;
    bool unauthorized = false;
    while (std::getline(response_stream, line) && line != "\r") 
    {
        ROS_INFO_STREAM("[NTRIP HEADER] " << line);
        if (line.find("401 Unauthorized") != std::string::npos)
        {
            unauthorized = true;
        }
    }

    if (unauthorized) 
    {
        ROS_ERROR("[NTRIP] Unauthorized access: check username, password, or mountpoint");
        return;
    }

    running_ = true;
    startRead();

    if (gga_provider_) sendGGA();

    boost::thread bt([this]() 
    {
        ROS_INFO("[NTRIP] IO service thread started.");
        io_service_.run();
    });
}

void NtripClientHandler::stop()
{
    running_ = false;
    socket_.close();
    gga_timer_.cancel();
    if (!io_service_.stopped())
        io_service_.stop();
    ROS_INFO("[NTRIP] NTRIP client stopped.");
}

void NtripClientHandler::startRead()
{
    ROS_INFO("[NTRIP] Starting async RTCM read...");
    socket_.async_read_some(boost::asio::buffer(buffer_),
        [this](const boost::system::error_code& ec, std::size_t len)
        {
            if (!ec && running_)
            {
                ROS_DEBUG("[NTRIP] Received RTCM data: %lu bytes", len);
                for (auto& cb : callbacks_)
                    cb(buffer_.data(), len);
                startRead();
            }
            else if (ec)
            {
                ROS_WARN("[NTRIP] RTCM read error: %s", ec.message().c_str());
            }
        });
}

void NtripClientHandler::sendGGA()
{
    if (!running_ || !gga_provider_) return;
    std::string gga = gga_provider_();
    if (!gga.empty())
    {
        ROS_DEBUG("[NTRIP] Sending GGA: %s", gga.c_str());
        boost::asio::write(socket_, boost::asio::buffer(gga));
    }
    gga_timer_.expires_from_now(boost::posix_time::seconds(1));
    gga_timer_.async_wait([this](const boost::system::error_code& error) 
    {
        if (!error) sendGGA();
    });
}

std::string NtripClientHandler::encodeBase64(const std::string& src)
{
    using namespace boost::archive::iterators;
    using It = base64_from_binary<transform_width<std::string::const_iterator, 6, 8>>;
    std::stringstream os;
    std::copy(It(src.begin()), It(src.end()), std::ostream_iterator<char>(os));
    size_t mod = src.size() % 3;
    if (mod > 0) os << std::string(3 - mod, '=');
    return os.str();
}
