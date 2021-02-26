/*
    Copyright (C) 2020  University of Extremadura, University of Málaga, Blue Ocean Robotics.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

		Author: Renan Freitas, renan028@gmail.com
		Maintainer: Renan Freitas, renan028@gmail.com
*/

#pragma once

#include <mutex>
#include <zmq.hpp>

class QueryClient {
  
  std::mutex mutex;
  zmq::context_t context;
  zmq::socket_t requester;
  int uid;
  std::string request_msg;

  public:
    QueryClient() :
        context(1),
        requester(context, ZMQ_REQ),
        uid(1), 
        request_msg("") {
      requester.connect("tcp://127.0.0.1:5556");
      requester.setsockopt(ZMQ_SNDTIMEO, 500);
      requester.setsockopt(ZMQ_RCVTIMEO, 500);
      requester.setsockopt(ZMQ_REQ_RELAXED, 1);
    }
    
    ~QueryClient() {
      requester.disconnect("tcp://127.0.0.1:5556");
    }

    void setMsg(std::string msg) {
	    std::lock_guard<std::mutex> lck (mutex);
        request_msg = std::move(msg);
    }

    int getID() const {
      return uid;
    }

    void send() {
      std::lock_guard<std::mutex> lck (mutex);
      zmq::message_t zmq_request_msg(request_msg.size());
      memcpy(zmq_request_msg.data(), request_msg.c_str(), request_msg.size());
      if (!requester.send(zmq_request_msg)) {
        std::cout << "send TIMEOUT" << std::endl;
        return;
      }
      std::cout << "message sent" << std::endl;
      zmq::message_t ack;
      if (!requester.recv(&ack)) {
        return;
      }
      std::cout << "ack received" << std::endl;
      uid++;
    }
};

