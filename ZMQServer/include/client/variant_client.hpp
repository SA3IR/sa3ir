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

#include "zhelpers.hpp"
#include <thread>
#include <mutex>
#include <json.hpp>
#include <atomic>

class VariantClient
{
  std::thread thread_;
  std::mutex mutex;
  zmq::context_t context;
  zmq::socket_t request_socket;
  zmq::socket_t reply_socket;
  std::string variant_;
  unsigned id_;
  bool connection_ok;
  std::atomic<bool> finish_thread;

  void main() {
    while(!finish_thread) {
        zmq::message_t request;
        zmq::message_t ack;
            
        if (!request_socket.recv(&request)) {
            return;
        } 
        std::string rcv_msg(static_cast<const char*>(request.data()), request.size());
    
        if(!rcv_msg.compare("finish"))
            continue;
        
        if(request_socket.send(ack))
            connection_ok = true;
        
        id_ = [](std::string result_string) -> unsigned {
            nlohmann::json json = nlohmann::json::parse(std::move(result_string));
            unsigned msg_id = json["id"];
            std::string msg_type  = json["msg-type"];
            return msg_id;
        }(std::move(rcv_msg));
    }
  }

  public:
    VariantClient() :
        context(1),
        request_socket(context, ZMQ_REP),
        reply_socket(context, ZMQ_PUB),
        variant_(""),
        connection_ok(false),
        finish_thread(false){
      request_socket.bind("tcp://127.0.0.1:8274");
      reply_socket.bind("tcp://127.0.0.1:8275");
      thread_ = std::thread(&VariantClient::main, this);
    }

    ~VariantClient() {
      thread_.join(); 
      //request_socket.unbind("tcp://127.0.0.1:8274"); //not working
      //reply_socket.unbind("tcp://127.0.0.1:8275");//not working
    }

    void sendVariant(std::string value) {
      const std::lock_guard<std::mutex> lock(mutex);
      variant_ = std::move(value);
      std::cout << "enviando.." << std::endl;
      if(connection_ok) {
          std::cout << "enviando connection ok.." << std::endl;
        auto reply_msg = [&]() -> std::string {
          nlohmann::json json;
          json["msg-type"] = "variant";
          json["id"] = id_;
          auto& result = json["result"] = nlohmann::json({});
          result["value"] = variant_;
          return json.dump(1);
        }();
        zmq::message_t reply(reply_msg.size());
        memcpy(reply.data(), reply_msg.c_str(), reply_msg.size());
        reply_socket.send(reply);
        std::cout << "enviado.." << std::endl;
      }
    }
    
    void stop() {
        zmq::socket_t request_socket(context, ZMQ_REQ);
        request_socket.connect("tcp://127.0.0.1:8274");
        std::string str_msg("finish");
        zmq::message_t message(str_msg.size());
        memcpy(message.data(), str_msg.c_str(), str_msg.size());
        finish_thread = true;  
        request_socket.send(message);
        request_socket.disconnect("tcp://127.0.0.1:8274");        
    }
};
