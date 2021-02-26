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

    Authors:  Antonio Bandera, ajbandera@uma.es 
              Renan Freitas, renan028@gmail.com
    Maintainer: Renan Freitas, renan028@gmail.com
*/

#include "zhelpers.hpp"
#include "positions.hpp"
#include "change_velocity.hpp"
#include <thread>
#include <mutex>
#include <signal.h>
#include <unistd.h>
#include <condition_variable> // std::condition_variable
#include "variant_client.hpp"
#include "query_client.hpp"
#include <abort_current_skill.hpp>
#include "mironDDS_listener.hpp"


using namespace zmqserver;

std::mutex mtx;
std::condition_variable cv;
bool finish = false;

void handler(int s){
    std::unique_lock<std::mutex> lck(mtx);
    std::cout << "Control-C capturado"  << std::endl;
    finish = true;
    cv.notify_one();
}

int main(int argc, char *argv[])
{
    struct sigaction sigIntHandler;
    auto variant_client = std::make_shared<VariantClient>();
    auto query_client = std::make_shared<QueryClient>();
    MironDDSListener mironListener(query_client, variant_client);

    sigIntHandler.sa_handler = handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
   
    std::unique_lock<std::mutex> lck(mtx);
    while(!finish) {
        std::cout << "Waiting..."  << std::endl;
       /* std::string action;
        std::cin>>action;
        finish = true;
        query_client->setMsg("abort_current_action");
        query_client->send();
        sleep(10);
        variant_client->sendVariant(action);*/
        cv.wait(lck);
    }
    variant_client->stop();
    std::cout << "Bye!" << std::endl;
    
}
