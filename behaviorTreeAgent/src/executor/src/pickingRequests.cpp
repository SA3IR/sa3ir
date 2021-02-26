#include "pickingRequests.hpp"
#include "specificworker.h"

PickingRequests::PickingRequests(SpecificWorker *sw,	const std::string& instance_name,
    				const BT::NodeConfiguration& config) :
  CoroActionNode(instance_name, config),
  read_parameter(false) 
{
    this->sw = sw;
}

BT::NodeStatus PickingRequests::tick() {
    
    //std::cout << "PickingRequests::tick()" << std::endl;
    
    //return BT::NodeStatus::SUCCESS;
    if (sw->startPickDeliverStart())
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
  
}

