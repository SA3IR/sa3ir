#include "validEstimatedPosition.hpp"
#include "specificworker.h"

ValidEstimatedPosition::ValidEstimatedPosition(SpecificWorker *sw,	const std::string& instance_name,
    				const BT::NodeConfiguration& config) :
  CoroActionNode(instance_name, config),
  read_parameter(false) 
{
    this->sw = sw;
}

BT::NodeStatus ValidEstimatedPosition::tick() {
    
    if(sw->robotAtGoodPositionFromMark())
         return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
    
}

