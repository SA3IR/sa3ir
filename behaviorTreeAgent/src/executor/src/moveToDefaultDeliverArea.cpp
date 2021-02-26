#include "moveToDefaultDeliverArea.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "specificworker.h"

MoveToDefaultDeliverArea::MoveToDefaultDeliverArea(SpecificWorker* sw, const std::string& instance_name,
    const BT::NodeConfiguration& config) :
  CoroActionNode(instance_name, config),
  read_parameter(false) 
{
    this->sw = sw;
}

BT::NodeStatus MoveToDefaultDeliverArea::tick() {
   
  
    sw->startNavigationAction("moveToDefaultDeliverArea");
    ActionRes resultAvailable = ActionRes::RUNNING;
    
    while(resultAvailable == ActionRes::RUNNING)
    {
        setStatusRunningAndYield();
        resultAvailable = sw->checkNavigationResult();
    }
    if(resultAvailable == ActionRes::SUCCESS)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

