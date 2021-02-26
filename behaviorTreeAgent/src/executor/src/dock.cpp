#include "deliver_object.hpp"
#include "specificworker.h"

Dock::Dock(	SpecificWorker* sw, const std::string& instance_name,
    				const BT::NodeConfiguration& config) :
  CoroActionNode(instance_name, config),
  read_parameter(false)
{
 this->sw = sw;
    
}

BT::NodeStatus Dock::tick() {
    
    sw->startNavigationAction("moveToDock", true);
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

