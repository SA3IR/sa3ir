#include "pick_object.hpp"
#include "specificworker.h"

PickObject::PickObject(SpecificWorker *sw,	const std::string& instance_name,
    				const BT::NodeConfiguration& config) :
  CoroActionNode(instance_name, config),
  read_parameter(false) 
{
    this->sw = sw;
}

BT::NodeStatus PickObject::tick() {
  /*if (!read_parameter) {
    std::string x;

    if(!getInput("x", x))
    {
      throw BT::RuntimeError("Missing parameter in PickObject");
    }*/
   /* if(!getInput("x", x) or !getInput("y", y) or !getInput("z", z))
    {
      throw BT::RuntimeError("Missing parameter in PickObject");
    }*/
   // read_parameter = true;
  //}
    sw->startPickUpAction();
    ActionRes resultAvailable = ActionRes::RUNNING;
    while(resultAvailable == ActionRes::RUNNING)
    {
        setStatusRunningAndYield();
        resultAvailable = sw->checkPickUpResult();
    }
    
    if(resultAvailable == ActionRes::SUCCESS)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

