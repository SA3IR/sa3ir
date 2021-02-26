#include "doNothing.hpp"
#include "specificworker.h"

DoNothing::DoNothing(SpecificWorker *sw,	const std::string& instance_name,
    				const BT::NodeConfiguration& config) :
  CoroActionNode(instance_name, config),
  read_parameter(false) 
{
    this->sw = sw;
}

BT::NodeStatus DoNothing::tick() {
    
    //std::cout << "DoNothing::tick()" << std::endl;
    
    return BT::NodeStatus::SUCCESS;
  
}

