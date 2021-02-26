#include "get_position.hpp"

GetPosition::GetPosition(	const std::string& instance_name,
    				const BT::NodeConfiguration& config) :
  SyncActionNode(instance_name, config),
  read_parameter(false) {}

BT::NodeStatus GetPosition::tick() {

  /*setOutput("x", "0.0");
  setOutput("y", "0.0");
 */
  return BT::NodeStatus::SUCCESS;
}

