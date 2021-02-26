#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

class GetPosition : public BT::SyncActionNode
{
public:
  GetPosition(const std::string& instance_name,
      const BT::NodeConfiguration& config);

  ~GetPosition() = default;

  BT::NodeStatus tick() override;

  /*static BT::PortsList providedPorts()
  {
    return { 	BT::OutputPort<std::string>("x"),	
		BT::OutputPort<std::string>("y") };
  }*/

private:
	bool read_parameter;


};
