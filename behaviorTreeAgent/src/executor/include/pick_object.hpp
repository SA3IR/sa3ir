#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

class SpecificWorker;

class PickObject : public BT::CoroActionNode
{
public:
  PickObject(SpecificWorker *sw, const std::string& instance_name,
      const BT::NodeConfiguration& config);

  ~PickObject() = default;

  BT::NodeStatus tick() override;

  /*static BT::PortsList providedPorts()
  {
    return { 	BT::InputPort<std::string>("x"),	
		BT::InputPort<std::string>("y"),
		BT::InputPort<std::string>("z") };
  }*/
  void halt() override {
    CoroActionNode::halt();
  }

private:
	bool read_parameter;
    SpecificWorker *sw;


};
