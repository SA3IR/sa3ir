#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

class SpecificWorker;

class DetectObject : public BT::CoroActionNode
{
public:
  DetectObject(SpecificWorker *sw, const std::string& instance_name,
      const BT::NodeConfiguration& config);

  ~DetectObject() = default;

  BT::NodeStatus tick() override;

  /*static BT::PortsList providedPorts()
  {
    return { 	BT::OutputPort<std::string>("x"),	
		BT::OutputPort<std::string>("y"),
		BT::OutputPort<std::string>("z") };
  }*/
  void halt() override {
    CoroActionNode::halt();
  }

private:
	bool read_parameter;
    SpecificWorker *sw;


};
