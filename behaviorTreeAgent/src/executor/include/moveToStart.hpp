#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

class SpecificWorker;

class MoveToStart : public BT::CoroActionNode
{
public:
  MoveToStart(SpecificWorker* sw, const std::string& instance_name,
      const BT::NodeConfiguration& config);

  ~MoveToStart() = default;

  BT::NodeStatus tick() override;

 /* static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("angle"),
	BT::InputPort<std::string>("x"),	
	BT::InputPort<std::string>("z")};
  }*/
   void halt() override {
    CoroActionNode::halt();
  }

private:
	bool read_parameter;
    SpecificWorker* sw;


};
