#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

class SpecificWorker;

class ApproachObject : public BT::CoroActionNode
{
public:
  ApproachObject(SpecificWorker* sw, const std::string& instance_name,
      const BT::NodeConfiguration& config);

  ~ApproachObject() = default;

  BT::NodeStatus tick() override;

  void halt() override {
    CoroActionNode::halt();
  }
  
private:
	bool read_parameter;
    SpecificWorker* sw;


};
