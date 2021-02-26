#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>

class SpecificWorker;

class Dock : public BT::CoroActionNode
{
public:
  Dock(SpecificWorker* sw, const std::string& instance_name,
      const BT::NodeConfiguration& config);

  ~Dock() = default;

  BT::NodeStatus tick() override;

  void halt() override {
    CoroActionNode::halt();
  }
  
private:
    bool read_parameter;
    SpecificWorker* sw;


};
