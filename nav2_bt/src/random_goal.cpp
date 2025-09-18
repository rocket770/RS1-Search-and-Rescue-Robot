#include "nav2_bt/random_goal.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <pluginlib/class_list_macros.hpp>

namespace nav2_bt {
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<RandomGoal>("RandomGoal");
}
} // namespace nav2_bt

// Export against ActionNodeBase is fine once a default ctor exists
PLUGINLIB_EXPORT_CLASS(nav2_bt::RandomGoal, BT::ActionNodeBase)
