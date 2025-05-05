#ifndef TEST_GAZEBO_GENERIC_H
#define TEST_GAZEBO_GENERIC_H

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/msg/gazebo_spawner_diagnostics.hpp>

/* #include <gz/msgs10/gz/msgs/pose.pb.h> */

namespace mrs_uav_gazebo_testing
{

class UAVHandler : public mrs_uav_testing::UAVHandler {

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics> sh_gazebo_spawner_diag_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>                sch_spawn_gazebo_uav_;
  /* mrs_lib::PublisherHandler<gz::msg::Pose>                            ph_gazebo_pose_; */

  std::atomic<bool> spawned_              = false;
  std::atomic<bool> initialized_gz_tools_ = false;

  std::string _gazebo_spawner_params_ = "";

public:
  UAVHandler(const rclcpp::Node::SharedPtr node, std::string uav_name, std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts, std::shared_ptr<mrs_lib::Transformer> Transformer, bool use_hw_api);

  std::tuple<bool, std::string> checkPreconditions(void) override;

  std::tuple<bool, std::string> spawn(const std::string& gazebo_spawner_params);

  /* std::tuple<bool, std::string> moveTo(double x, double y, double z, double hdg); */
};

class TestGeneric : public mrs_uav_testing::TestGeneric {

  bool initialized_spawner_params_ = false;

public:
  TestGeneric();

  std::tuple<std::optional<std::shared_ptr<mrs_uav_gazebo_testing::UAVHandler>>, std::string> getUAVHandler(const std::string& uav_name, const bool use_hw_api = true);

  std::tuple<bool, std::string> setRTFactorPercent(double percent);
};

}  // namespace mrs_uav_gazebo_testing

#endif  // TEST_GAZEBO_GENERIC_H
