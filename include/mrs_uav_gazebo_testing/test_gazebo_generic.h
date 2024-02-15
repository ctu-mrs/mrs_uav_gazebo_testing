#ifndef TEST_GAZEBO_GENERIC_H
#define TEST_GAZEBO_GENERIC_H

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/GazeboSpawnerDiagnostics.h>
#include <gazebo_msgs/ModelState.h>


namespace mrs_uav_gazebo_testing
{
class UAVHandlerGZ: public mrs_uav_testing::UAVHandler {

  mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>     sh_gazebo_spawner_diag_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>  sch_spawn_gazebo_uav_;
  mrs_lib::PublisherHandler<gazebo_msgs::ModelState> ph_gazebo_model_state_;
  bool spawned_     = false;
  std::string _gazebo_spawner_params_ = "";
  bool initialized_gz_tools_ = false;

public:
  UAVHandlerGZ(std::string uav_name, mrs_lib::SubscribeHandlerOptions shopts, bool use_hw_api);

  std::tuple<bool, std::string> checkPreconditions(void) override;

  std::tuple<bool, std::string> spawn(const std::string& gazebo_spawner_params);

  std::tuple<bool, std::string> moveTo(double x, double y, double z, double hdg);
};

class TestGenericGZ : public mrs_uav_testing::TestGeneric {

  bool initialized_spawner_params_ = false;
  
public:
  TestGenericGZ();
  std::tuple<std::optional<std::shared_ptr<UAVHandlerGZ>>, std::string> getUAVHandler(const std::string &uav_name, const bool use_hw_api);
};

}  // namespace mrs_uav_gazebo_testing

#endif  // TEST_GAZEBO_GENERIC_H
