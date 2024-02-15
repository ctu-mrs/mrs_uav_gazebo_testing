#include <gtest/gtest.h>
#include <mrs_uav_gazebo_testing/test_gazebo_generic.h>

class Tester : public mrs_uav_gazebo_testing::TestGenericGZ {

public:
  bool test();
};

bool Tester::test() {

  std::shared_ptr<mrs_uav_gazebo_testing::UAVHandlerGZ> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }
  
  {
    std::string _gazebo_spawner_params;
    pl_->loadParam("gazebo_spawner_params", _gazebo_spawner_params, std::string());
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Spawning " << _uav_name_);
    auto [success, message] = uh->spawn(_gazebo_spawner_params);
    if (!success){
      ROS_ERROR("[%s]: UAV failed to spawn: %s", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  auto [success, message] = uh->takeoff();

  if (!success) {
    ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
    return false;
  }

  sleep(5.0);

  if (uh->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
  }
}


TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}