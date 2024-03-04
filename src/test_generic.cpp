#include <mrs_uav_gazebo_testing/test_generic.h>

namespace mrs_uav_gazebo_testing
{

UAVHandler::UAVHandler(std::string uav_name, mrs_lib::SubscribeHandlerOptions shopts, bool use_hw_api)
    : mrs_uav_testing::UAVHandler(uav_name, shopts, use_hw_api) {

  // | ------------------- subscribe handlers ------------------- |

  sh_gazebo_spawner_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>(shopts_, "/mrs_drone_spawner/diagnostics");

  // | --------------------- service clients -------------------- |

  sch_spawn_gazebo_uav_ = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "/mrs_drone_spawner/spawn");

  // | -------------------- publish handlers -------------------- |

  ph_gazebo_model_state_ = mrs_lib::PublisherHandler<gazebo_msgs::ModelState>(nh_, "/gazebo/set_model_state");

  initialized_gz_tools_ = true;

  return;
}

/* checkPreconditions() //{ */

std::tuple<bool, std::string> UAVHandler::checkPreconditions(void) {

  auto [success, message] = mrs_uav_testing::UAVHandler::checkPreconditions();

  if (!success) {
    ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: " << message);
    return {false, message};
  }

  if (!initialized_gz_tools_) {
    ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Gazebo UAV handler for " << _uav_name_ << " is not initialized!");
    return {false, "Gazebo UAV handler for " + _uav_name_ + " is not initialized!"};
  }

  if (!spawned_) {
    ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Using Gazebo simulation, but UAV is not spawned!");
    return {false, "Using Gazebo simulation, but UAV is not spawned!"};
  }

  return {true, "All clear."};
}

//}

/* spawn() //{ */

std::tuple<bool, std::string> UAVHandler::spawn(const std::string &gazebo_spawner_params) {

  _gazebo_spawner_params_ = gazebo_spawner_params;

  // | ------------ wait for the spawner to be ready ------------ |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the Gazebo drone spawner", name_.c_str());

    if (sh_gazebo_spawner_diag_.hasMsg()) {
      break;
    }

    sleep(0.01);
  }

  // | -------------------------- wait  ------------------------- |

  sleep(1.0);

  // | ------------------- call spawn service ------------------- |

  {
    mrs_msgs::String srv;

    srv.request.value = _gazebo_spawner_params_;

    bool service_call = sch_spawn_gazebo_uav_.call(srv);

    if (!service_call || !srv.response.success) {
      return {false, "gazebo drone spawner service call failed"};
    }
  }

  // | ------------------ wait while processing ----------------- |

  while (true) {

    if (!ros::ok()) {
      return {false, "shut down from outside"};
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the drone to spawn", name_.c_str());

    if (!sh_gazebo_spawner_diag_.getMsg()->processing) {
      break;
    }

    sleep(0.01);
  }

  if (use_hw_api_) {
    // | ------------- wait for the HW API to connect ------------- |

    while (true) {

      if (!ros::ok()) {
        return {false, "shut down from outside"};
      }

      ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the hw API", name_.c_str());

      if (sh_hw_api_status_.hasMsg()) {
        if (sh_hw_api_status_.getMsg()->connected) {
          break;
        }
      }

      sleep(0.01);
    }

    // | -------------- wait for PX4 to finish bootup ------------- |
  }

  sleep(20.0);

  spawned_ = true;

  return {true, "drone spawned"};
}

//}

/* moveTo() Implements direct transporting of a UAVs in the simulation//{ */

std::tuple<bool, std::string> UAVHandler::moveTo(double x, double y, double z, double hdg) {

  gazebo_msgs::ModelState msg;
  msg.model_name = _uav_name_;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;

  double qw              = cos(hdg / 2.0);
  double qz              = sin(hdg / 2.0);
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  ph_gazebo_model_state_.publish(msg);

  return {true, "Success!"};
}

//}

/* constructor TestGeneric()//{ */

TestGeneric::TestGeneric() : mrs_uav_testing::TestGeneric() {
}

//}

/* getUAVHandler() //{ */

std::tuple<std::optional<std::shared_ptr<UAVHandler>>, std::string> TestGeneric::getUAVHandler(const std::string &uav_name, const bool use_hw_api) {

  if (!initialized_) {
    return {std::nullopt, std::string("Can not obtain UAV handler for  " + uav_name + " - testing is not initialized yet!")};
  } else {
    return {std::make_shared<UAVHandler>(uav_name, shopts_, use_hw_api), "Success!"};
  }
}

//}

/* setRTFactorPercent() sets the Gazebo real-time factor to the percent % of real time. //{ */
std::tuple<bool, std::string> TestGeneric::setRTFactorPercent(double percent){
  double rtfactor = 250.0*(percent/100.0);
  std::string command;
  command = "gz physics -u "+std::to_string((int)(std::round(rtfactor)));
  int status = system(command.c_str());
  if (status == 0){
    return {true, "Success!"};
  }
  else {
    return {false, "Setting of RT factor to "+std::to_string(percent)+"% exited with the code "+std::to_string(status)+"!"};
  }
}
//}


}  // namespace mrs_uav_gazebo_testing
