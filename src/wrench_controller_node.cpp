#include "signal.h"
#include "sun_controllers_ros/ROSWrenchController.h"
#include <memory>

// C++11 patch
namespace patch {
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}

std::unique_ptr<sun::ROSWrenchController> wrench_controller;

void mySigintHandler(int sig) {
  wrench_controller->stop();
  ros::shutdown();
}

int main(int argc, char *argv[]) {
  // ROS INIT - Use a coustom signinthandler
  ros::init(argc, argv, "wrench_controller", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigintHandler);

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  wrench_controller =
      patch::make_unique<sun::ROSWrenchController>(nh_public, nh_private);

  wrench_controller->print_params();

  ros::spin();

  return 0;
}
