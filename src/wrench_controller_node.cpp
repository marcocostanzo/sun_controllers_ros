#include "sun_controllers_ros/ROSWrenchController.h"
#include "signal.h"
#include <memory>

//C++11 patch
namespace patch{
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}

std::unique_ptr<sun::ROSWrenchController> force_controller;

void mySigintHandler(int sig){ 
    force_controller->stop();
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    //ROS INIT - Use a coustom signinthandler
    ros::init(argc,argv, "force_controller",ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);
     
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    force_controller = patch::make_unique<sun::ROSWrenchController>(
        nh_public,
        nh_private
    );

    force_controller->print_params();

    ros::spin();

    return 0;
}
