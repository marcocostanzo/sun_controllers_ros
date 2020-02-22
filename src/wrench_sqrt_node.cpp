#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"

double smart_sqrt(double x);
void get_mask(const ros::NodeHandle &nh_private);

ros::Publisher pub;
bool mask[6] = { false, false, false, false, false, false };
double m;

void sub_cb(geometry_msgs::WrenchStamped msg)
{
  if (mask[0])
  {
    msg.wrench.force.x = smart_sqrt(msg.wrench.force.x);
  }
  if (mask[1])
  {
    msg.wrench.force.y = smart_sqrt(msg.wrench.force.y);
  }
  if (mask[2])
  {
    msg.wrench.force.z = smart_sqrt(msg.wrench.force.z);
  }
  if (mask[3])
  {
    msg.wrench.torque.x = smart_sqrt(msg.wrench.torque.x);
  }
  if (mask[4])
  {
    msg.wrench.torque.y = smart_sqrt(msg.wrench.torque.y);
  }
  if (mask[5])
  {
    msg.wrench.torque.z = smart_sqrt(msg.wrench.torque.z);
  }
  pub.publish(msg);
}

int main(int argc, char *argv[])
{
  // ROS INIT - Use a coustom signinthandler
  ros::init(argc, argv, "wrench_sqrt");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public;

  std::string tipic_in_str, topic_out_str;

  bool b = nh_private.getParam("in_topic", tipic_in_str);
  b = b && nh_private.getParam("out_topic", topic_out_str);
  get_mask(nh_private);
  b = b && nh_private.getParam("max_angular_coeff", m);

  if (!b || m < 0.0)
  {
    ROS_ERROR("[wrench SQRT] Parameter error");
    ros::shutdown();
    return -1;
  }

  ros::Subscriber sub = nh_public.subscribe(tipic_in_str, 1, sub_cb);

  pub = nh_public.advertise<geometry_msgs::WrenchStamped>(topic_out_str, 1);

  ros::spin();

  return 0;
}

void get_mask(const ros::NodeHandle &nh_private)
{
  std::vector<bool> mask_std;
  if (!nh_private.getParam("mask", mask_std))
  {
    ROS_ERROR("[wrench SQRT] INVALID PARAM: mask has to be a "
              "vector<bool> [...]");
    ros::shutdown();
    exit(-1);
  }
  if (mask_std.size() != 6)
  {
    ROS_ERROR("[wrench SQRT] INVALID PARAM: mask length has to be 6");
    ros::shutdown();
    exit(-1);
  }
  for (int i = 0; i < 6; i++)
    mask[i] = mask_std[i];
}

double smart_sqrt(double x)
{
  if (m == 0.0)
  {
    return 0.0;
  }
  double x_sat = pow(m, -2.0);
  if (x < -x_sat)
  {
    return -sqrt(-x);
  }
  if (x <= x_sat)
  {
    return m * x;
  }
  return sqrt(x);
}