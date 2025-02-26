#include <ros/ros.h>
#include "websocket_comm/usi.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lipros_node");
  ros::NodeHandle nh;
  Usi interface(nh);
  return 0;
}
