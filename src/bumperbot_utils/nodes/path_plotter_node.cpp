#include <ros/ros.h>
#include "bumperbot_utils/path_plotter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_plotter");
    ros::NodeHandle nh;

    PathPlotter pathPlotter(nh);

    ros::spin();
    return 0;
};