#include "litter_destruction/boundary_visualizer.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "boundary_visualizer");
    ros::NodeHandle nh;

    BoundaryVisualizer visualizer(nh);

    ros::spin();
    return 0;
}