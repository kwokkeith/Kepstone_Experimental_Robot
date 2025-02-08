#include "bumperbot_controller/sidebrush_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sidebrush_controller_node");
    ros::NodeHandle nh;
    
    SideBrushController sidebrush_controller(nh);

    ros::spin();
    return 0;
}
