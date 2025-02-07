#include "bumperbot_controller/rollerbrush_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rollerbrush_controller_node");
    ros::NodeHandle nh;
    
    RollerBrushController rollerbrush_controller(nh);

    ros::spin();
    return 0;
}
