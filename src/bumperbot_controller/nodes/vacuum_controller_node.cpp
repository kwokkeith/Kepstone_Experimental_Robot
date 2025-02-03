#include "bumperbot_controller/vacuum_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vacuum_controller_node");
    ros::NodeHandle nh;
    
    VacuumController vacuum_controller(nh);

    ros::spin();
    return 0;
}
