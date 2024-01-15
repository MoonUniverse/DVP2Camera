#include "dvp2camera.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dvp2camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    dvp2_camera::DVP2Camera dvp2camera(nh, nh_private);
    ros::spin();
    return 0;
}