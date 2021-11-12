#include "ros_sdk_driver.h"

int main(int argc, char *argv[])
{
    /* Config File Location */
    std::string configPath = "/home/nvidia/catkin_ws/src/oculii_ros_driver/ROS_DRIVER_WS/config.xml";

    /* ROS initialization */
    ros::init(argc, argv, "main");
    
    /* ROS nodehandler */
    ros::NodeHandle nh("~");

    /* ROS SDK Driver setup */
    SdkDriver drv(configPath);
    ROS_INFO("end initialization");

    /* Threads to run ROS Publishers */

    /* Publish Point Cloud and Tracker */
    std::thread thRadarPub = std::thread(&SdkDriver::PubRadarData, drv);

    /* Publish Image if available. Uncomment the following line*/
    //std::thread thImgPub = std::thread(&SdkDriver::PubImg, drv);

    /* Publish Enhanced Point Cloud */
    std::thread thRadarEnhancedPub = std::thread(&SdkDriver::PubRadarEnhancedPcl, drv);

    /* Publish Axis */
    std::thread thAxisPub = std::thread(&SdkDriver::PubAxis, drv);
    ros::spin();

    return 0;
}
