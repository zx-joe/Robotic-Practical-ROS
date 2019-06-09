#include <ros/ros.h>
#include "ros_cellulo_coverage/RosCelluloCoverage.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_cellulo_coverage");
    ros::NodeHandle nodeHandle("~");

    /* get name from argv*/
    char* name=argv[1];
    ROS_INFO("init: %s",name);
    RosCelluloCoverage  RosCelluloCoverage(nodeHandle);

    RosCelluloCoverage.mac_Adr=name;
    RosCelluloCoverage.rate=20;
    RosCelluloCoverage.setPublishers();
    RosCelluloCoverage.setSubscribers();


    ros::Rate rate(RosCelluloCoverage.rate);
    while(ros::ok())
    {

        RosCelluloCoverage.calculate_new_velocities();

        //RosCelluloCoverage.calculate_new_velocities_formation();
        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}
