#include <ros/ros.h>
#include "ros_cellulo_aggregation/RosCelluloAggregation.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_cellulo_aggregation");
    ros::NodeHandle nodeHandle("~");

    /* get name from argv*/
    char* name=argv[1];
    ROS_INFO("init: %s",name);
    RosCelluloAggregation  RosCelluloAggregation(nodeHandle);

    RosCelluloAggregation.mac_Adr=name;
    RosCelluloAggregation.rate=20;
    RosCelluloAggregation.setPublishers();
    RosCelluloAggregation.setSubscribers();




    ros::Rate rate(RosCelluloAggregation.rate);
    while(ros::ok())
    {

        RosCelluloAggregation.naive_calculate_new_velocities(); //  Part III step 1
	      //RosCelluloAggregation.fourstates_calculate_new_velocities(); //  Part III step 2
        //RosCelluloAggregation.field_calculate_new_velocities(); //  Part IV Bonus
        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}
