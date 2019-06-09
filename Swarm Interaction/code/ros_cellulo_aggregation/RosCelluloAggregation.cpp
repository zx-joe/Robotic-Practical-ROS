#include "ros_cellulo_aggregation/RosCelluloAggregation.hpp"
#include "tf2_ros/transform_listener.h"

RosCelluloAggregation::RosCelluloAggregation(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
        if (!readParameters()) {

                ROS_ERROR("Could not read parameters :(.");
                ros::requestShutdown();
        }

        //subscribers
	// TODO add subscribers to parameters you want to test in live

        nb_of_robots_detected=-1;
        nb_of_obstacles_detected=-1;
        velocity_updated=false;
        time=0;

}

RosCelluloAggregation::~RosCelluloAggregation(){
}

bool RosCelluloAggregation::readParameters()
{
         if(!nodeHandle_.getParam("scale", scale))
            return false;
        return true;
}


void RosCelluloAggregation::setPublishers()
{
    //publisher
    char publisherTopic_setvelocity[100];
    sprintf(publisherTopic_setvelocity, "/cellulo_node_%s/setGoalVelocity",RosCelluloAggregation::mac_Adr);
    publisher_Velocity = nodeHandle_.advertise<geometry_msgs::Vector3>(publisherTopic_setvelocity,1);

}

void RosCelluloAggregation::setSubscribers()
{
    //subscribers
    char subscriberTopicRobots[100],subscriberTopicObstacles[100],subscriberTopicVelocity[100];
    sprintf(subscriberTopicRobots, "/sensor_node_%s/detectedRobots",mac_Adr);

    sprintf(subscriberTopicObstacles, "/sensor_node_%s/detectedObstacles",mac_Adr);
    subscriber_Robots=nodeHandle_.subscribe(subscriberTopicRobots,1,&RosCelluloAggregation::topicCallback_getDetectedRobots,this);
    subscriber_Obstacles=nodeHandle_.subscribe(subscriberTopicObstacles,1,&RosCelluloAggregation::topicCallback_getDetectedObstacles,this);

    sprintf(subscriberTopicVelocity, "/cellulo_node_%s/velocity",mac_Adr);
    subscriber_Velocity= nodeHandle_.subscribe(subscriberTopicVelocity, 1, &RosCelluloAggregation::topicCallback_getVelocity,this);

}

void RosCelluloAggregation::topicCallback_getDetectedRobots(ros_cellulo_swarm::ros_cellulo_sensor sensor)
{
    distances_to_robots=sensor.Distance;
    nb_of_robots_detected=sensor.detected;
}

void RosCelluloAggregation::topicCallback_getDetectedObstacles(ros_cellulo_swarm::ros_cellulo_sensor sensor)
{
    distances_to_obstacles=sensor.Distance;
    nb_of_obstacles_detected=sensor.detected;

}

void RosCelluloAggregation::topicCallback_getVelocity(geometry_msgs::Vector3 v)
{
    RosCelluloAggregation::velocity.x=v.x;
    RosCelluloAggregation::velocity.y=v.y;
    RosCelluloAggregation::velocity_updated=true;  

}

void RosCelluloAggregation::naive_calculate_new_velocities()
{
    time = time + 1;
    if(nb_of_robots_detected!=-1 && nb_of_obstacles_detected!=-1 && velocity_updated)
    {
        //*****************
        // FILL HERE - Part III Step 1
        //*****************
        double velocity_ = 80;
        geometry_msgs::Vector3 vel_cellulo;
        //std::cout<<(rand()%360)<<"endl";
        static double yaw_angle = 0;//((double)(rand()%360)/180.0)*M_PI;
        if(nb_of_robots_detected >0){
            //if it sees another robot close by a certain distanceth, it waits
            vel_cellulo.x=0;
            vel_cellulo.y=0;
            vel_cellulo.z=0;
        }
        // if wait too longer
        else if(time % 40 == 0){
            //srand(1);
            yaw_angle = ((double)(rand()%360)/180.0)*M_PI;
            //for(int i=0; i<30;i++){
            //yaw_angle = (rand()%(int(360/10)))*10;
            vel_cellulo.x=velocity_*cos(yaw_angle);
            vel_cellulo.y=velocity_*sin(yaw_angle);
            vel_cellulo.z=0;//}
        }
        else
        {
            //yaw_angle = (rand()%360)/2*3.14159;
            //yaw_angle = (rand()%360)/2*3.14159;
            //for(int i=0; i<30;i++){
            vel_cellulo.x=velocity_*cos(yaw_angle);
            vel_cellulo.y=velocity_*sin(yaw_angle);
            vel_cellulo.z=0;/**/
        }
        //std::cout<<(yaw_angle);
        


        ROS_INFO("random number %lf velocity %lf %lf",yaw_angle, vel_cellulo.x,vel_cellulo.y);
        RosCelluloAggregation::publisher_Velocity.publish(vel_cellulo);

        nb_of_robots_detected=-1;
        nb_of_obstacles_detected=-1;
        velocity_updated=false;

    }
    else
    {
        return;
    }
}


void RosCelluloAggregation::field_calculate_new_velocities()
{
   
    if(nb_of_robots_detected!=-1 && nb_of_obstacles_detected!=-1 && velocity_updated)
    {
        //*****************
        // FILL HERE - Part IV 
        //*****************


        geometry_msgs::Vector3 vel;
        ROS_INFO("velocity calcul %lf %lf",vel.x,vel.y);
        RosCelluloAggregation::publisher_Velocity.publish(vel);

        nb_of_robots_detected=-1;
        nb_of_obstacles_detected=-1;
        velocity_updated=false;

    }
    else
    {
        return;
    }
}



double RosCelluloAggregation::norm(double x,double y)
{
        return sqrt(pow(x,2)+pow(y,2));
}
