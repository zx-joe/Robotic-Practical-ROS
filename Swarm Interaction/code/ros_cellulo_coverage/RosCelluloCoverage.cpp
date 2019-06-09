#include "ros_cellulo_coverage/RosCelluloCoverage.hpp"
#include "tf2_ros/transform_listener.h"

RosCelluloCoverage::RosCelluloCoverage(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
        if (!readParameters()) {

                ROS_ERROR("Could not read parameters :(.");
                ros::requestShutdown();
        }
        //subscribers
        subscriber_setMass = nodeHandle_.subscribe(subscriberTopic_setMass, 1, &RosCelluloCoverage::topicCallback_setMass, this);
        subscriber_setViscocity=nodeHandle_.subscribe(subscriberTopic_setViscocity,1,&RosCelluloCoverage::topicCallback_setViscocity,this);
        subscriber_setFieldStrengthObstacles=nodeHandle_.subscribe(subscriberTopic_setFieldStrengthObstacles,1,&RosCelluloCoverage::topicCallback_setFieldStrengthObstacles,this);
        subscriber_setFieldStrengthRobots=nodeHandle_.subscribe(subscriberTopic_setFieldStrengthRobots,1,&RosCelluloCoverage::topicCallback_setFieldStrengthRobots,this);

        nb_of_robots_detected=-1;
        nb_of_obstacles_detected=-1;
        velocity_updated=false;

}

RosCelluloCoverage::~RosCelluloCoverage(){
}

bool RosCelluloCoverage::readParameters()
{
        if (!nodeHandle_.getParam("ko",ko))
            return false;
        else if(!nodeHandle_.getParam("kr", kr))
            return false;
        else if(!nodeHandle_.getParam("m", m))
            return false;
        else if(!nodeHandle_.getParam("mu", mu))
            return false;
        else if(!nodeHandle_.getParam("scale", scale))
            return false;

        return true;
}

void RosCelluloCoverage::topicCallback_setMass(const std_msgs::Float64 & message)
{
        m=message.data;
}

void RosCelluloCoverage::topicCallback_setViscocity(const std_msgs::Float64 &message)
{
        mu=message.data;
}

void RosCelluloCoverage::topicCallback_setFieldStrengthObstacles(const std_msgs::Float64 &message)
{
        ko=message.data;
}

void RosCelluloCoverage::topicCallback_setFieldStrengthRobots(const std_msgs::Float64 &k)
{
        kr=k.data;
}

void RosCelluloCoverage::setPublishers()
{
    //publisher
    char publisherTopic_setvelocity[100];
    sprintf(publisherTopic_setvelocity, "/cellulo_node_%s/setGoalVelocity",RosCelluloCoverage::mac_Adr);
    publisher_Velocity = nodeHandle_.advertise<geometry_msgs::Vector3>(publisherTopic_setvelocity,1);

}

void RosCelluloCoverage::setSubscribers()
{
    //subscribers
    char subscriberTopicRobots[100],subscriberTopicObstacles[100],subscriberTopicVelocity[100];
    sprintf(subscriberTopicRobots, "/sensor_node_%s/detectedRobots",mac_Adr);
    //ROS_INFO("=====-%s",subscriberTopicRobots);
    sprintf(subscriberTopicObstacles, "/sensor_node_%s/detectedObstacles",mac_Adr);
    subscriber_Robots=nodeHandle_.subscribe(subscriberTopicRobots,1,&RosCelluloCoverage::topicCallback_getDetectedRobots,this);
    subscriber_Obstacles=nodeHandle_.subscribe(subscriberTopicObstacles,1,&RosCelluloCoverage::topicCallback_getDetectedObstacles,this);

    sprintf(subscriberTopicVelocity, "/cellulo_node_%s/velocity",mac_Adr);
    subscriber_Velocity= nodeHandle_.subscribe(subscriberTopicVelocity, 1, &RosCelluloCoverage::topicCallback_getVelocity,this);

}

void RosCelluloCoverage::topicCallback_getDetectedRobots(ros_cellulo_swarm::ros_cellulo_sensor sensor)
{
    distances_to_robots=sensor.Distance;
    nb_of_robots_detected=sensor.detected;
}

void RosCelluloCoverage::topicCallback_getDetectedObstacles(ros_cellulo_swarm::ros_cellulo_sensor sensor)
{
    distances_to_obstacles=sensor.Distance;
    nb_of_obstacles_detected=sensor.detected;
}

void RosCelluloCoverage::topicCallback_getVelocity(geometry_msgs::Vector3 v)
{
    RosCelluloCoverage::velocity.x=v.x;
    RosCelluloCoverage::velocity.y=v.y;
    RosCelluloCoverage::velocity_updated=true;

}

void RosCelluloCoverage::calculate_new_velocities()
{
    //// TO IMPLEMENT ////
    if(nb_of_robots_detected!=-1 && nb_of_obstacles_detected!=-1 && velocity_updated)
    {
        float Fox=0;
        float Foy = 0;
        float Fnx = 0;
        float Fny = 0;
        float Vx = 0;
        float Vy = 0;

        for(int i=0;i<nb_of_obstacles_detected;i++)
        {
            Fox=Fox+ko*(distances_to_obstacles[i]).x;
            Foy=Foy+ko*(distances_to_obstacles[i]).y;

        }

        for(int i=0;i<nb_of_robots_detected;i++)
        {
            Fnx=Fnx+kr*(distances_to_robots[i].x);
            Fny=Fny+kr*(distances_to_robots[i].y);  

        }
        
        float Fx= Fox+Fnx;// fro attractive you just need to change the minus sign
        float Fy = Foy +Fny;

        float t = 1/rate; 
        Vx = Vx*(1-float(t)*mu/m) + Fx*t/m;
        Vy = Vy*(1-float(t)*mu/m) + Fy*t/m;

        geometry_msgs::Vector3 vel;
        vel.x = Vx;
        vel.y = Vy;
        vel.z = 0;

        RosCelluloCoverage::publisher_Velocity.publish(vel);

 
        nb_of_robots_detected=-1;
        nb_of_obstacles_detected=-1;
        velocity_updated=false;
    }

}


geometry_msgs::Vector3 RosCelluloCoverage::limit_velocity(geometry_msgs::Vector3 v,double limit)
{
    // TO DO //
    //Limit speed to a maximum (300 mm/s). NB: direction should not change, only the norm.
    //You can also put a lower bound for the speed to avoid small in-place oscillations
    geometry_msgs::Vector3 newV;
    return newV;
}

double RosCelluloCoverage::norm(double x,double y)
{
        return sqrt(pow(x,2)+pow(y,2));
}
