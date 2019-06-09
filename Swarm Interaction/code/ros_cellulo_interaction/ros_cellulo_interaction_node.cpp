#include <ros/ros.h>
#include "ros_cellulo_swarm/cellulo_touch_key.h"
#include "ros_cellulo_swarm/cellulo_visual_effect.h"
#include "ros_cellulo_swarm/ros_cellulo_sensor.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/Float64.h"


class FollowLeader
{
public:
    explicit FollowLeader(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle){
        if (!readParameters()) {

                ROS_ERROR("Could not read parameters :(.");
                ros::requestShutdown();
        }
        subscriber_setKu = nodeHandle_.subscribe("/setKu", 1, &FollowLeader::topicCallback_setKu, this);
        LeaderSubscriber=nodeHandle_.subscribe("/leader",10,&FollowLeader::leader_callback,this);
        leader_selected=false;
    }

    //Parameters
    std::string myleader;
    std::string myname;
    bool cleared;
    bool leader_selected=false;
    geometry_msgs::Vector3 distance_to_leader; 
    geometry_msgs::Vector3 reference_distance; //the reference distance set to the distance from leader in the callback function of the subscriber to the "Leader" topic

    //Control parameters 
    double Ku; 

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Publishers and Subsribers
    ros::Subscriber LeaderSubscriber;
    ros::Subscriber subscriber_Robots;
    ros::Subscriber subscriber_setKu;
    ros::Publisher VelocityPublisher;
    ros::Publisher clearTrackingPublisher;


    bool readParameters()
    {
            if (!nodeHandle_.getParam("Ku", Ku))
                return false;

            return true;
    }
    void setSubscribersPublishers()
    {
        char subscriberTopicRobots[100];
        sprintf(subscriberTopicRobots, "/sensor_node_%s/detectedRobots",myname.c_str());
        subscriber_Robots=nodeHandle_.subscribe(subscriberTopicRobots,1,&FollowLeader::topicCallback_getDetectedRobots,this);
    
        char publisherTopic_setvelocity[100];
        sprintf(publisherTopic_setvelocity, "/cellulo_node_%s/setGoalVelocity",myname.c_str());
        VelocityPublisher = nodeHandle_.advertise<geometry_msgs::Vector3>(publisherTopic_setvelocity,1);

        char publisherTopic_clearTracking[100];
        sprintf(publisherTopic_clearTracking, "/cellulo_node_%s/clearTracking",myname.c_str());
        clearTrackingPublisher = nodeHandle_.advertise<std_msgs::Empty>(publisherTopic_clearTracking,1);
    }

    void leader_callback(std_msgs::String message)
    {
        myleader=message.data;
        cleared=false;
        leader_selected=true;
        reference_distance=distance_to_leader;

    }
    void topicCallback_setKu(const std_msgs::Float64 & message)
    {
        Ku=message.data;
    }
    void clearTracking(){
        std_msgs::Empty clear;
        clearTrackingPublisher.publish(clear);
        cleared=true;

    }

    void topicCallback_getDetectedRobots(ros_cellulo_swarm::ros_cellulo_sensor sensor)
    {
        if(sensor.detected!=0)
            distance_to_leader = sensor.Distance[0];
    }

    void followingMyLeader()
    {
        // Implement here your control. 
        // 1- Calculate the required velocity
        // (Useful variables: Ku, distance_to_leader and reference_distance)
        // 2- Publish the velocity
        float vel_x;
        float vel_y;
        vel_x = Ku*(distance_to_leader.x - reference_distance.x);
        vel_y = Ku*(distance_to_leader.y - reference_distance.y);
        geometry_msgs::Vector3 vel_cellulo;
        vel_cellulo.x= vel_x;
        vel_cellulo.y= vel_y;
        vel_cellulo.z = 0.0;

        VelocityPublisher.publish(vel_cellulo);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_cellulo_interaction");
    ros::NodeHandle nodeHandle("~");

    FollowLeader followLeader(nodeHandle);

    followLeader.myname=argv[1];
    followLeader.setSubscribersPublishers();
    ros::Rate rate(20);
    while(ros::ok())
    {
        if(followLeader.leader_selected)
        {
            if(followLeader.myleader!=followLeader.myname)
                followLeader.followingMyLeader();
            else {
                if(!followLeader.cleared)
                    followLeader.clearTracking();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
