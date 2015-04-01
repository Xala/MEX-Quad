#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>


class positioncontrolnode
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_setpoint;
    ros::Subscriber sub_pose;
    geometry_msgs::PoseStamped pose;


public:
    positioncontrolnode()
        : nh("~")
    {

        pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint/local_position", 1);
        sub_pose = nh.subscribe("/mavros/position/local", 1, &positioncontrolnode::poseCallback, this);
    }

    ~positioncontrolnode()
    {
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = *msg;
        calc();
    }


    void calc()
    {
        geometry_msgs::PoseStamped desired_pose;
        ros::Time now(0);
        desired_pose.header.stamp = pose.header.stamp;
        desired_pose.header.frame_id = "local_origin";
        desired_pose.pose.position.x = 0;
        desired_pose.pose.position.y = 0;
        desired_pose.pose.position.z = 0.7;
        desired_pose.pose.orientation.x = pose.pose.orientation.x;
        desired_pose.pose.orientation.y = pose.pose.orientation.y;
        desired_pose.pose.orientation.z = pose.pose.orientation.z;
        desired_pose.pose.orientation.w = pose.pose.orientation.w;
        pub_setpoint.publish(desired_pose);
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "positioncontrolnode");
  positioncontrolnode positioncontrol_node;
  positioncontrol_node.run();
}
