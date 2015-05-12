#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>


class fakePose
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_pose;
    ros::Subscriber sub_pose;
    geometry_msgs::PoseStamped pose;
    int count;


public:
    fakePose()
        : nh("~")
    {

        pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/mavros/position/vision", 1);
        sub_pose = nh.subscribe("/mavros/position/local", 1, &fakePose::poseCallback, this);
        count = 0;
    }

    ~fakePose()
    {
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = *msg;
        calc();
    }


    void calc()
    {
        if (count >= 6)
        {
            geometry_msgs::PoseStamped desired_pose;
            ros::Time now(0);
            desired_pose.header.stamp = pose.header.stamp;
            desired_pose.header.frame_id = "local_origin";
            desired_pose.pose.position.x = 0;
            desired_pose.pose.position.y = 0;
            desired_pose.pose.position.z = 0.7;
            /*desired_pose.pose.orientation.x = pose.pose.orientation.x;
            desired_pose.pose.orientation.y = pose.pose.orientation.y;
            desired_pose.pose.orientation.z = pose.pose.orientation.z;
            desired_pose.pose.orientation.w = pose.pose.orientation.w;*/
            pub_pose.publish(desired_pose);
            count = 0;
        }
        else
        {
            count++;
        }
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "fakePose");
  fakePose positioncontrol_node;
  positioncontrol_node.run();
}
