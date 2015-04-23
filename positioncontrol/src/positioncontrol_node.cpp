#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <mavros/State.h>


class positioncontrolnode
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_setpoint;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_state;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped desired_pose;
    mavros::State state;
    bool posLock;
    bool once;


public:
    positioncontrolnode()
        : nh("~")
    {

        pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint/local_position", 1);
        sub_pose = nh.subscribe("/mavros/position/local", 1, &positioncontrolnode::poseCallback, this);
        sub_state = nh.subscribe("/mavros/state", 1, &positioncontrolnode::stateCallback, this);
        once = true;

    }

    ~positioncontrolnode()
    {
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = *msg;
        calc();
    }

    void stateCallback(const mavros::State::ConstPtr& msg)
    {
        state = *msg;
        if (state.mode == "OFFBOARD")
        {
        	static bool posLock = true;
        }
        calc();
    }


    void calc()
    {
        
        ros::Time now(0);
        desired_pose.header.stamp = pose.header.stamp;
        desired_pose.header.frame_id = "local_origin";
        if (posLock and once)
        {
        	once = false;
        	desired_pose.pose.position.x = pose.pose.position.x;
	        desired_pose.pose.position.y = pose.pose.position.y;
	        desired_pose.pose.position.z = pose.pose.position.z;
	        desired_pose.pose.orientation.x = pose.pose.orientation.x;
	        desired_pose.pose.orientation.y = pose.pose.orientation.y;
	        desired_pose.pose.orientation.z = pose.pose.orientation.z;
	        desired_pose.pose.orientation.w = pose.pose.orientation.w;
	        tf::Quaternion pri;
	        double roll, pitch, yaw;
			yaw = tf::getYaw(pose.pose.orientation);
			pri.setRPY(0, 0, yaw);
			std::cout << pri.x() << std::endl;
			std::cout << pri.y() << std::endl;
			std::cout << pri.z() << std::endl;
			std::cout << pri.w() << std::endl;
        }
        else if (once)
        {
	        desired_pose.pose.position.x = 0;
	        desired_pose.pose.position.y = 0;
	        desired_pose.pose.position.z = 0.3;
	        tf::Quaternion q;
	        q.setRPY(0, 0, 3.1415);
	        desired_pose.pose.orientation.x = q.x();
	        desired_pose.pose.orientation.y = q.y();
	        desired_pose.pose.orientation.z = q.z();
	        desired_pose.pose.orientation.w = q.w();
	    }
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
