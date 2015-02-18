#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>


//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>
#include "ekz.h"

using namespace std;

bool firstTime;
string bow_path;
Map3D * m;


class AICKNode
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_Pose;
    ros::Publisher pub_Pose_test;
    ros::Subscriber sub_Points;
    int count;
	int counter;
	Map3D * m;
	vector<Matrix4f> lastRotationMatrix;
	tf::StampedTransform tf_camera_link_to_local_origin;
	tf::TransformListener tfl;
    public:
    AICKNode()
        : n("~")
    {
        init();
    }

    ~AICKNode()
    {
    	vector<Matrix4f> poses = m->estimate(); //Estimate poses for the frames using the map object.
		char mapbuf[512];
		sprintf(mapbuf,"map.pcd");
		m->savePCD(string(mapbuf));
		cout << "Poses:" << endl;
		for(unsigned int i = 0; i < poses.size(); i++)
		{
			cout << poses.at(i) << endl << endl;
		}
	}

    void init()
    {
        sub_Points = n.subscribe("/camera/depth_registered/points", 1, &AICKNode::pointsCallback, this);
        pub_Pose = n.advertise<geometry_msgs::PoseStamped> ("/mavros/position/vision", 1);
        pub_Pose_test = n.advertise<geometry_msgs::PoseStamped> ("/test", 1);
        count = 0;
        counter = 0;
        firstTime = true;
        m = new Map3D();
        lastRotationMatrix.push_back(Matrix4f::Identity());

    }

    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        if (count >= 0)
        {
            counter++;


            pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
			pcl::fromROSMsg (*input, input_cloud);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			*input_cloud_ptr = input_cloud;

            count = 0;
            if (firstTime == true)
	    	{
		    		//Create a standard map object
				m->setVerbose(true);		//Set the map to give text output

				m->loadCalibrationWords(bow_path,"orb", 500);	//set bag of words to orb 500 orb features from bow_path
				m->setFeatureExtractor(new OrbExtractor());		//Use orb features

				int max_points = 300;							//Number of keypoints used by matcher
				int nr_iter = 10;								//Number of iterations the matcher will run
				float shrinking = 0.7;							//The rate of convergence for the matcher
				float bow_threshold = 0.15;						//Bag of words threshold to avoid investigating bad matches
				float distance_threshold = 0.015;				//Distance threshold to discard bad matches using euclidean information.
				float feature_threshold = 0.15;					//Feature threshold to discard bad matches using feature information.

				m->setMatcher(new BowAICK(max_points, nr_iter,shrinking,bow_threshold,distance_threshold,feature_threshold));//Create a new matcher
				firstTime = false;
				vector< RGBDFrame * > frames;  
				m->addFrame(input_cloud_ptr); 		
	    	}
	    	else
	    	{
		    	
				printf("----------------------%i-------------------\nadding a new frame\n",counter);

				//Add frame to map
				m->addFrame(input_cloud_ptr);
				vector<Matrix4f> rotationMatrix = m->estimateCurrentPose(lastRotationMatrix);
				cout << rotationMatrix.front() << endl << endl;
				lastRotationMatrix = rotationMatrix;
				//Convert rotation matrix to quaternion
				double q4 = 0.5 * sqrt(1 + rotationMatrix.front()(0,0) + rotationMatrix.front()(1,1) + rotationMatrix.front()(2,2));
				double q1 = (1/(4*q4))*(rotationMatrix.front()(2,1) - rotationMatrix.front()(1,2));
				double q2 = (1/(4*q4))*(rotationMatrix.front()(0,2) - rotationMatrix.front()(2,0));
				double q3 = (1/(4*q4))*(rotationMatrix.front()(1,0) - rotationMatrix.front()(0,1));
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(rotationMatrix.front()(0,3), rotationMatrix.front()(1,3), rotationMatrix.front()(2,3)));
				tf::Quaternion q;
				q.setX(q1);
				q.setY(q2);
				q.setZ(q3);
				q.setW(q4);
				transform.setRotation(q);
				ros::Time now(0);
				//publish pose
				geometry_msgs::PoseStamped pose;
				pose.header.stamp = input->header.stamp;
				pose.header.frame_id = "camera_link";
				pose.pose.position.x = 0;//rotationMatrix.front()(0,3);
				pose.pose.position.y = 0;//rotationMatrix.front()(1,3);
				pose.pose.position.z = 0;//rotationMatrix.front()(2,3);
				pose.pose.orientation.x = q.x();
				pose.pose.orientation.y = q.y();
				pose.pose.orientation.z = q.z();
				pose.pose.orientation.w = q.w();
				
				//pub_transform.sendTransform(tf::StampedTransform(transform, now, "map", "robot"));
				
				while (!tfl.waitForTransform("local_origin", "camera_link", now, ros::Duration(1)))
            		ROS_ERROR("Couldn't find transform from 'camera_link' to 'local_origin', retrying...");
            	geometry_msgs::PoseStamped local_origin_pose;
            	tfl.transformPose("local_origin", now, pose, "camera_link", local_origin_pose);
				pub_Pose.publish(local_origin_pose);
				pub_Pose_test.publish(pose);
			}
		}
        else
        {
            count++;
        }
    }

    void run()
    {
        while(ros::ok())
        {
            ros::spin(); //should be 30 fps approx

            //calc();
            //loop_rate.sleep();
        }
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "aick_node");
	bow_path = argv[1];

    AICKNode my_node;
    my_node.run();
}