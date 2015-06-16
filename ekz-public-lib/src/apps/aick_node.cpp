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
#include <iostream>
#include <fstream>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
using namespace std;

bool firstTime;
string bow_path;
string path;
Map3D * m;


class AICKNode
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_Pose;
    ros::Publisher pub_Pose_test;
    ros::Subscriber sub_Points;
    ros::Subscriber sub_Pose;
    int count;
	int counter;
	Map3D * m;
	vector<Matrix4f> lastTransformationMatrix;
	vector<Matrix4f> transformationMatrix;
	tf::StampedTransform tf_camera_link_to_local_origin;
	tf::TransformListener tfl;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped lastPose;
	geometry_msgs::PoseStamped lastLocalPose;
	int nrMatches;
	int lastMatches;
	int badcount;
	boost::circular_buffer<geometry_msgs::PoseStamped> lastPoses;
	double xSpeed, ySpeed, zSpeed;
	ros::Time lastCloudTime;
	Matrix4f frameConversionMat;
    public:
    AICKNode()
        : n("~")
    {
        init();
    }

    ~AICKNode()
    {
    	/*vector<Matrix4f> poses = m->estimate(); //Estimate poses for the frames using the map object.
		char mapbuf[512];
		sprintf(mapbuf,"map.pcd");
		m->savePCD(string(mapbuf));
		cout << "bad frame removed" << badcount << endl;
		ofstream myfile;
		myfile.open ("example.txt");  		
		myfile << "Poses:" << endl;
		for(unsigned int i = 0; i < poses.size(); i++)
		{
			myfile << poses.at(i) << endl << endl;
		}
		myfile.close();*/
	}

    void init()
    {
        sub_Points = n.subscribe("/camera/depth_registered/points", 1, &AICKNode::pointsCallback, this);
        sub_Pose = n.subscribe("/mavros/position/local", 1, &AICKNode::poseCallback, this);
        pub_Pose = n.advertise<geometry_msgs::PoseStamped> ("/mavros/position/vision", 1);
        pub_Pose_test = n.advertise<geometry_msgs::PoseStamped> ("/test", 1);
        count = 0;
        counter = 0;
        firstTime = true;
        m = new Map3D();//bow(bow_path);
        lastTransformationMatrix.push_back(Matrix4f::Identity());
        transformationMatrix = lastTransformationMatrix;
        lastMatches = 0;
        badcount = 0;
    	
        
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "camera_link";
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		/*pose.pose.orientation.x = 1;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 0;*/
		lastPose = pose;
		lastPoses = boost::circular_buffer<geometry_msgs::PoseStamped>(3, pose);
		frameConversionMat << 1,0,0,0, 0,0,1,0, 0,-1,0,0, 0,0,0,1;
		

    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        lastLocalPose = *msg;
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
				int nr_iter = 8;								//Number of iterations the matcher will run
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
				nrMatches = m->numberOfMatchesInLastFrame();
				int hej = m->numberOfFrames();
				/*float time = (input->header.stamp - lastCloudTime).toSec();
				xSpeed = (lastTransformationMatrix.front()(0,3) - transformationMatrix.front()(0,3))/time;
				ySpeed = (lastTransformationMatrix.front()(1,3) - transformationMatrix.front()(1,3))/time;
				zSpeed = (lastTransformationMatrix.front()(2,3) - transformationMatrix.front()(2,3))/time;
				//cout << time << " HEJ" << endl;*/
				if ((nrMatches < 40 and hej > 2)) //or xSpeed > 1.3 or ySpeed > 1.3 or zSpeed > 1.3)
				{
					
					cout << "BAD MATCH" << endl << endl << endl;
					m->removeLastFrame();
					badcount ++;
					//cout << lastPoses.front().pose.position.x << endl;

					
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "local_origin";
					pose.pose.position.x = lastPoses.at(2).pose.position.x + (lastPoses.at(2).pose.position.x - lastPoses.at(1).pose.position.x);
					pose.pose.position.y = lastPoses.at(2).pose.position.y + (lastPoses.at(2).pose.position.y - lastPoses.at(1).pose.position.y);
					pose.pose.position.z = lastPoses.at(2).pose.position.z + (lastPoses.at(2).pose.position.z - lastPoses.at(1).pose.position.z);
					pose.pose.orientation.x = lastPoses.at(2).pose.orientation.x; //lastLocalPose.pose.orientation.x; //q.x();
					pose.pose.orientation.y = lastPoses.at(2).pose.orientation.y; //lastLocalPose.pose.orientation.y; //q.y();
					pose.pose.orientation.z = lastPoses.at(2).pose.orientation.z; //lastLocalPose.pose.orientation.z; //q.z();
					pose.pose.orientation.w = lastPoses.at(2).pose.orientation.w; //lastLocalPose.pose.orientation.w; //q.w();
					//pub_Pose.publish(pose);
					pub_Pose.publish(pose);
				}
				else
				{
					//transformationMatrix = m->estimateCurrentPose(lastTransformationMatrix);
					transformationMatrix = m->estimateCurrentPose(lastTransformationMatrix);
					//cout << transformationMatrix.front() << endl << endl;
					
					lastTransformationMatrix = transformationMatrix;
					transformationMatrix.front() = frameConversionMat*transformationMatrix.front();
					//Convert rotation matrix to quaternion
					tf::Matrix3x3 rotationMatrix;
    				rotationMatrix.setValue(transformationMatrix.front()(0,0), transformationMatrix.front()(0,1),transformationMatrix.front()(0,2),
    					transformationMatrix.front()(1,0), transformationMatrix.front()(1,1),transformationMatrix.front()(1,2),
                        transformationMatrix.front()(2,0), transformationMatrix.front()(2,1),transformationMatrix.front()(2,2) );
					
					tf::Quaternion q;
    				rotationMatrix.getRotation(q);
					//tf::Transform transform;
					//transform.setOrigin(tf::Vector3(transformationMatrix.front()(0,3), transformationMatrix.front()(1,3), transformationMatrix.front()(2,3)));
					
					//publish pose
					//geometry_msgs::PoseStamped pose;
					
					pose.header.stamp = input->header.stamp;
					pose.header.frame_id = "local_origin";
					pose.pose.position.x = transformationMatrix.front()(0,3);
					pose.pose.position.y = transformationMatrix.front()(1,3);
					pose.pose.position.z = transformationMatrix.front()(2,3);
					pose.pose.orientation.x = q.x(); //
					pose.pose.orientation.y = q.y(); //
					pose.pose.orientation.z = q.z(); //
					pose.pose.orientation.w = q.w(); // 
					pub_Pose.publish(pose);
				}
				//pub_transform.sendTransform(tf::StampedTransform(transform, now, "map", "robot"));
				//ros::Time now(0);
				/*while (!tfl.waitForTransform("local_origin", "camera_link", now, ros::Duration(1)))
            		ROS_ERROR("Couldn't find transform from 'camera_link' to 'local_origin', retrying...");
            	geometry_msgs::PoseStamped local_origin_pose;
            	tfl.transformPose("local_origin", now, pose, "camera_link", local_origin_pose);*/
				//pub_Pose.publish(pose);
				//pub_Pose_test.publish(pose);
				lastMatches = nrMatches;
				lastPoses.push_back(pose);
				lastCloudTime = input->header.stamp;
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
	//path = string(argv[2]);

    AICKNode my_node;
    my_node.run();
}
