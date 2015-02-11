#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


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
    ros::Subscriber sub_Points;
    int count;
	int counter;
	Map3D * m;
	vector<Matrix4f> lastPose;

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
        pub_Pose = n.advertise<sensor_msgs::PointCloud2> ("/preProcessed/test", 1);
        count = 0;
        counter = 0;
        firstTime = true;
        m = new Map3D();
        lastPose.push_back(Matrix4f::Identity());

    }

    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        if (count >= 6)
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
				vector<Matrix4f> pose = m->estimateCurrentPose(lastPose);
				cout << pose.front() << endl << endl;
				lastPose = pose;
				pub_Pose.publish(input);
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