#include "ekz.h"
#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <pcl/visualization/cloud_viewer.h>


using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "publishPointClouds");
	printf("starting testing software2\n");
	string input = argv[1];
	ros::NodeHandle n;
	ros::Publisher pub_Image = n.advertise<sensor_msgs::Image> ("/camera/image_raw", 1);
	Calibration * cal = new Calibration();
	cal->fx			= 525.0;
	cal->fy			= 525.0;
	cal->cx			= 319.5;
	cal->cy			= 239.5;
	cal->ds			= 1;
	cal->scale		= 5000;

	int k;
	int l;
	int m;
	cout << "Please enter start image: ";
	cin >> k;
	cout << "Please enter end image: " ;
	cin >> l;
	cout << "Enter stepsize: ";
	cin >> m;
	for(int i = k; i <= l; i+= m){
		cout << i << endl;
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);
		FrameInput * fi = new FrameInput(cal, string(rgbbuf) , string(depthbuf));
		pcl::PointCloud<pcl::PointXYZRGB> createdCloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr createdCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);;

		createdCloud = fi->getCloud();
		*createdCloudPtr = createdCloud;
		//*createdCloud.is_dense = true;
		/*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	   	viewer.showCloud (createdCloud);
	   	while (!viewer.wasStopped ())
	   	{
	   	}*/
		sensor_msgs::Image img;
		sensor_msgs::PointCloud2 cloud;// (new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*createdCloudPtr, cloud);
		//pub_PC.publish(cloud);
		pcl::toROSMsg(cloud, img);
		pub_Image.publish(img);
		//usleep(33333);
	}

	//void pcl::toROSMsg	(	const pcl::PointCloud<pcl::PointXYZRGB> & createdCloud, sensor_msgs::Image & img );
	return 0;
}
