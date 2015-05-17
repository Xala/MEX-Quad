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
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

using namespace cv;
using namespace std;
ros::Publisher img_pub;

int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }
    ros::init(argc, argv, "preProccessing_node");
    ros::NodeHandle nh;
    img_pub = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 1);

    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    cv_bridge::CvImage out_msg;
	//out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
	out_msg.image    = image; // Your cv::Mat
	int width = 640;
	int height = 480;

	IplImage * rgb_img							= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	Mat * rgb_data								= (Mat *)(rgb_img->imageData);
	//rgb_data = * image;
	while(true)
	{
		img_pub.publish(out_msg.toImageMsg());
	}
    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
