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



using namespace std;

class preProccesing
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_Points;
    ros::Subscriber sub_Points;
    int count;


    public:
    preProccesing()
        : n("~")
    {
        init();
    }

    ~preProccesing()
    {

	}

    void init()
    {
        sub_Points = n.subscribe("/camera/depth_registered/points", 1, &preProccesing::rcCallback, this);
        pub_Points = n.advertise<sensor_msgs::PointCloud2> ("/preProcessed/Points", 1);
        count = 0;
    }

    void rcCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        if (count >= 6)
        {


            /*pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
			pcl::fromROSMsg (*input, input_cloud);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			*input_cloud_ptr = input_cloud;*/
            pub_Points.publish(input);

            count = 0;
		}

        else
        {
            count++;
        }
    }

    void run()
    {
        ros::Rate r(30);
        while(ros::ok())
        {
            ros::spin(); //should be 30 fps approx
            //r.sleep();
            //calc();
            //loop_rate.sleep();
        }
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "preProccesing_node");

    preProccesing my_node;
    my_node.run();
}