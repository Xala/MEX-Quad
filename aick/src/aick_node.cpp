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

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>
#include "ekz.h"

using namespace std;


class AICKNode
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_RC;
    ros::Subscriber sub_Points;
    int count;
    int counter;


    public:
    AICKNode()
        : n("~")
    {
        init();
    }

    ~AICKNode()
    {
    }

    void init()
    {
        sub_Points = n.subscribe("/camera/depth_registered/points", 1, &AICKNode::rcCallback, this);
        count = 0;
        counter = 0;
    }

    void rcCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        if (count >= 5)
        {
            counter++;
            pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
            pcl::fromROSMsg (*input, input_cloud);
            //Read data into images and save.

            int width = input_cloud.width;
            int height = input_cloud.height;

            IplImage * rgb_img                          = cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_8U, 3);
            char * rgb_data                             = (char *)(rgb_img->imageData);
            IplImage * depth_img                        = cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_16U, 1);
            unsigned short * depth_data                 = (unsigned short *)(depth_img->imageData);

            for(int w = 0; w < width; w++){
                for(int h = 0; h < height; h++){
                    int ind = h*input_cloud.width + w;
                    rgb_data[3*ind+0] = int(input_cloud.points[ind].b);
                    rgb_data[3*ind+1] = int(input_cloud.points[ind].g);
                    rgb_data[3*ind+2] = int(input_cloud.points[ind].r);
                    depth_data[ind]   = (unsigned short)(5000*input_cloud.points[ind].z);
                }
            }

            char buf[1024];

            sprintf(buf,"%s/RGB%.10i.png",path.c_str(),counter);
            if(!cvSaveImage(buf,rgb_img)){printf("Could not save: %s\n",buf);}
            cvReleaseImage( &rgb_img);
            sprintf(buf,"%s/Depth%.10i.png",path.c_str(),counter);
            if(!cvSaveImage(buf,depth_img)){printf("Could not save: %s\n",buf);}
            cvReleaseImage( &depth_img );
            count = 0;
            calc();
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



    void calc()
    {
        for(int i = 25; i <= 95; i+=5)
        {
            printf("----------------------%i-------------------\nadding a new frame\n",i);
            
            //Get paths to image files
            char rgbbuf[512];
            char depthbuf[512];
            sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
            sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);

            //Add frame to map
            m->addFrame(string(rgbbuf) , string(depthbuf));
        }
    
        vector<Matrix4f> poses = m->estimate(); //Estimate poses for the frames using the map object.
        m->savePCD("test.pcd");                 //Saves a downsampled pointcloud with aligned data.
        
        //Print poses
        cout << "Poses:" << endl;
        for(unsigned int i = 0; i < poses.size(); i++)
        {
            cout << poses.at(i) << endl << endl;
        }    
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "aick");

    printf("starting testing software2\n");
    printf("give path to files as input\n");
    string input = argv[1];
    string path = input;

    string bow_path = argv[2];

    Map3D * m = new Map3D();    //Create a standard map object
    m->setVerbose(true);        //Set the map to give text output

    m->loadCalibrationWords(bow_path,"orb", 500);   //set bag of words to orb 500 orb features from bow_path
    m->setFeatureExtractor(new OrbExtractor());     //Use orb features

    int max_points = 300;                           //Number of keypoints used by matcher
    int nr_iter = 10;                               //Number of iterations the matcher will run
    float shrinking = 0.7;                          //The rate of convergence for the matcher
    float bow_threshold = 0.15;                     //Bag of words threshold to avoid investigating bad matches
    float distance_threshold = 0.015;               //Distance threshold to discard bad matches using euclidean information.
    float feature_threshold = 0.15;                 //Feature threshold to discard bad matches using feature information.
    
    m->setMatcher(new BowAICK(max_points, nr_iter,shrinking,bow_threshold,distance_threshold,feature_threshold));//Create a new matcher
    vector< RGBDFrame * > frames;
    
    AICKNode my_node;
    my_node.run();
}