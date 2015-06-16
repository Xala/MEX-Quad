#include "ICP.h"
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;
const bool debugg_ICP = false;

ICP::ICP()
{
	name = "ICP";
}

ICP::~ICP(){printf("delete ICP\n");}

Transformation * ICP::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_in = src->input->getCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_out = dst->input->getCloud();

	struct timeval start, end;
	gettimeofday(&start, NULL);
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = icp.getFinalTransformation();
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 100;

	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("ICP cost: %f\n",time);
	transformation->time = time;
	return transformation;
}
