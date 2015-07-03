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

  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass_in;
  	pass_in.setInputCloud (cloud_in);
  	pass_in.setFilterFieldName ("z");
  	pass_in.setFilterLimits (0.5, 4.0);
	pass_in.filter (*cloud_in_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass_out;
  	pass_out.setInputCloud (cloud_out);
  	pass_out.setFilterFieldName ("z");
  	pass_out.setFilterLimits (0.5, 4.0);
	pass_out.filter (*cloud_out_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_voxel (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (cloud_in_filtered);
  	sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	sor.filter (*cloud_in_voxel);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_voxel (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor_out;
  	sor_out.setInputCloud (cloud_out_filtered);
  	sor_out.setLeafSize (0.01f, 0.01f, 0.01f);
  	sor_out.filter (*cloud_out_voxel);

	struct timeval start, end;
	gettimeofday(&start, NULL);
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(cloud_in_voxel);
	icp.setInputTarget(cloud_out_voxel);
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
	ofstream myfile1;
	myfile1.open ("times.txt", std::ios_base::app);  		
	myfile1 << time << endl;
	myfile1.close();

	transformation->time = time;
	return transformation;
}
