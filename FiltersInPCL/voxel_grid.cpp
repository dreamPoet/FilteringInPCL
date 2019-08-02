#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		printf("Usage : FiltersInPCL.exe pointcloud.ply\n");
		PCL_ERROR("Provide one ply file.\n");
		return (-1);
	}

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// Fill in the cloud data
	if (pcl::io::loadPLYFile(argv[1], *cloud) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}
	std::cerr << "PointCloud before filtering: " 
		<< cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").\n";

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(10.0f, 10.0f, 10.0f);//leaf size is 10m
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " 
		<< cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").\n";

	pcl::io::savePLYFile("voxel_grid.ply", *cloud_filtered);

	return (0);
}