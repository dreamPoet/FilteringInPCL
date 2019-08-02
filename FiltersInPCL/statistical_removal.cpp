#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		printf("Usage : FiltersInPCL.exe pointcloud.ply\n");
		PCL_ERROR("Provide one ply file.\n");
		return (-1);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	if (pcl::io::loadPLYFile(argv[1], *cloud) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(100);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::io::savePLYFile("statistical_removal_inliers.ply", *cloud_filtered);

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	pcl::io::savePLYFile("statistical_removal_outliers.ply", *cloud_filtered);

	return (0);
}