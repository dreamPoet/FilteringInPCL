/*
	PassThrough filter:Cut off values that are either inside or outside a given user range.
*/
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

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

	if (pcl::io::loadPLYFile(argv[1], *cloud) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}

	std::cout << "original point number is: " << cloud->points.size() << std::endl;

#if PRINT_POINTS
	std::cerr << "Cloud before filtering: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
#endif

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(580.0, 900.0);
	pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);

	std::cout << "Filtered point number is: " << cloud_filtered->points.size() << std::endl;
	pcl::io::savePLYFile("passthrough.ply", *cloud_filtered);

#if PRINT_POINTS
	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << std::endl;
#endif

	return (0);
}