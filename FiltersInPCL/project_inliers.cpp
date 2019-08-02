#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		printf("Usage : FiltersInPCL.exe pointcloud.ply\n");
		PCL_ERROR("Provide one ply file.\n");
		return (-1);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	if (pcl::io::loadPLYFile(argv[1], *cloud) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}
	std::cout << "original point number is: " << cloud->points.size() << std::endl;

#if PRINT_POINTS
	std::cerr << "Cloud before projection: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
#endif

	// Create a set of planar coefficients with a=b=d=0,c=1 in other words, xy plane.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	std::cout << "Projected point number is: " << cloud_projected->points.size() << std::endl;
	pcl::io::savePLYFile("projected.ply", *cloud_projected);

#if PRINT_POINTS
	std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << std::endl;
#endif

	return (0);
}