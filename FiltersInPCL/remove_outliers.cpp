#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

//#define PRINT_POINTS

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		exit(0);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	if (pcl::io::loadPLYFile(argv[1], *cloud) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}

	if (strcmp(argv[2], "-r") == 0) {
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// build the filter
		outrem.setInputCloud(cloud);
		outrem.setRadiusSearch(50.0);
		outrem.setMinNeighborsInRadius(50);
		// apply filter
		outrem.filter(*cloud_filtered);
	}
	else if (strcmp(argv[2], "-c") == 0) {
		// build the condition
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
			pcl::ConditionAnd<pcl::PointXYZ>());
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 600.0)));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 900.0)));
		// build the filter
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud);
		condrem.setKeepOrganized(true);
		// apply filter
		condrem.filter(*cloud_filtered);
	}
	else {
		std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		exit(0);
	}
	pcl::io::savePLYFile("remove_outliers.ply", *cloud_filtered);

#ifdef PRINT_POINTS
	std::cerr << "Cloud before filtering: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
	// display pointcloud after filtering

	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << std::endl;
#endif
	return (0);
}