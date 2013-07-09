#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	if (argc != 2) {
                PCL_ERROR ("Syntax: %s input.pcd\n", argv[0]);
                return -1;
        }

        pcl::io::loadPCDFile (argv[1], *cloud);
	std::string inputfile = argv[1];

	std::cout << "Loaded "
		  << cloud->width * cloud->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	float radius = 0.05;

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
	normal_estimation.setRadiusSearch (radius);
	normal_estimation.setInputCloud (cloud);
	normal_estimation.compute (*normals);

	//pcl::copyPointCloud(*cloud, *cloud_with_normals); 
	//pcl::copyPointCloud(*normals, *cloud_with_normals); 

	std::string delimiter = ".pcd";
	std::string outfile = "normals" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));
	
	pcl::io::savePCDFileASCII (outfile, *normals);
	std::cerr << "Saved " 
		  << normals->width * normals->height 
		  << " data points to " << outfile << "." 
		  << std::endl;

	return (0);
}
