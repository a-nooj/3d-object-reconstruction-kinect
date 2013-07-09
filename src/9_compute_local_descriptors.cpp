#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
	const pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (argc != 4) {
                PCL_ERROR ("Syntax: %s input.pcd normals.pcd keypoints.pcd\n", argv[0]);
                return -1;
        }

        pcl::io::loadPCDFile (argv[1], *points);
	std::string inputfile = argv[1];

	std::cout << "Loaded "
		  << points->width * points->height
		  << " data points from " << inputfile << "."
		  << std::endl;

        pcl::io::loadPCDFile (argv[2], *normals);
	inputfile = argv[2];

	std::cout << "Loaded "
		  << normals->width * normals->height
		  << " data points from " << inputfile << "."
		  << std::endl;

        pcl::io::loadPCDFile (argv[3], *keypoints);
	inputfile = argv[3];

	std::cout << "Loaded "
		  << keypoints->width * keypoints->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	float feature_radius = 0.1;

	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	fpfh_estimation.setSearchMethod (tree);
	fpfh_estimation.setRadiusSearch (feature_radius);
	fpfh_estimation.setSearchSurface (points);  
	fpfh_estimation.setInputNormals (normals);
	fpfh_estimation.setInputCloud (keypoints);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptors (new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfh_estimation.compute (*local_descriptors);

	std::string delimiter = ".pcd";
	std::string outfile = "localDescriptors" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));
	
	pcl::io::savePCDFileASCII (outfile, *local_descriptors);
	std::cerr << "Saved " 
		  << local_descriptors->width * local_descriptors->height 
		  << " data points to " << outfile << "." 
		  << std::endl;

	return (0);
}
