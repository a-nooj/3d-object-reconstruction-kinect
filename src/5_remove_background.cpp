#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

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

	float distance_threshold = 0.02;
	float max_iterations = 100;

	//find the dominant plane
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients (false);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distance_threshold);
	seg.setMaxIterations (max_iterations);
	seg.setInputCloud (cloud);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	seg.segment (*inliers, *coefficients);

	// Extract the inliers
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud_filtered);

	std::string delimiter = ".pcd";
	std::string outfile = "sansBackground" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));
	
	pcl::io::savePCDFileASCII (outfile, *cloud_filtered);
	std::cerr << "Saved " 
		  << cloud_filtered->width * cloud_filtered->height 
		  << " data points to " << outfile << "." 
		  << std::endl;

	return (0);
}
