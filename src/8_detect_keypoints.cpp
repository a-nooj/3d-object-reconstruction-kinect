#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

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

	float min_scale = 0.002; 
	int nr_octaves = 4;
	int nr_scales_per_octave = 5; 
	float min_contrast = 1;
	float radius = 0.01;

	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
	//sift_detect.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	sift_detect.setSearchMethod (tree);
	sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast (min_contrast);
	sift_detect.setSearchSurface (cloud);
	sift_detect.setInputCloud (cloud);
	sift_detect.setRadiusSearch (radius);
	pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
	sift_detect.compute (keypoints_temp);
	pcl::copyPointCloud (keypoints_temp, *cloud_with_keypoints);

	std::string delimiter = ".pcd";
	std::string outfile = "keypoints" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));
	
	pcl::io::savePCDFileASCII (outfile, *cloud_with_keypoints);
	std::cerr << "Saved " 
		  << cloud_with_keypoints->width * cloud_with_keypoints->height 
		  << " data points to " << outfile << "." 
		  << std::endl;

	return (0);
}
