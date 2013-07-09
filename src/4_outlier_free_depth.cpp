#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

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

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (100);
	sor.setStddevMulThresh(1.0);
	sor.filter (*cloud_filtered);

	std::string delimiter = ".pcd";
	std::string outfile_inliers = "inliersCloud" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));

	pcl::io::savePCDFileASCII (outfile_inliers, *cloud_filtered);
        std::cerr << "Saved "
                  << cloud_filtered->width * cloud_filtered->height
                  << " data points to " << outfile_inliers << "."
                  << std::endl;

	return (0);
}
