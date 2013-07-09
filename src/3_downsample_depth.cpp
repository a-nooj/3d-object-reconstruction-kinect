#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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

        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	std::string delimiter = ".pcd";
        std::string outfile = "downsampledCloud" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));


        pcl::io::savePCDFileASCII (outfile, *cloud_filtered);
        std::cerr << "Saved "
                  << cloud_filtered->width * cloud_filtered->height
                  << " data points to " << outfile << "."
                  << std::endl;

	return (0);
}
