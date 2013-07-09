#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

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

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointNormal> mls_points;

	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

	mls.setComputeNormals (true);

	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	mls.process (mls_points);

	std::string delimiter = ".pcd";
	std::string outfile = "smoothedCloud" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));
	
	pcl::io::savePCDFileASCII (outfile, mls_points);
	std::cerr << "Saved " 
		  << mls_points.width * mls_points.height 
		  << " data points to " << outfile << "." 
		  << std::endl;
}
