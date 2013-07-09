//take as input final point cloud and add surface to it, save to final_reconstruction.vtk

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	if (argc != 3) {
		PCL_ERROR ("Syntax: %s input.pcd output.vtk\n", argv[0]);
		return -1;
	}

	pcl::io::loadPCDFile (argv[1], *cloud);

        std::cout << "Loaded "
                  << cloud->width * cloud->height
                  << " data points from input_cloud.pcd."
                  << std::endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>  n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (40);
	n.compute (*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	gp3.setSearchRadius (0.25);
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (200);
	gp3.setMaximumSurfaceAngle (M_PI/4);
	gp3.setMinimumAngle (M_PI/18);
	gp3.setMaximumAngle (2*M_PI/3);
	gp3.setNormalConsistency (false);
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	pcl::io::saveVTKFile (argv[2], triangles);

	return (0);
}
