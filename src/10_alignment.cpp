#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

int main (int argc, char ** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_descriptors (new pcl::PointCloud<pcl::FPFHSignature33>);

	if (argc != 7) {
                PCL_ERROR ("Syntax: %s source.pcd sourceKeypoints.pcd sourceDescriptors.pcd target.pcd targetKeypoints.pcd targetDescriptors.pcd\n", argv[0]);
                return -1;
        }

        pcl::io::loadPCDFile (argv[1], *source);
	std::string inputfile = argv[1];

	std::cout << "Loaded "
		  << source->width * source->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	pcl::io::loadPCDFile (argv[2], *source_keypoints);
	inputfile = argv[2];

	std::cout << "Loaded "
		  << source_keypoints->width * source_keypoints->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	pcl::io::loadPCDFile (argv[3], *source_descriptors);
	inputfile = argv[3];

	std::cout << "Loaded "
		  << source_descriptors->width * source_descriptors->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	pcl::io::loadPCDFile (argv[4], *target);
	inputfile = argv[4];

	std::cout << "Loaded "
		  << target->width * target->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	pcl::io::loadPCDFile (argv[5], *target_keypoints);
	inputfile = argv[5];

	std::cout << "Loaded "
		  << target_keypoints->width * target_keypoints->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	pcl::io::loadPCDFile (argv[6], *target_descriptors);
	inputfile = argv[6];

	std::cout << "Loaded "
		  << target_descriptors->width * target_descriptors->height
		  << " data points from " << inputfile << "."
		  << std::endl;

	Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity ();
	Eigen::Matrix4f final_alignment = Eigen::Matrix4f::Identity ();

	float min_sample_distance = 0.025;
	float max_correspondence_distance = 0.01;
	int nr_iterations = 500;

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
	sac_ia.setMinSampleDistance (min_sample_distance);
	sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
	sac_ia.setMaximumIterations (nr_iterations);

	sac_ia.setInputCloud (source_keypoints);
	sac_ia.setSourceFeatures (source_descriptors);

	sac_ia.setInputTarget (target_keypoints);
	sac_ia.setTargetFeatures (target_descriptors);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_source (new pcl::PointCloud<pcl::PointXYZRGB>);
	sac_ia.align (*aligned_source);

	initial_alignment = sac_ia.getFinalTransformation();

	pcl::console::print_info ("Computed initial alignment\n");

	float max_correspondence_distance_refine = 0.05;
	float outlier_rejection_threshold = 0.05;
	float transformation_epsilon = 1e-6;
	int max_iterations = 1000;

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaxCorrespondenceDistance (max_correspondence_distance);
	icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
	icp.setTransformationEpsilon (transformation_epsilon);
	icp.setMaximumIterations (max_iterations);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_points_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::transformPointCloud (*source, *source_points_transformed, initial_alignment);

	icp.setInputCloud (aligned_source);
	icp.setInputTarget (target);

	pcl::PointCloud<pcl::PointXYZRGB> registration_output;
	icp.align (registration_output);

	final_alignment = icp.getFinalTransformation () * initial_alignment;

	pcl::console::print_info ("Refined alignment\n");


	// Transform the source point to align them with the target points
	pcl::transformPointCloud (*source, *source, final_alignment);

	// Save output
	//std::string filename;


	// Merge the two clouds
	(*source) += (*target);

	std::string delimiter = ".pcd";
	std::string outfile = "finalAlignedCloud" + inputfile.substr(inputfile.find(delimiter) - 1, inputfile.find('\0'));
	
	pcl::io::savePCDFileASCII (outfile, *source);
	std::cerr << "Saved " 
		  << source->width * source->height 
		  << " data points to " << outfile << "." 
		  << std::endl;

	return (0);
}
