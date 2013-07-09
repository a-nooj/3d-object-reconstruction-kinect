#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

typedef pcl::FPFHSignature33 Signature; 
typedef pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, Signature> 
Estimation; 
typedef pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, 
pcl::PointXYZ, Signature> Alignment; 
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud; 
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals; 
typedef pcl::PointCloud<Signature> LocalFeatures; 
typedef pcl::search::KdTree<pcl::PointXYZRGBA> SearchMethod;

void doAlign (PointCloud::Ptr modelCloud, LocalFeatures::Ptr modelFeatures, PointCloud::Ptr imageCloud, LocalFeatures::Ptr imageFeatures, Result* guess, Result& result) {
	Alignment sac;
}

int main (int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
	loadData (argc, argv, data);

	if (data.empty()) {
		PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
		PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
		return (-1);
	}

	PCL_INFO ("Loaded %d datasets.", (int)data.size ());

	for (size_t i = 1; i < data.size(); ++i) {
		source = data[i-1].cloud;
		target = data[i].cloud;
	}

//----------------compute surface normals-----------
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud (source);
	ne.setNumberOfThreads (4);

	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	ne.setSearchMethod (tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch (0.03);
	ne.compute (*cloud_normals);
//---------end of computer surface normals--------

//-----------compute local features----------	
	pcl::PointCloud<Signature>::Ptr features (new pcl::PointCloud<Signature> ());
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr searchtree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
        pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, Signature> est;

        est.setInputCloud (source);
        est.setInputNormals (cloud_normals);
        est.setSearchMethod (searchtree);
        est.setRadiusSearch (0.04);
        est.compute (*features);
//-----------end of compute local features---------
	
	Result estimation, convex, final;
	
//--------------alignment-----------
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, Signature> sac;
	sac.setMinSampleDistance (0.05);
        sac.setMaxCorrespondenceDistance (0.01*0.01);
        sac.setMaximumIterations (500);
        sac.setInputCloud(modelCloud);
        sac.setSourceFeatures(modelFeatures);
        sac.setInputTarget(imageCloud);
        sac.setTargetFeatures(imageFeatures);

        pcl::PointCloud<pcl::PointXYZRGBA> registration_output;
        if (!guess)
                sac.align (registration_output);
        else
                sac.align (registration_output.guess->transformation);

        result.score = (float) sac.getFitnessScore (100);
        result.transformation = sac.getFinalTransformation();
//-----------end of alignment-----------

//------------convex points extraction--------
	pcl::ConvexHull<pcl::PointXYZRGBA> chull1;
//----------end of convex points extraction---------

//----------icp on convex points--------
//-----------end of icp on convex points--------

//----------icp on all points-------
//---------end of icp on all points--------


	pcl::registration::TransformationEstimationSVD <pcl::PointXYZRGBA, pcl::PointXYZRGBA> te;
	Eigen::Matrix4f T;

	te.estimateRigidTransformation (*cloud, *cloud2, T);	




	return (0);
}

//identify normals
//cluster at common origin
//find rotation
//apply rotation to one point cloud
//translate one point cloud over other
//use icp for refinement of alignment
