#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>

typedef pcl::FPFHSignature33 Signature; 
typedef pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, Signature> Estimation; 
typedef pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, Signature> Alignment; 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; 
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals; 
typedef pcl::PointCloud<Signature> LocalFeatures; 
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod; 
typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> Result;

void getScaledPointCloud(const std::string &pcdFile, PointCloud::Ptr original, PointCloud::Ptr rescaled,IndicesPtr indices){ 
     pcl::io::loadPCDFile (pcdFile, *original); 

     //downsampling 
     float gridSize=0.003; 
     pcl::VoxelGrid<pcl::PointXYZ> vox; 
     vox.setInputCloud (original); 
     vox.setLeafSize (gridSize, gridSize, gridSize); 
     vox.filter (*rescaled); 
} 

void computeSurfaceNormals (PointCloud::Ptr in, SurfaceNormals::Ptr out){ 
     SearchMethod::Ptr method(new SearchMethod); 

     pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> est; 
     est.setInputCloud (in); 
     est.setNumberOfThreads(4); 
     est.setSearchMethod (method); 
     est.setRadiusSearch(0.02); 
     est.compute (*out); 
} 

void computeLocalFeatures (PointCloud::Ptr in, SurfaceNormals::Ptr normals, LocalFeatures::Ptr out) { 
     SearchMethod::Ptr method(new SearchMethod); 

     Estimation est; 
     est.setInputCloud (in); 
     est.setInputNormals (normals); 
     est.setSearchMethod (method); 
     est.setRadiusSearch(0.04); //IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!! 
     est.compute (*out); 
} 

void doAlign(PointCloud::Ptr modelCloud, LocalFeatures::Ptr modelFeatures, PointCloud::Ptr imageCloud, LocalFeatures::Ptr imageFeatures,Result* guess,Result& result){ 

     Alignment sac; 

     sac.setMinSampleDistance (0.05); 
     sac.setMaxCorrespondenceDistance (0.01*0.01); 
     sac.setMaximumIterations (500); 

     sac.setInputCloud(modelCloud); 
     sac.setSourceFeatures(modelFeatures); 

     sac.setInputTarget(imageCloud); 
     sac.setTargetFeatures(imageFeatures); 

     pcl::PointCloud<pcl::PointXYZ> registration_output; 
     if(!guess)
 sac.align (registration_output); 
     else
 sac.align(registration_output,guess->transformation); 

     result.score = (float) sac.getFitnessScore (100); 
     result.transformation =  sac.getFinalTransformation (); 
} 

int main(int argc, char** argv) { 

     PointCloud::Ptr modelCloud = PointCloud::Ptr (new PointCloud); 
     PointCloud::Ptr modelRescaledCloud = PointCloud::Ptr (new PointCloud); 
     PointCloud::Ptr modelConvexCloud = PointCloud::Ptr (new PointCloud); 
     PointCloud::Ptr imageCloud = PointCloud::Ptr (new PointCloud); 
     PointCloud::Ptr imageRescaledCloud = PointCloud::Ptr (new PointCloud); 
     PointCloud::Ptr imageConvexCloud = PointCloud::Ptr (new PointCloud); 

     //getting cloud, image and downsampled cloud 
     getScaledPointCloud(argv[1],modelCloud, modelRescaledCloud); 
     getScaledPointCloud(argv[2],imageCloud, imageRescaledCloud); 

     SurfaceNormals::Ptr modelNormals = SurfaceNormals::Ptr (new SurfaceNormals); 
     LocalFeatures::Ptr modelFeatures = LocalFeatures::Ptr (new LocalFeatures); 
     computeSurfaceNormals(modelRescaledCloud,modelNormals); 
     computeLocalFeatures(modelRescaledCloud,modelNormals,modelFeatures); 

     std::cout << "Calculating frature points for image"<<std::endl; 
     SurfaceNormals::Ptr imageNormals = SurfaceNormals::Ptr (new SurfaceNormals); 
     LocalFeatures::Ptr imageFeatures = LocalFeatures::Ptr (new LocalFeatures); 
     computeSurfaceNormals(imageRescaledCloud,imageNormals); 
     computeLocalFeatures(imageRescaledCloud,imageNormals,imageFeatures); 

     Result esimation,convex,final; 

     //registration 
    
  doAlign(modelRescaledCloud,modelFeatures,imageRescaledCloud,imageFeatures,NULL,esimation); 
     esimation.dump(); 

     pcl::PointCloud<pcl::PointXYZ>::Ptr transformedEstimation (new pcl::PointCloud<pcl::PointXYZ>); 
     pcl::transformPointCloud (*modelCloud, *transformedEstimation, esimation.transformation); 
     pcl::io::savePCDFileBinary ("estimation.pcd", *transformedEstimation); 

     //convex points extraction 
     pcl::ConvexHull<pcl::PointXYZ> chull1; 
     chull1.setInputCloud (modelCloud); 
     chull1.reconstruct (*modelConvexCloud); 

     pcl::ConvexHull<pcl::PointXYZ> chull2; 
     chull2.setInputCloud (imageCloud); 
     chull2.reconstruct (*imageConvexCloud); 

     //ICP on convex points 
     doICP(model, modelConvexCloud, input ,imageConvexCloud,esimation,convex); 
     convex.dump(); 

     pcl::PointCloud<pcl::PointXYZ>::Ptr transformedConvex (new pcl::PointCloud<pcl::PointXYZ>); 
     pcl::transformPointCloud (*modelCloud, *transformedConvex, convex.transformation); 
     pcl::io::savePCDFileBinary ("convex.pcd", *transformedConvex); 

     //ICP on all points 
     doICP(model, modelRescaledCloud, input ,imageRescaledCloud,convex,final); 
     final.dump(); 

     pcl::PointCloud<pcl::PointXYZ>::Ptr transformedICP (new pcl::PointCloud<pcl::PointXYZ>); 
     pcl::transformPointCloud (*modelCloud, *transformedICP, final.transformation); 
     pcl::io::savePCDFileBinary ("icp.pcd", *transformedICP); 
} 
