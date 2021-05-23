#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <chrono>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>

#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>		
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;
using namespace io;

void 	showCloud			(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);
void 	showCloud 			(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, 
							 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

//Elimination of dominant planes of the scene
void 	eliminatePlanes		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, 
							 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

//Busqueda de Keypoints
void 	harrisMethodAll		(pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

void 	siftMethodAll		(pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

void 	ISSMethodAll		(pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

void 	SUSANMethodAll		(pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

void 	USMethodAll			(pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr&, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);



pcl::PointCloud<pcl::PointXYZI>::Ptr Harris3D 		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, int);
pcl::PointCloud<pcl::PointXYZI>::Ptr Sift3D			(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, int);
pcl::PointCloud<pcl::PointXYZI>::Ptr ISS			(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, int);
pcl::PointCloud<pcl::PointXYZI>::Ptr SUSAN			(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, int);
pcl::PointCloud<pcl::PointXYZI>::Ptr USampling		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, int);


double 		computeCloudResolution					(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);


//Descriptores
void 	kdTreeMethod		(pcl::PointCloud<pcl::PFHSignature125>::Ptr&,pcl::PointCloud<pcl::PointXYZI>::Ptr&,pcl::PointCloud<pcl::Normal>::Ptr&,
							 pcl::PointCloud<pcl::PFHSignature125>::Ptr&,pcl::PointCloud<pcl::PointXYZI>::Ptr&,pcl::PointCloud<pcl::Normal>::Ptr&,
							 pcl::PointCloud<pcl::PFHSignature125>::Ptr&,pcl::PointCloud<pcl::PointXYZI>::Ptr&,pcl::PointCloud<pcl::Normal>::Ptr&,
							 pcl::PointCloud<pcl::PFHSignature125>::Ptr&,pcl::PointCloud<pcl::PointXYZI>::Ptr&,pcl::PointCloud<pcl::Normal>::Ptr&,
							 pcl::PointCloud<pcl::PFHSignature125>::Ptr&,pcl::PointCloud<pcl::PointXYZI>::Ptr&,pcl::PointCloud<pcl::Normal>::Ptr&);
pcl::PointCloud<pcl::Normal>::Ptr 	 		get_Normals	(pcl::PointCloud<pcl::PointXYZI>::Ptr);

pcl::PointCloud<pcl::PFHSignature125>::Ptr 	PFH 		(pcl::PointCloud<pcl::PointXYZI>::Ptr, 
														 pcl::PointCloud<pcl::Normal>::Ptr);



//Emparejamientos
pcl::CorrespondencesPtr match_Keypoints(pcl::PointCloud<pcl::PFHSignature125>::Ptr, 
										pcl::PointCloud<pcl::PFHSignature125>::Ptr,
										pcl::PointCloud<pcl::PointXYZI>::Ptr, 
										pcl::PointCloud<pcl::PointXYZI>::Ptr,
										Eigen::Matrix4f&, int);


//Refinamiento ICP
void 	refinar		(pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>,
	                 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
					 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&, Eigen::Matrix4f&);

#endif


