#ifndef PIPELINE_H
#define PIPELINE_H

#include <iostream> 
#include <thread>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_cloud.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/rsd.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;


class pipeline{
    public:
        pipeline();
        ~pipeline();

        void display_menu();
        void load_data();

        void start();
        
        void showCloud();
        void showCloud_XYZ ();
        
        void showMatches    (PointCloud<PointXYZRGBA>::Ptr,
                             PointCloud<PointXYZRGBA>::Ptr,
                             Correspondences);
                             
        void showInfo();
		
		PointCloud<PointXYZI>::Ptr 	Harris3D(string);
        PointCloud<Normal>::Ptr 	get_Normals_Harris(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);
        PointCloud<Normal>::Ptr 	get_Normals_ISS(PointCloud<PointXYZRGBA>::Ptr point_cloud);

    private:      

        int    object;
        int    keypoints;
        int    descriptors;

        void    eliminatePlanes();

        void    method_HARRIS_PFH();
        void    method_HARRIS_FPFH();
        void    method_HARRIS_RSD();
        void    method_HARRIS_SHOT();

        void    method_SIFT_PFH();
        void    method_SIFT_FPFH();
        void    method_SIFT_RSD();
        void    method_SIFT_SHOT();
        
        double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

        void    method_ISS_PFH();
        void    method_ISS_FPFH();
        void    method_ISS_RSD();
        void    method_ISS_SHOT();

        void    method_US_PFH();
        void    method_US_FPFH();
        void    method_US_SHOT();
        

        PointCloud<PointXYZRGBA>::Ptr scene;
        PointCloud<PointXYZRGBA>::Ptr obj;
        
        PointCloud<PointXYZ>::Ptr scene_harris;
        PointCloud<PointXYZ>::Ptr obj_harris;

        float                          porcentaje_planos;
        float                          threshold;
        double                         puntos_restantes;
        chrono::duration<float,milli>  duration_proces; 
        chrono::duration<float,milli>  duration_elim_planes;
        int                            num_keypoints_scene;
        int                            num_keypoints_obj;
        int                            num_descriptors_scene;
        int                            num_descriptors_obj;
        int                            num_emparejamientos;
        int                            num_emparejamientos_correctos;
        double                         accuracy;

        Eigen::Matrix4f trans;
        Eigen::Matrix4f bestTrans;
};




#endif 
