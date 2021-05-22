#include "functions.h"

using namespace std;
using namespace pcl;



int main(int argc, char** argv){

	boost::thread visualizer;
	string pcdFile,object1,object2,object3,object4;

	//Direcciones de la escena y los objetos
	pcdFile = "../nubes/scenes/snap_0point.pcd";	
	object1 = "../nubes/objects/s0_mug_corr.pcd";
	object2 = "../nubes/objects/s0_piggybank_corr.pcd";
	object3 = "../nubes/objects/s0_plant_corr.pcd";
	object4 = "../nubes/objects/s0_plc_corr.pcd";

	//Carga de la escena y de los objetos
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr originalCloud	   (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud         (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object1_pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object2_pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object3_pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object4_pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	if((pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile, *originalCloud)      == -1) or
	   (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile, *pointCloud)         == -1) or
	   (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (object1, *object1_pointCloud) == -1) or
	   (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (object2, *object2_pointCloud) == -1) or
	   (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (object3, *object3_pointCloud) == -1) or
	   (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (object4, *object4_pointCloud) == -1))	
	{
		cerr << "Error. Could not load the PCD File" <<endl;
		return -1;
	}


	/*********************************************/

	// Eliminación plano dominante de la escena
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aux(new pcl::PointCloud<pcl::PointXYZRGBA>);
	eliminatePlanes(pointCloud,planes,aux);



	// Extraccion de los KeyPoints
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_scene  (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_object1(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_object2(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_object3(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_object4(new pcl::PointCloud<pcl::PointXYZI>());
	harrisMethod(keypoints_scene,   pointCloud,         keypoints_object1, object1_pointCloud, 
				 keypoints_object2, object2_pointCloud, keypoints_object3, object3_pointCloud, 
				 keypoints_object4, object4_pointCloud);


	// Extraccion de los descriptores
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals;	scene_normals   = get_Normals(keypoints_scene);
	pcl::PointCloud<pcl::Normal>::Ptr object1_normals;  object1_normals = get_Normals(keypoints_object1);
	pcl::PointCloud<pcl::Normal>::Ptr object2_normals;  object2_normals = get_Normals(keypoints_object2);
	pcl::PointCloud<pcl::Normal>::Ptr object3_normals;  object3_normals = get_Normals(keypoints_object3);
	pcl::PointCloud<pcl::Normal>::Ptr object4_normals;  object4_normals = get_Normals(keypoints_object4);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr object1_descriptors;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr object2_descriptors;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr object3_descriptors;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr object4_descriptors;

	kdTreeMethod(scene_descriptors,   keypoints_scene,   scene_normals,
				 object1_descriptors, keypoints_object1, object1_normals,
				 object2_descriptors, keypoints_object2, object2_normals,
				 object3_descriptors, keypoints_object3, object3_normals,
				 object4_descriptors, keypoints_object4, object4_normals);



	// Emparejamientos y Descarte de emparejamientos incorrectos
	pcl::CorrespondencesPtr matches_corrected_obj1 (new pcl::Correspondences ());	Eigen::Matrix4f trans1;
	pcl::CorrespondencesPtr matches_corrected_obj2 (new pcl::Correspondences ());	Eigen::Matrix4f trans2;
	pcl::CorrespondencesPtr matches_corrected_obj3 (new pcl::Correspondences ());	Eigen::Matrix4f trans3;
	pcl::CorrespondencesPtr matches_corrected_obj4 (new pcl::Correspondences ());	Eigen::Matrix4f trans4;
	
	matches_corrected_obj1 = match_Keypoints(scene_descriptors, object1_descriptors, keypoints_scene, keypoints_object1, trans1, 1);	
	//matches_corrected_obj2 = match_Keypoints(scene_descriptors, object2_descriptors, keypoints_scene, keypoints_object2, trans2, 2);
	//matches_corrected_obj2 = match_Keypoints(scene_descriptors, object3_descriptors, keypoints_scene, keypoints_object3, trans3, 3);
	//matches_corrected_obj2 = match_Keypoints(scene_descriptors, object4_descriptors, keypoints_scene, keypoints_object4, trans4, 4);


	// Refinamiento del resultado con ICP
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr auxCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	Eigen::Matrix4f bestTrans;

	refinar(icp, pointCloud, object1_pointCloud, auxCloud, bestTrans);
	//refinar(icp, pointCloud, object2_pointCloud, auxCloud, bestTrans);
	//refinar(icp, pointCloud, object3_pointCloud, auxCloud, bestTrans);
	//refinar(icp, pointCloud, object4_pointCloud, auxCloud, bestTrans);




	showCloud(originalCloud,auxCloud);


	return 0;
}
