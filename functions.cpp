#include "functions.h"

void showCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){
	
	pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    
    viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "id0");
    
    while(!viewer.wasStopped ()) {
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void showCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& scene, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object){

	pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
	
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> custom(object,255,0,0);
    
    viewer.addPointCloud<pcl::PointXYZRGBA> (scene, rgb, "id0");
    viewer.addPointCloud<pcl::PointXYZRGBA> (object, custom, "id1");
    
    while(!viewer.wasStopped ()) {
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}



/*		Eliminación de planos dominantes de la escena

		Se van a hacer diferentes pruebas de diferentes metodos. 
		En primer luegar se va a emplear el modulo SEGMENTATION -> SACSEGMENTATION. 
		Esta clase deriva de PCLBase, de la que también se va a utilizar algunos metodos.

		Metodo 1:
			- Primero se crea el objeto de la clase SACSegmentation
			- Se le pasa a dicho objeto la nube de puntos
			- Luego se seleccionan el modelo, metodo, numero máximo de iteraciones, 
			distancia umbral al modelo
			- Inicialización de los punteros a los coeficientes e inliers y segmentación y 
			checkeo de que se ha encontrado algun plano (de lo establecido en los parametros)
			en la nube de puntos cargada
			- Creación de objeto de filtrado
			- Extracción de los inliers
			- Setting de los indices a partir de los inliers calculados de pointCloud
			- setNegative=falso devuelve el plano que se va a retirar
			- setNegative=true  devuelve el plano que queda del pointCloud
			- Se hace un swap para guardar la nueva nube de puntos en la variable pointCloud


*/
void eliminatePlanes(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloud, 
					 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& planes,
					 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& aux){

	auto start = std::chrono::system_clock::now();

	size_t pts = pointCloud->points.size();	
	while(pointCloud->points.size() > pts*0.4){

		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		seg.setInputCloud(pointCloud);

		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setMaxIterations(500);

		
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers 			 (new pcl::PointIndices);
		seg.segment(*inliers, *coefficients);
		if(inliers->indices.size() == 0)
			cerr << "Could not estimate a planar model" << endl;
			

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(pointCloud);
		extract.setIndices(inliers);

		extract.setNegative(false);
		extract.filter(*planes);

		extract.setNegative(true);
		extract.filter(*aux);

		pointCloud->swap(*aux);
	}

	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start; 

	cout << "Planos eliminados. Nuevo tamaño nube de puntos: ";
	cout << pointCloud->points.size() << endl;
	cout << "Coste de procesamiento: ";
	cout << duration.count() << " ms" << endl;

}


/*		BUSQUEDA DE KEYPOINTS

		Metodo 1: Harris 3D

*/
void 	harrisMethodAll	(pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_scene,   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object1_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object2_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object3_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object4, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object4_pointCloud)
{
	
	keypoints_scene		= Harris3D(pointCloud, 0);			
	keypoints_object1 	= Harris3D(object1_pointCloud, 1);
	keypoints_object2 	= Harris3D(object2_pointCloud, 2);	
	keypoints_object3 	= Harris3D(object3_pointCloud, 3);	
	keypoints_object4 	= Harris3D(object4_pointCloud, 4);	
	
}

void 	siftMethodAll	(pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_scene,   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object1_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object2_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object3_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object4, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object4_pointCloud)
{

	keypoints_scene		= Sift3D(pointCloud, 0);			
	keypoints_object1 	= Sift3D(object1_pointCloud, 1);
	keypoints_object2 	= Sift3D(object2_pointCloud, 2);	
	keypoints_object3 	= Sift3D(object3_pointCloud, 3);	
	keypoints_object4 	= Sift3D(object4_pointCloud, 4);	
}

void 	ISSMethodAll	(pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_scene,   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object1_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object2_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object3_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object4, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object4_pointCloud)
{
	keypoints_scene		= ISS(pointCloud, 0);			
	keypoints_object1 	= ISS(object1_pointCloud, 1);
	keypoints_object2 	= ISS(object2_pointCloud, 2);	
	keypoints_object3 	= ISS(object3_pointCloud, 3);	
	keypoints_object4 	= ISS(object4_pointCloud, 4);
}

void 	SUSANMethodAll	(pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_scene,   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object1_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object2_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object3_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object4, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object4_pointCloud)
{
	keypoints_scene		= SUSAN(pointCloud, 0);			
	keypoints_object1 	= SUSAN(object1_pointCloud, 1);
	keypoints_object2 	= SUSAN(object2_pointCloud, 2);	
	keypoints_object3 	= SUSAN(object3_pointCloud, 3);	
	keypoints_object4 	= SUSAN(object4_pointCloud, 4);
}

void 	USMethodAll		(pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_scene,   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object1_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object2_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object3_pointCloud,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object4, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& object4_pointCloud)
{
	keypoints_scene		= USampling(pointCloud, 0);			
	keypoints_object1 	= USampling(object1_pointCloud, 1);
	keypoints_object2 	= USampling(object2_pointCloud, 2);	
	keypoints_object3 	= USampling(object3_pointCloud, 3);	
	keypoints_object4 	= USampling(object4_pointCloud, 4);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Harris3D(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, int c){
	
	auto start = std::chrono::system_clock::now();

	pcl::HarrisKeypoint3D<pcl::PointXYZRGBA,pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	
	detector.setInputCloud(point_cloud);	 
	detector.setNonMaxSupression(true);
	detector.setRadius(0.01);
	detector.compute(*keypoints);

	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;
	if(c)
		cout << endl << "KeyPoints objeto " << c << " calculados " << keypoints->size() << endl;
	else
		cout << endl << "KeyPoints escena calculados " << keypoints->size() << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;
  

	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Sift3D(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, int c){
	const float min_scale = 0.01f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 10;
	const float min_contrast = 0.3f;
	
	auto start = std::chrono::system_clock::now();

	pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	
	detector.setInputCloud(point_cloud);	
	detector.setMinimumContrast(0.3);
	detector.setScales(min_scale, n_octaves, n_scales_per_octave);
	detector.setMinimumContrast(min_contrast);	 
	detector.compute(*keypoints); 
	
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;
	if(c)
		cout << endl << "KeyPoints objeto " << c << " calculados " << keypoints->size() << endl;
	else
		cout << endl << "KeyPoints escena calculados " << keypoints->size() << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;

  
	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ISS(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, int c){
	
	ISSKeypoint3D<pcl::PointXYZRGBA,pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new search::KdTree<pcl::PointXYZRGBA>);
    
    double resolution = computeCloudResolution(point_cloud);
    
    auto start = std::chrono::system_clock::now();

    detector.setInputCloud(point_cloud);
    detector.setSearchMethod(kdtree);
    detector.setSalientRadius(6 * resolution);
    detector.setNonMaxRadius(4 * resolution);
    detector.setMinNeighbors(5);
    detector.setThreshold21(0.975);
    detector.setThreshold32(0.975);
    detector.compute(*keypoints);


    auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;
	if(c)
		cout << endl << "KeyPoints objeto " << c << " calculados " << keypoints->size() << endl;
	else
		cout << endl << "KeyPoints escena calculados " << keypoints->size() << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;

    return keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SUSAN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, int c){

	SUSANKeypoint<pcl::PointXYZRGBA,pcl::PointXYZI>::Ptr detector(new SUSANKeypoint<PointXYZRGBA,PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());

	auto start = std::chrono::system_clock::now();


	detector->setInputCloud(point_cloud);
	detector->setNonMaxSupression(true);
	detector->compute(*keypoints);


	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;
	if(c)
		cout << endl << "KeyPoints objeto " << c << " calculados " << keypoints->size() << endl;
	else
		cout << endl << "KeyPoints escena calculados " << keypoints->size() << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;

	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr USampling (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud, int c){

	UniformSampling<pcl::PointXYZRGB> detector;
    PointCloud<int>::Ptr _k(new PointCloud<int>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());

	auto start = std::chrono::system_clock::now();


	detector.setRadiusSearch(0.05);
    detector.setInputCloud(point_cloud);
    detector.compute(*_k); 

    keypoints->reset(new pcl::PointXYZRGB);
    for (size_t i=0; i<_k->points.size (); ++i)
      keypoints->points.push_back(point_cloud->points[_k->points[i]]);



	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;
	if(c)
		cout << endl << "KeyPoints objeto " << c << " calculados " << keypoints->size() << endl;
	else
		cout << endl << "KeyPoints escena calculados " << keypoints->size() << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;

	return keypoints;
}




double computeCloudResolution(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	vector<int> indices(2);
	vector<float> squaredDistances(2);
	search::KdTree<pcl::PointXYZRGBA> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
	if (! pcl_isfinite((*cloud)[i].x))
	  continue;

	// Considering the second neighbor since the first is the point itself.
	nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
	if (nres == 2)
	{
	  resolution += sqrt(squaredDistances[1]);
	  ++numberOfPoints;
	}
	}
	if (numberOfPoints != 0)
	resolution /= numberOfPoints;

	return resolution;
}

/*		BUSQUEDA DE DESCRIPTORES

		Primero (general) -> Calculo de las normales sobre los keypoints hallados

*/

pcl::PointCloud<pcl::Normal>::Ptr get_Normals(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud){
	
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normals;
	normals.setInputCloud(point_cloud);
	
	
	pcl::search::KdTree<pcl::PointXYZI>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZI> ());
	normals.setSearchMethod (Kdtree);
	
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	normals.setRadiusSearch (0.03);

	normals.compute (*cloud_normals);
		
	return cloud_normals;
}

void kdTreeMethod(pcl::PointCloud<pcl::PFHSignature125>::Ptr& scene_descriptors,  pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_scene,  pcl::PointCloud<pcl::Normal>::Ptr& scene_normals,
				  pcl::PointCloud<pcl::PFHSignature125>::Ptr& object1_descriptors,pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object1,pcl::PointCloud<pcl::Normal>::Ptr& object1_normals,
				  pcl::PointCloud<pcl::PFHSignature125>::Ptr& object2_descriptors,pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object2,pcl::PointCloud<pcl::Normal>::Ptr& object2_normals,
				  pcl::PointCloud<pcl::PFHSignature125>::Ptr& object3_descriptors,pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object3,pcl::PointCloud<pcl::Normal>::Ptr& object3_normals,
				  pcl::PointCloud<pcl::PFHSignature125>::Ptr& object4_descriptors,pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints_object4,pcl::PointCloud<pcl::Normal>::Ptr& object4_normals)
{
	auto start = std::chrono::system_clock::now();

	scene_descriptors   = PFH(keypoints_scene,scene_normals);		  
	object1_descriptors = PFH(keypoints_object1,object1_normals); 
	object2_descriptors = PFH(keypoints_object2,object2_normals); 
	object3_descriptors = PFH(keypoints_object3,object3_normals); 
	object4_descriptors = PFH(keypoints_object4,object4_normals); 

	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;

	cout << endl << "Descriptores calculados" << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
	
	pcl::PFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125> pfh;// dispositivo de estimación de características phf
	pfh.setInputCloud(point_cloud);
	pfh.setInputNormals(normals);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZI>());
	pfh.setSearchMethod(tree2);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_fe_ptr(new pcl::PointCloud<pcl::PFHSignature125>());

	pfh.setRadiusSearch(0.05);

	pfh.compute(*pfh_fe_ptr);
	
	return pfh_fe_ptr;	
}


/* 		EMPAREJAMIENTOS Y ELIMINACION DE EMPAREJAMIENTOS INCORRECTOS

		Metodo -> RANSAC: 
			En esta función se va a encontrar

*/

pcl::CorrespondencesPtr match_Keypoints(pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors, 
										pcl::PointCloud<pcl::PFHSignature125>::Ptr object_descriptors,
										pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keypoints, 
										pcl::PointCloud<pcl::PointXYZI>::Ptr object_keypoints,
										Eigen::Matrix4f& trans, int i)
{
	auto start = std::chrono::system_clock::now();

	pcl::CorrespondencesPtr matches (new pcl::Correspondences ());

	pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	match_search.setInputCloud(object_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size (); ++i){
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!std::isfinite (scene_descriptors->at(i).histogram[0])){
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f){
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			matches->push_back(corr);
		}
	}
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<float,std::milli> duration = end - start;

	cout << endl << "Emparejamientos objeto " << i << " calculados: " << matches->size () << endl;
	cout << "Coste de procesamiento: ";		
	cout << duration.count() << " ms" << endl;

	//////////

	start = std::chrono::system_clock::now();

	pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZI> rej_samp;
	
	rej_samp.setInputTarget(object_keypoints);
	rej_samp.setInputSource(scene_keypoints);
	
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);

	end = std::chrono::system_clock::now();
	duration = end - start;
	
	cout << "Descarte de emparejamientos objeto " << i << " incorrectos" <<endl;
	cout << "Coste de procesamiento: ";	
	cout << duration.count() << " ms" << endl;

	//////////

	if(matches->size()>0){
		start = std::chrono::system_clock::now();	

		trans = rej_samp.getBestTransformation();
		cout << "Matriz transformacion objeto " << i << ": " << endl;
		cout << trans << endl;

		end = std::chrono::system_clock::now();
		duration = end - start;

		cout << "Coste de procesamiento: ";	
		cout << duration.count() << " ms" << endl;
	}
	else
		cout << "No hay matriz de transformacion" << endl;
		
	return matches_corrected;
	
}



/*		REFINAMINETO 

			Metodo: ICP
*/

void refinar	(pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp,
				 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object,
				 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& aux, Eigen::Matrix4f& bestTrans)
{
	auto start = std::chrono::system_clock::now();

	icp.setInputSource(object);
	icp.setInputTarget(scene);	

	icp.setMaxCorrespondenceDistance(100);  
	icp.setTransformationEpsilon(1e-10); 
	icp.setEuclideanFitnessEpsilon(0.001); 
	icp.setMaximumIterations(100); 

	icp.align(*aux);  
	bestTrans = icp.getFinalTransformation();
	
	auto end = std::chrono::system_clock::now();

	std::chrono::duration<float,std::milli> duration = end - start;
	
	cout << endl << "Refinamiento completado. Nueva transformada: " <<endl;
	cout << bestTrans << endl;
	cout << "Coste de procesamiento: ";	
	cout << duration.count() << " ms" << endl;
}
