#include "pipeline.h"

pipeline::pipeline(){}
pipeline::~pipeline(){}

void pipeline::display_menu(){
	cout << "Objetos:" << endl << "1-TAZA" << "\t" << "2-HUCHA" << "\t" << "3-PLANTA  " << " 4-PLC" << endl;
    cout << "Opcion: "; cin >> object;
    
    cout << endl << "Metodo para extracción KeyPoints:" << endl;
    cout << "1-HARRIS" <<"\t" << "2-SIFT" << "\t" << "3-ISS" << "\t" << "4-Uniform Sampling" << endl;
    cout << "Opcion: "; cin >> keypoints;

    cout << endl << "Metodo para extraccion Descriptores" << endl;
    cout << "1-PFH" << "\t" << "2-FPFH"  << "\t";
    if(keypoints==4)
        cout << "3-SHOT" << endl;
    else 
        cout << "3-RSD" << "\t" << "4-SHOT" << endl;
    cout << "Opcion: "; cin >> descriptors;
}

void pipeline::load_data(){
    scene = PointCloud<PointXYZRGBA>::Ptr (new PointCloud<PointXYZRGBA>());

    if(io::loadPCDFile<PointXYZRGBA> ("../nubes/scenes/snap_0point.pcd", *scene) == -1)
	{
		cerr << "Error al cargar la nube \n" << endl;
	}
	
	scene_harris = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>());

    if(io::loadPCDFile<PointXYZ> ("../nubes/scenes/snap_0point.pcd", *scene_harris) == -1)
	{
		cerr << "Error al cargar la nube \n" << endl;
	}

    obj = PointCloud<PointXYZRGBA>::Ptr (new PointCloud<PointXYZRGBA>());
    obj_harris = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>());
    string name;
    switch (object){
        case 1:
            name = "../nubes/objects/s0_mug_corr.pcd"; break;
        case 2:
            name = "../nubes/objects/s0_piggybank_corr.pcd"; break;
        case 3:
            name = "../nubes/objects/s0_plant_corr.pcd"; break;
        case 4:
            name = "../nubes/objects/s0_plc_corr.pcd"; break;
        default:
            break;            
    }

    if(io::loadPCDFile<PointXYZRGBA> (name, *obj) == -1)
	{
		cerr << "Te hemos dado del 1 al 4, elige bien melón \n" << endl;
        exit(0);
	}   
	
	if(io::loadPCDFile<PointXYZ> (name, *obj_harris) == -1)
	{
		cerr << "Te hemos dado del 1 al 4, elige bien melón \n" << endl;
        exit(0);
	} 

}

void pipeline::showCloud (){

    pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> custom(obj,255,0,0);
    
    viewer.addPointCloud<pcl::PointXYZRGBA> (scene, rgb, "id0");
    viewer.addPointCloud<pcl::PointXYZRGBA> (obj, custom, "id1");
    
    while(!viewer.wasStopped ()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void pipeline::showCloud_XYZ (){

    pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
    
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb(obj_harris);//custom(obj_harris,255,0,0);
    
    viewer.addPointCloud<pcl::PointXYZRGBA> (scene, "id0");
    viewer.addPointCloud<pcl::PointXYZ> (obj_harris, "id1");
    
    while(!viewer.wasStopped ()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void pipeline::showMatches(PointCloud<PointXYZRGBA>::Ptr scene_cloud_keypoints,
                           PointCloud<PointXYZRGBA>::Ptr obj_cloud_keypoints,
                           Correspondences corr){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene);
    viewer->addPointCloud (scene,rgb, "nube_esc_cloud");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr off_nube_esc_nube_obj (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr off_nube_esc_nube_obj_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());

    //  We are translating the nube_obj so that it doesn't end in the middle of the nube_esc representation
    pcl::transformPointCloud (*obj, *off_nube_esc_nube_obj, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*obj_cloud_keypoints, *off_nube_esc_nube_obj_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> off_nube_esc_nube_obj_color_handler (off_nube_esc_nube_obj, 255, 255, 128);
    viewer->addPointCloud (off_nube_esc_nube_obj, off_nube_esc_nube_obj_color_handler, "off_nube_esc_nube_obj");

    for (size_t j = 0; j < corr.size (); ++j)
    {
        std::stringstream ss_line;
        ss_line << "correspondence_line"  << j << endl;
        pcl::PointXYZRGBA& nube_obj_point = off_nube_esc_nube_obj_keypoints->at (corr.at(j).index_query);
        pcl::PointXYZRGBA& nube_esc_point = scene_cloud_keypoints->at (corr.at(j).index_match);//query

        //  We are drawing a line for each pair of clustered correspondences found between the nube_obj and the nube_esc
        viewer->addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA    > (nube_obj_point, nube_esc_point, 0, 255, 0, ss_line.str ());
    }

    while(!viewer->wasStopped ()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer->close();

}

void pipeline::showInfo(){
    cout << "porcentajeplanos\t" << porcentaje_planos << endl;
    cout << "threshold       \t" << threshold << endl;
    cout << "puntos restantes\t" << puntos_restantes << endl;
    cout << "Eliminar planos \t" << duration_elim_planes.count() << endl;
    cout << "Proceso         \t" << duration_proces.count() << endl;
    cout << "Keypoints scene \t" << num_keypoints_scene << endl;
    cout << "Keypoints obj   \t" << num_keypoints_obj << endl;
    cout << "Descriptor scene\t" << num_descriptors_scene << endl;
    cout << "Descriptor obj  \t" << num_descriptors_obj << endl;
    cout << "emparejamientos \t" << num_emparejamientos << endl;
    cout << "correctos       \t" << num_emparejamientos_correctos << endl;
    cout << "accuracy        \t" << accuracy << endl;
}

PointCloud<PointXYZI>::Ptr pipeline::Harris3D(string point_cloud){
	
	pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI> detector; //Se crea el detector
	
	//Parámetros del detector
	detector.setNonMaxSupression(true); 
	//detector.setRadius(0.03);
	detector.setThreshold(1e-6);
	
	// Se calculan los keypoints de la escena o el objeto ependiendo de la variable string que se pasa por parámetro
	if(point_cloud == "escena"){
		detector.setInputCloud(scene_harris);
	}
	else{
		detector.setInputCloud(obj_harris);
	}
		
	// Se crea el vector de keypoints
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	 
	// Se calculan los keypoints
	detector.compute(*keypoints);
  
	return keypoints;
}

PointCloud<Normal>::Ptr pipeline::get_Normals_Harris(PointCloud<PointXYZI>::Ptr point_cloud){
	
	// Se crea el detector de normales
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normals;
	
	// Se carga la nube a procesar
	normals.setInputCloud(point_cloud);
	
	// Se crea el método de búsqueda
	pcl::search::KdTree<pcl::PointXYZI>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZI> ());
	normals.setSearchMethod (Kdtree);
	
	// Se crea el vector donde se guardan las normales
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Se introducen los parámetros del detector
	normals.setRadiusSearch (0.03);

	// Se calculan las normales
	normals.compute (*cloud_normals);
		
	return cloud_normals;
}

PointCloud<Normal>::Ptr pipeline::get_Normals_ISS(PointCloud<PointXYZRGBA>::Ptr point_cloud){
	
	// Se crea el detector de normales
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normals;
	
	// Se carga la nube a procesar
	normals.setInputCloud(point_cloud);
	
	// Se crea el método de búsqueda
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	normals.setSearchMethod (Kdtree);
	
	// Se crea el vector donde se guardan las normales
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Se introducen los parámetros del detector
	normals.setRadiusSearch (0.03);

	// Se calculan las normales
	normals.compute (*cloud_normals);
		
	return cloud_normals;
}

void pipeline::start(){
	
	auto start = std::chrono::system_clock::now();
    eliminatePlanes();
    auto end = std::chrono::system_clock::now();
    duration_elim_planes = end - start;


    if(keypoints==1 and descriptors==1){
        start = std::chrono::system_clock::now();
        method_HARRIS_PFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==1 and descriptors==2){
        start = std::chrono::system_clock::now();
        method_HARRIS_FPFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==1 and descriptors==3){
        start = std::chrono::system_clock::now();
        method_HARRIS_RSD();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==1 and descriptors==4){
        start = std::chrono::system_clock::now();
        method_HARRIS_SHOT();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==2 and descriptors==1){
        start = std::chrono::system_clock::now();
        method_SIFT_PFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==2 and descriptors==2){
        start = std::chrono::system_clock::now();
        method_SIFT_FPFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==2 and descriptors==3){
        start = std::chrono::system_clock::now();
        method_SIFT_RSD();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==2 and descriptors==4){
        start = std::chrono::system_clock::now();
        method_SIFT_SHOT();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==3 and descriptors==1){
        start = std::chrono::system_clock::now();
        method_ISS_PFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==3 and descriptors==2){
        start = std::chrono::system_clock::now();
        method_ISS_FPFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==3 and descriptors==3){
        start = std::chrono::system_clock::now();
        method_ISS_RSD();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==3 and descriptors==4){
        start = std::chrono::system_clock::now();
        method_ISS_SHOT();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==4 and descriptors==1){
        start = std::chrono::system_clock::now();
        method_US_PFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==4 and descriptors==2){
        start = std::chrono::system_clock::now();
        method_US_FPFH();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else if(keypoints==4 and descriptors==3){
        start = std::chrono::system_clock::now();
        method_US_SHOT();
        end = std::chrono::system_clock::now();
        duration_proces = end - start;
    }
    else{
        cout << "Seleccioon no valida" << endl;
    }
}

void pipeline::eliminatePlanes(){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aux(scene);
    size_t pts = scene->points.size(); 
    porcentaje_planos = 0.5;
    threshold = 0.012;
    while(scene->points.size() > pts*porcentaje_planos){

        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        seg.setInputCloud(scene);

        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(threshold);
        seg.setMaxIterations(500);

        
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers           (new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0)
            cerr << "Could not estimate a planar model" << endl;
            

        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud(scene);
        extract.setIndices(inliers);

        extract.setNegative(true);
        extract.filter(*aux);

        scene->swap(*aux);
    }
    puntos_restantes = scene->size();
}

void pipeline::method_HARRIS_PFH(){
	
	
	//
	// ------ CÁLCULO DE KEYPOINTS ------
	//
	
	// DETECTOR HARRIS
	
	PointCloud<PointXYZI>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	PointCloud<PointXYZI>::Ptr object_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	
	string s = "escena";
	string o = "object";
	scene_keypoints = Harris3D(s);
	object_keypoints = Harris3D(o);	
	
	//pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
	
	num_keypoints_scene = scene_keypoints->size();
	num_keypoints_obj = object_keypoints->size();
	
	
	
	/*
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr s_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*scene_keypoints,*s_cloud_keypoints);
    
    cout << "Keypoints: " << s_cloud_keypoints->size() << endl;
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> custom(s_cloud_keypoints,255,0,0);
    
    viewer.addPointCloud<pcl::PointXYZRGBA> (scene, rgb,"id0");
    viewer.addPointCloud<pcl::PointXYZRGBA> (s_cloud_keypoints, custom, "id1");
	
	while(!viewer.wasStopped ()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
	*/
	
    //
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA HARRIS
	
	PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_Harris(scene_keypoints);
	object_normals = get_Normals_Harris(object_keypoints);
	
	
	//
	// ------ CALCULO DE DESCRIPTORES ------
	//
	
	// DESCRIPTOR PFH
	
	// Se crean lso descriptores de la escena y del objeto
	pcl::PFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125> pfh_scene;
	pcl::PFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125> pfh_object;
	
	// Se carga las nubes de keypoints y las normales
	pfh_scene.setInputCloud(scene_keypoints);
	pfh_scene.setInputNormals(scene_normals);	
	pfh_object.setInputCloud(object_keypoints);
	pfh_object.setInputNormals(object_normals);
	
	// Se establece el método de búsqueda
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZI>());
	
	pfh_scene.setSearchMethod(tree);
	pfh_object.setSearchMethod(tree2);
	
	// Se crean los vectores que contendrán los descriptores
	pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr object_descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
	
	// Se configuran los parámetros de los descriptores
	pfh_scene.setRadiusSearch(0.03);
	pfh_object.setRadiusSearch(0.03);

	// Se calculan los descriptores
	pfh_scene.compute(*scene_descriptors);
	pfh_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_scene = scene_descriptors->size();
    num_descriptors_obj = object_descriptors->size();
	
	//
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//
	
	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125,pcl::PFHSignature125> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	Correspondences matches;

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(matches);
	
	num_emparejamientos = matches.size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//
	
	// Se crea el vector de correspondencias corregidas
	Correspondences matches_corrected;	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZI> rej_samp;
	
	
	// Se cargan las nubes de keypoints
	rej_samp.setInputSource(object_keypoints);
	rej_samp.setInputTarget(scene_keypoints);
		
	// Se refinana las correspondencias para eliminar emparejamientos incorrectos
	rej_samp.getRemainingCorrespondences(matches, matches_corrected);
	
	cout << endl << "Emparejamientos calculados: " << matches_corrected.size () << endl << endl;;

	num_emparejamientos_correctos = matches_corrected.size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	// El método anterior de refinamiento ya tiene una matriz de transformación
	if(matches.size()>0){	
		trans = rej_samp.getBestTransformation();
		cout << trans << endl;
	}
	else
		cout << "No hay matriz de transformacion" << endl;


	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//
	
	transformPointCloud(*obj_harris,*obj_harris,trans);
	
	// Se refina la matriz de transformación para ajustar sus valores para hacer la transformación más precisa
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(obj_harris);
    icp.setInputTarget(scene_harris);
    
    // Se configuran los valores
    //icp.setMaxCorrespondenceDistance(100);  
    //icp.setTransformationEpsilon(1e-10); 
    //icp.setEuclideanFitnessEpsilon(0.001); 
    //icp.setMaximumIterations(100); 
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr resultante (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*resultante);
	
	// Se calcula la transformación refinada
    bestTrans = icp.getFinalTransformation();
    
    icp.align(*obj_harris); 
    
    //pcl::transformPointCloud (*obj_harris, *obj_harris, bestTrans);
     
    // Se calcula la precisión obtenida
    accuracy = icp.getFitnessScore();
    
	
}

void pipeline::method_HARRIS_FPFH(){
	
	//
	// ------ CÁLCULO DE KEYPOINTS ------
	//
	
	// DETECTOR HARRIS
	
	PointCloud<PointXYZI>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	PointCloud<PointXYZI>::Ptr object_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	
	string s = "escena";
	string o = "object";
	scene_keypoints = Harris3D(s);
	object_keypoints = Harris3D(o);
	
	num_keypoints_scene = scene_keypoints->size();
	num_keypoints_obj = object_keypoints->size();
	
	//
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA HARRIS
	
	PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_Harris(scene_keypoints);
	object_normals = get_Normals_Harris(object_keypoints);
	
	//
	// ------ CALCULO DE DESCRIPTORES ------
	//
	
	// DESCRIPTOR FPFH
	
	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_scene;
	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_object;
	
	fpfh_scene.setInputCloud(scene_keypoints);
	fpfh_scene.setInputNormals(scene_normals);	
	fpfh_object.setInputCloud(object_keypoints);
	fpfh_object.setInputNormals(object_normals);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	
	fpfh_scene.setSearchMethod(tree);
	fpfh_object.setSearchMethod(tree);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

	fpfh_scene.setRadiusSearch(0.05);
	fpfh_object.setRadiusSearch(0.05);

	fpfh_scene.compute(*scene_descriptors);
	fpfh_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_scene = scene_descriptors->size();
    num_descriptors_obj = object_descriptors->size();
	
	//
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//

	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);
	
	num_emparejamientos = matches->size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//

	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZI> rej_samp;
	
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
	num_emparejamientos_correctos = matches_corrected->size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
	}
	else
		cout << "No hay matriz de transformacion" << endl;
		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//

    transformPointCloud(*obj,*obj,trans);
	
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();
    

    icp.align(*obj);
    
    accuracy = icp.getFitnessScore();
	
}

void pipeline::method_HARRIS_RSD(){
	
	//
	// ------ CÁLCULO DE KEYPOINTS ------
	//
	
	// DETECTOR HARRIS
	
	PointCloud<PointXYZI>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	PointCloud<PointXYZI>::Ptr object_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	
	string s = "escena";
	string o = "object";
	scene_keypoints = Harris3D(s);
	object_keypoints = Harris3D(o);
	
	num_keypoints_scene = scene_keypoints->size();
	num_keypoints_obj = object_keypoints->size();
	
	//
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA HARRIS
	
	PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_Harris(scene_keypoints);
	object_normals = get_Normals_Harris(object_keypoints);
	
	//
	// ------ CALCULO DE DESCRIPTORES ------
	//
	
	// DESCRIPTOR RSD
	
	pcl::RSDEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalRadiiRSD> rsd_scene;
	pcl::RSDEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalRadiiRSD> rsd_object;
	
	rsd_scene.setInputCloud(scene_keypoints);
	rsd_scene.setInputNormals(scene_normals);
	
	rsd_object.setInputCloud(object_keypoints);
	rsd_object.setInputNormals(object_normals);
	
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	rsd_scene.setSearchMethod(tree);	
	rsd_object.setSearchMethod(tree);

	rsd_scene.setRadiusSearch(0.05);
	rsd_object.setRadiusSearch(0.05);
	
	
	rsd_scene.setPlaneRadius(0.1);
	rsd_object.setPlaneRadius(0.1);
	
	rsd_scene.setSaveHistograms(false);
	rsd_object.setSaveHistograms(false);
	
	pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr scene_descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
	pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr object_descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
	
	rsd_scene.compute(*scene_descriptors);
	rsd_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_obj = object_descriptors->size();
    num_descriptors_scene = scene_descriptors->size();
	
	//
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//

		// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::PrincipalRadiiRSD,pcl::PrincipalRadiiRSD> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);
	
	num_emparejamientos = matches->size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//

	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZI> rej_samp;
	
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
	num_emparejamientos_correctos = matches_corrected->size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
	}
	else
		cout << "No hay matriz de transformacion" << endl;
		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//

    transformPointCloud(*obj,*obj,trans);
	
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();
    

    icp.align(*obj);
    
    accuracy = icp.getFitnessScore();
	

}

void pipeline::method_HARRIS_SHOT(){
	
	//
	// ------ CÁLCULO DE KEYPOINTS ------
	//
	
	// DETECTOR HARRIS
	
	PointCloud<PointXYZI>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	PointCloud<PointXYZI>::Ptr object_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	
	string s = "escena";
	string o = "object";
	scene_keypoints = Harris3D(s);
	object_keypoints = Harris3D(o);
	
	num_keypoints_obj = object_keypoints->size();
	num_keypoints_scene = scene_keypoints->size();
	
	//
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA HARRIS
	
	PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_Harris(scene_keypoints);
	object_normals = get_Normals_Harris(object_keypoints);
	
	//
	// ------ CALCULO DE DESCRIPTORES ------
	//
	
	// DESCRIPTOR SHOT
	
	pcl::SHOTEstimation<pcl::PointXYZI, pcl::Normal, SHOT352> shot_scene;
	pcl::SHOTEstimation<pcl::PointXYZI, pcl::Normal, SHOT352> shot_object;
	
	shot_scene.setInputCloud(scene_keypoints);
	shot_scene.setInputNormals(scene_normals);
	
	shot_object.setInputCloud(object_keypoints);
	shot_object.setInputNormals(object_normals);
	
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	//shot_scene.setSearchMethod(tree);	
	//shot_object.setSearchMethod(tree);

	shot_scene.setRadiusSearch(0.1);
	shot_object.setRadiusSearch(0.1);
	
	//shot_scene.setSearchSurface (scene);
	//shot_scene.setSearchSurface (obj);
	
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr object_descriptors(new pcl::PointCloud<pcl::SHOT352>());
	
	shot_scene.compute(*scene_descriptors);
	shot_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_obj = object_descriptors->size();
    num_descriptors_scene = scene_descriptors->size();
	
	//
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//

	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::SHOT352,pcl::SHOT352> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);
	
	num_emparejamientos = matches->size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//

	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZI> rej_samp;
	
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
	cout << endl << "Emparejamientos calculados: " << matches_corrected->size() << endl;
	
	num_emparejamientos_correctos = matches_corrected->size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
		cout << trans << endl;
	}
	else
		cout << "No hay matriz de transformacion" << endl;
		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//
	
    transformPointCloud(*obj,*obj, trans);

	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();
    

    icp.align(*obj);
    
    accuracy = icp.getFitnessScore();

}

void pipeline::method_SIFT_PFH(){

    /* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointWithScale>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> scene_detector;
    
    pcl::PointCloud<pcl::PointWithScale>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setScales(0.001, 5, 10);
    scene_detector.setMinimumContrast(0.3f);   
    //scene_detector.setSearchMethod(tree);
    scene_detector.compute(*scene_cloud_keypoints); 

    object_detector.setInputCloud(obj);    
    object_detector.setScales(0.001, 5, 10);
    object_detector.setMinimumContrast(0.3f);  
    //object_detector.setSearchMethod(tree); 
    object_detector.compute(*object_cloud_keypoints);

    num_keypoints_scene   = scene_cloud_keypoints->size();
    num_keypoints_obj     = object_cloud_keypoints->size();


    /* Cambio del tipo de nube */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr s_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*scene_cloud_keypoints,*s_cloud_keypoints);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr o_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*object_cloud_keypoints,*o_cloud_keypoints);
    

    /* CALCULO DE DESCRIPTORES */
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(s_cloud_keypoints);
    scene_normals.setSearchMethod (kdtree);
    scene_normals.setRadiusSearch (0.03);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(o_cloud_keypoints);
    object_normals.setSearchMethod (kdtree);
    object_normals.setRadiusSearch (0.03);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> scene_descriptor;
    scene_descriptor.setInputCloud(s_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setSearchMethod(kdtree);
    scene_descriptor.setRadiusSearch(0.03);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> object_descriptor;
    object_descriptor.setInputCloud(o_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setSearchMethod(kdtree);
    object_descriptor.setRadiusSearch(0.03);
    object_descriptor.compute(*object_cloud_descriptor);

    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125,pcl::PFHSignature125> corresp;
    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches);    
    num_emparejamientos = matches->size();

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointWithScale> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();
    

    showMatches(s_cloud_keypoints,o_cloud_keypoints,*matches_corrected);


}

void pipeline::method_SIFT_FPFH(){

/* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointWithScale>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> scene_detector;
    
    pcl::PointCloud<pcl::PointWithScale>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setScales(0.001, 5, 10);
    scene_detector.setMinimumContrast(0.3f);   
    //scene_detector.setSearchMethod(tree);
    scene_detector.compute(*scene_cloud_keypoints); 

    object_detector.setInputCloud(obj);    
    object_detector.setScales(0.001, 5, 10);
    object_detector.setMinimumContrast(0.3f);  
    //object_detector.setSearchMethod(tree); 
    object_detector.compute(*object_cloud_keypoints);

    num_keypoints_scene   = scene_cloud_keypoints->size();
    num_keypoints_obj     = object_cloud_keypoints->size();


    /* Cambio del tipo de nube */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr s_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*scene_cloud_keypoints,*s_cloud_keypoints);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr o_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*object_cloud_keypoints,*o_cloud_keypoints);
    

    /* CALCULO DE DESCRIPTORES */
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(s_cloud_keypoints);
    scene_normals.setSearchMethod (kdtree);
    scene_normals.setRadiusSearch (0.03);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(o_cloud_keypoints);
    object_normals.setSearchMethod (kdtree);
    object_normals.setRadiusSearch (0.03);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> scene_descriptor;
    scene_descriptor.setInputCloud(s_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setSearchMethod(kdtree);
    scene_descriptor.setRadiusSearch(0.03);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> object_descriptor;
    object_descriptor.setInputCloud(o_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setSearchMethod(kdtree);
    object_descriptor.setRadiusSearch(0.03);
    object_descriptor.compute(*object_cloud_descriptor);

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> corresp;
    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches);  
    num_emparejamientos = matches->size();

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointWithScale> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();

    showMatches(s_cloud_keypoints,o_cloud_keypoints,*matches_corrected);

}

void pipeline::method_SIFT_RSD(){

    /* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointWithScale>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> scene_detector;
    
    pcl::PointCloud<pcl::PointWithScale>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setScales(0.001, 5, 10);
    scene_detector.setMinimumContrast(0.3f);   
    //scene_detector.setSearchMethod(tree);
    scene_detector.compute(*scene_cloud_keypoints); 

    object_detector.setInputCloud(obj);    
    object_detector.setScales(0.001, 5, 10);
    object_detector.setMinimumContrast(0.3f);  
    //object_detector.setSearchMethod(tree); 
    object_detector.compute(*object_cloud_keypoints);


    num_keypoints_scene = scene_cloud_keypoints->size();
    num_keypoints_obj   = object_cloud_keypoints->size();

    /* Cambio del tipo de nube */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr s_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*scene_cloud_keypoints,*s_cloud_keypoints);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr o_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*object_cloud_keypoints,*o_cloud_keypoints);
    

    /* CALCULO DE DESCRIPTORES */
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(s_cloud_keypoints);
    scene_normals.setSearchMethod (kdtree);
    scene_normals.setRadiusSearch (0.03);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(o_cloud_keypoints);
    object_normals.setSearchMethod (kdtree);
    object_normals.setRadiusSearch (0.03);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    pcl::RSDEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalRadiiRSD> scene_descriptor;
    scene_descriptor.setInputCloud(s_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setSearchMethod(kdtree);
    scene_descriptor.setRadiusSearch(0.05);
    scene_descriptor.setPlaneRadius(0.1);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    pcl::RSDEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalRadiiRSD> object_descriptor;
    object_descriptor.setInputCloud(o_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setSearchMethod(kdtree);
    object_descriptor.setRadiusSearch(0.05);
    object_descriptor.setPlaneRadius(0.1);
    object_descriptor.compute(*object_cloud_descriptor);

    pcl::registration::CorrespondenceEstimation<pcl::PrincipalRadiiRSD,pcl::PrincipalRadiiRSD> corresp;
    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches);    
    num_emparejamientos = matches->size();

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointWithScale> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();

    showMatches(s_cloud_keypoints,o_cloud_keypoints,*matches_corrected);

}

void pipeline::method_SIFT_SHOT(){
    
    /* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointWithScale>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> scene_detector;
    
    pcl::PointCloud<pcl::PointWithScale>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::SIFTKeypoint<pcl::PointXYZRGBA,pcl::PointWithScale> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setScales(0.001, 5, 10);
    scene_detector.setMinimumContrast(0.3f);   
    //scene_detector.setSearchMethod(tree);
    scene_detector.compute(*scene_cloud_keypoints); 

    object_detector.setInputCloud(obj);    
    object_detector.setScales(0.001, 5, 10);
    object_detector.setMinimumContrast(0.3f);  
    //object_detector.setSearchMethod(tree); 
    object_detector.compute(*object_cloud_keypoints);


    num_keypoints_scene = scene_cloud_keypoints->size();
    num_keypoints_obj   = object_cloud_keypoints->size();


    /* Cambio del tipo de nube */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr s_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*scene_cloud_keypoints,*s_cloud_keypoints);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr o_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
    copyPointCloud(*object_cloud_keypoints,*o_cloud_keypoints);
    

    /* CALCULO DE DESCRIPTORES */
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(s_cloud_keypoints);
    scene_normals.setSearchMethod (kdtree);
    scene_normals.setRadiusSearch (0.03);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(o_cloud_keypoints);
    object_normals.setSearchMethod (kdtree);
    object_normals.setRadiusSearch (0.03);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::SHOT352>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> scene_descriptor;
    scene_descriptor.setInputCloud(s_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setSearchMethod(kdtree);
    scene_descriptor.setRadiusSearch(0.05);
    scene_descriptor.setKSearch(0);
    scene_descriptor.setSearchSurface(scene);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::SHOT352>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> object_descriptor;
    object_descriptor.setInputCloud(o_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setSearchMethod(kdtree);
    object_descriptor.setRadiusSearch(0.05);
    object_descriptor.setKSearch(0);
    object_descriptor.setSearchSurface(obj);
    object_descriptor.compute(*object_cloud_descriptor);

    pcl::registration::CorrespondenceEstimation<pcl::SHOT352,pcl::SHOT352> corresp;
    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches);    
    num_emparejamientos = matches->size();

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointWithScale> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();

    //showMatches(s_cloud_keypoints,o_cloud_keypoints,*matches_corrected);

}

double pipeline::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud){
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZRGBA> tree;
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

void pipeline::method_ISS_PFH(){
	
	//
	// DETECCIÓN DE KEYPOINTS
	//
	
	// ISS
			
	pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> scene_detector;
    pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> object_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	
    search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new search::KdTree<pcl::PointXYZRGBA>);
    
    double scene_resolution = computeCloudResolution(scene);
    double object_resolution = computeCloudResolution(obj);

    scene_detector.setInputCloud(scene);
    scene_detector.setSearchMethod(kdtree);
    scene_detector.setSalientRadius(6 * scene_resolution);
    scene_detector.setNonMaxRadius(4 * scene_resolution);
    scene_detector.setMinNeighbors(5);
    scene_detector.setThreshold21(0.975);
    scene_detector.setThreshold32(0.975);
    scene_detector.compute(*scene_keypoints);
    
    object_detector.setInputCloud(obj);
    object_detector.setSearchMethod(kdtree);
    object_detector.setSalientRadius(6 * object_resolution);
    object_detector.setNonMaxRadius(4 * object_resolution);
    object_detector.setMinNeighbors(5);
    object_detector.setThreshold21(0.975);
    object_detector.setThreshold32(0.975);
    object_detector.compute(*object_keypoints);
    
    num_keypoints_obj = object_keypoints->size();
    num_keypoints_scene = scene_keypoints->size();
    
    cout << " KEYPOINTS CALCULADOS" << endl;
    
    //
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA ISS
	
	PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_ISS(scene_keypoints);
	object_normals = get_Normals_ISS(object_keypoints);
	
	cout << "NORMALES CALCULADOS" << endl;
    
    
    //
    // ----- CÁLCULO DE LOS DESCRIPTORES -----
    //
    
    // DESCRIPTOR PFH
	
	// Se crean lso descriptores de la escena y del objeto
	pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> pfh_scene;
	pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> pfh_object;
	
	// Se carga las nubes de keypoints y las normales
	pfh_scene.setInputCloud(scene_keypoints);
	pfh_scene.setInputNormals(scene_normals);	
	pfh_object.setInputCloud(object_keypoints);
	pfh_object.setInputNormals(object_normals);
	
	// Se establece el método de búsqueda
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	
	pfh_scene.setSearchMethod(tree);
	pfh_object.setSearchMethod(tree);
	
	// Se crean los vectores que contendrán los descriptores
	pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::PFHSignature125>::Ptr object_descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
	
	// Se configuran los parámetros de los descriptores
	pfh_scene.setRadiusSearch(0.03);
	pfh_object.setRadiusSearch(0.03);

	// Se calculan los descriptores
	pfh_scene.compute(*scene_descriptors);
	pfh_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    
    //
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//
	
	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125,pcl::PFHSignature125> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);
	
	
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//
	
	// Se crea el vector de correspondencias corregidas
	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
	
	// Se cargan las nubes de keypoints
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	// Se refinana las correspondencias para eliminar emparejamientos incorrectos
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
    //num_descriptors_scene = de
	
	num_emparejamientos = matches->size();
	num_emparejamientos_correctos = matches_corrected->size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	// El método anterior de refinamiento ya tiene una matriz de transformación
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
		
	}
	else
		cout << "No hay matriz de transformacion" << endl;
	
		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//
	
	transformPointCloud(*obj,*obj,trans);
	
	// Se refina la matriz de transformación para ajustar sus valores para hacer la transformación más precisa
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    // Se configuran los valores
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

	
	// Se calcula la transformación refinada
    bestTrans = icp.getFinalTransformation();
    
    //pcl::transformPointCloud (*obj, *obj, bestTrans);
    
    icp.align(*obj);
     
    // Se calcula la precisión obtenida
    accuracy = icp.getFitnessScore();	
    showMatches(scene_keypoints,object_keypoints,*matches_corrected);

}

void pipeline::method_ISS_FPFH(){
	
	//
	// DETECCIÓN DE KEYPOINTS
	//
	
	// ISS
			
	pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> scene_detector;
    pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> object_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	
    search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new search::KdTree<pcl::PointXYZRGBA>);
    
    double scene_resolution = computeCloudResolution(scene);
    double object_resolution = computeCloudResolution(obj);

    scene_detector.setInputCloud(scene);
    scene_detector.setSearchMethod(kdtree);
    scene_detector.setSalientRadius(6 * scene_resolution);
    scene_detector.setNonMaxRadius(4 * scene_resolution);
    scene_detector.setMinNeighbors(5);
    scene_detector.setThreshold21(0.975);
    scene_detector.setThreshold32(0.975);
    scene_detector.compute(*scene_keypoints);
    
    object_detector.setInputCloud(obj);
    object_detector.setSearchMethod(kdtree);
    object_detector.setSalientRadius(6 * object_resolution);
    object_detector.setNonMaxRadius(4 * object_resolution);
    object_detector.setMinNeighbors(5);
    object_detector.setThreshold21(0.975);
    object_detector.setThreshold32(0.975);
    object_detector.compute(*object_keypoints);
    
    num_keypoints_obj = object_keypoints->size();
    num_keypoints_scene = scene_keypoints->size();
    
    //
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA ISS
	
	PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_ISS(scene_keypoints);
	object_normals = get_Normals_ISS(object_keypoints);

    
    
    //
    // ----- CÁLCULO DE LOS DESCRIPTORES -----
    //
    
    // DESCRIPTOR PFH
	
	// Se crean lso descriptores de la escena y del objeto
	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_scene;
	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_object;
	
	// Se carga las nubes de keypoints y las normales
	fpfh_scene.setInputCloud(scene_keypoints);
	fpfh_scene.setInputNormals(scene_normals);	
	fpfh_object.setInputCloud(object_keypoints);
	fpfh_object.setInputNormals(object_normals);
	
	// Se establece el método de búsqueda
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	
	fpfh_scene.setSearchMethod(tree);
	fpfh_object.setSearchMethod(tree);
	
	// Se crean los vectores que contendrán los descriptores
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
	
	// Se configuran los parámetros de los descriptores
	fpfh_scene.setRadiusSearch(0.03);
	fpfh_object.setRadiusSearch(0.03);

	// Se calculan los descriptores
	fpfh_scene.compute(*scene_descriptors);
	fpfh_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_obj = object_descriptors->size();
    num_descriptors_scene = scene_descriptors->size();
    
    //
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//
	
	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);
	
	num_emparejamientos = matches->size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//
	
	// Se crea el vector de correspondencias corregidas
	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
	
	// Se cargan las nubes de keypoints
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	// Se refinana las correspondencias para eliminar emparejamientos incorrectos
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
	num_emparejamientos_correctos = matches_corrected->size();
	
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	// El método anterior de refinamiento ya tiene una matriz de transformación
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
		cout << trans << endl;
	}
	else
		cout << "No hay matriz de transformacion" << endl;
		
		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//
	
	transformPointCloud(*obj,*obj,trans);
	
	// Se refina la matriz de transformación para ajustar sus valores para hacer la transformación más precisa
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    // Se configuran los valores
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

	
	// Se calcula la transformación refinada
    bestTrans = icp.getFinalTransformation();
    
    //pcl::transformPointCloud (*obj, *obj, bestTrans);
    
    icp.align(*obj);
     
    // Se calcula la precisión obtenida
    accuracy = icp.getFitnessScore();
    showMatches(scene_keypoints,object_keypoints,*matches_corrected);

}

void pipeline::method_ISS_RSD(){
	
	//
	// DETECCIÓN DE KEYPOINTS
	//
	
	// ISS
			
	pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> scene_detector;
    pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> object_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	
    search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new search::KdTree<pcl::PointXYZRGBA>);
    
    double scene_resolution = computeCloudResolution(scene);
    double object_resolution = computeCloudResolution(obj);

    scene_detector.setInputCloud(scene);
    scene_detector.setSearchMethod(kdtree);
    scene_detector.setSalientRadius(6 * scene_resolution);
    scene_detector.setNonMaxRadius(4 * scene_resolution);
    scene_detector.setMinNeighbors(5);
    scene_detector.setThreshold21(0.975);
    scene_detector.setThreshold32(0.975);
    scene_detector.compute(*scene_keypoints);
    
    object_detector.setInputCloud(obj);
    object_detector.setSearchMethod(kdtree);
    object_detector.setSalientRadius(6 * object_resolution);
    object_detector.setNonMaxRadius(4 * object_resolution);
    object_detector.setMinNeighbors(5);
    object_detector.setThreshold21(0.975);
    object_detector.setThreshold32(0.975);
    object_detector.compute(*object_keypoints);
    
    num_keypoints_obj = object_keypoints->size();
    num_keypoints_scene = scene_keypoints->size();
    

    //
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA ISS
	
	PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_ISS(scene_keypoints);
	object_normals = get_Normals_ISS(object_keypoints);
	
    
    
    //
	// ------ CALCULO DE DESCRIPTORES ------
	//
	
	// DESCRIPTOR RSD
	
	pcl::RSDEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalRadiiRSD> rsd_scene;
	pcl::RSDEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PrincipalRadiiRSD> rsd_object;
	
	rsd_scene.setInputCloud(scene_keypoints);
	rsd_scene.setInputNormals(scene_normals);
	
	rsd_object.setInputCloud(object_keypoints);
	rsd_object.setInputNormals(object_normals);
	
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	rsd_scene.setSearchMethod(tree);	
	rsd_object.setSearchMethod(tree);

	rsd_scene.setRadiusSearch(0.05);
	rsd_object.setRadiusSearch(0.05);
	
	
	rsd_scene.setPlaneRadius(0.1);
	rsd_object.setPlaneRadius(0.1);
	
	rsd_scene.setSaveHistograms(false);
	rsd_object.setSaveHistograms(false);
	
	pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr scene_descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
	pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr object_descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
	
	rsd_scene.compute(*scene_descriptors);
	rsd_object.compute(*object_descriptors);
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_obj = object_descriptors->size();
    num_descriptors_scene = scene_descriptors->size();
    
    
    //
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//
	
	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::PrincipalRadiiRSD,pcl::PrincipalRadiiRSD> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);

	num_emparejamientos = matches->size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//
	
	// Se crea el vector de correspondencias corregidas
	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
	
	// Se cargan las nubes de keypoints
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	// Se refinana las correspondencias para eliminar emparejamientos incorrectos
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
	cout << endl << "Emparejamientos calculados: " << matches_corrected->size () << endl;
	
	num_emparejamientos_correctos = matches_corrected->size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	// El método anterior de refinamiento ya tiene una matriz de transformación
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
		cout << trans << endl;
	}
	else
		cout << "No hay matriz de transformacion" << endl;
			
		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//
	
	transformPointCloud(*obj,*obj,trans);
	
	// Se refina la matriz de transformación para ajustar sus valores para hacer la transformación más precisa
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    // Se configuran los valores
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

	
	// Se calcula la transformación refinada
    bestTrans = icp.getFinalTransformation();
    
    //pcl::transformPointCloud (*obj, *obj, bestTrans);
    
    icp.align(*obj);
     
    // Se calcula la precisión obtenida
    accuracy = icp.getFitnessScore();
    showMatches(scene_keypoints,object_keypoints,*matches_corrected);

}

void pipeline::method_ISS_SHOT(){
	
	//
	// DETECCIÓN DE KEYPOINTS
	//
	
	// ISS
			
	pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> scene_detector;
    pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> object_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	
    search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new search::KdTree<pcl::PointXYZRGBA>);
    
    double scene_resolution = computeCloudResolution(scene);
    double object_resolution = computeCloudResolution(obj);

    scene_detector.setInputCloud(scene);
    scene_detector.setSearchMethod(kdtree);
    scene_detector.setSalientRadius(6 * scene_resolution);
    scene_detector.setNonMaxRadius(4 * scene_resolution);
    scene_detector.setMinNeighbors(5);
    scene_detector.setThreshold21(0.975);
    scene_detector.setThreshold32(0.975);
    scene_detector.compute(*scene_keypoints);
    
    object_detector.setInputCloud(obj);
    object_detector.setSearchMethod(kdtree);
    object_detector.setSalientRadius(6 * object_resolution);
    object_detector.setNonMaxRadius(4 * object_resolution);
    object_detector.setMinNeighbors(5);
    object_detector.setThreshold21(0.975);
    object_detector.setThreshold32(0.975);
    object_detector.compute(*object_keypoints);
    
    num_keypoints_obj = object_keypoints->size();
    num_keypoints_scene = scene_keypoints->size();
    
    //
	// ------ CÁLCULO DE NORMALES ------
	//
	
	// NORMALES PARA ISS
	
	PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal>);
	PointCloud<Normal>::Ptr object_normals(new PointCloud<Normal>);
	
	scene_normals = get_Normals_ISS(scene_keypoints);
	object_normals = get_Normals_ISS(object_keypoints);
	
    
    
    //
	// ------ CALCULO DE DESCRIPTORES ------
	//
	
	// DESCRIPTOR SHOT
	
	pcl::SHOTEstimation<pcl::PointXYZRGBA, pcl::Normal, SHOT352> shot_scene;
	pcl::SHOTEstimation<pcl::PointXYZRGBA, pcl::Normal, SHOT352> shot_object;
	
	shot_scene.setInputCloud(scene_keypoints);
	shot_scene.setInputNormals(scene_normals);
	
	shot_object.setInputCloud(object_keypoints);
	shot_object.setInputNormals(object_normals);
	
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	//shot_scene.setSearchMethod(tree);	
	//shot_object.setSearchMethod(tree);

	shot_scene.setRadiusSearch(0.1);
	shot_object.setRadiusSearch(0.1);
	
	//shot_scene.setSearchSurface (scene);
	//shot_scene.setSearchSurface (obj);
	
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr object_descriptors(new pcl::PointCloud<pcl::SHOT352>());
	
	shot_scene.compute(*scene_descriptors);
	shot_object.compute(*object_descriptors);
	
	cout << "Keypoints encontrados en este algoritmo: " << scene_descriptors->size() << endl;
	
	vector<int> aux;
	
	removeNaNFromPointCloud(*object_keypoints, *object_keypoints, aux);
    aux.clear();
    removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, aux);
    aux.clear();
    
    num_descriptors_obj = object_descriptors->size();
    num_descriptors_scene = scene_descriptors->size();
    
    
    //
	// ----- CÁLCULO DE CORRESPONDENCIAS -----
	//
	
	// Vector donde se almacenarán las correspondencias
	pcl::registration::CorrespondenceEstimation<pcl::SHOT352,pcl::SHOT352> correspondences;
	
	// Se crea el método de correspondencia y se le introduce la nube del objeto
	//pcl::KdTreeFLANN<pcl::PFHSignature125> match_search;
	correspondences.setInputSource(object_descriptors);
	correspondences.setInputTarget(scene_descriptors);
	
	CorrespondencesPtr matches(new Correspondences());

	// Se compara cada keypoint de la escena con cada keypoint del objeto
	correspondences.determineCorrespondences(*matches);
	
	num_emparejamientos = matches->size();
	
	//
	// ----- REFINAMIENTO DE CORRESPONDENCIAS -----
	//
	
	// Se crea el vector de correspondencias corregidas
	CorrespondencesPtr matches_corrected (new pcl::Correspondences ());	
	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
	
	// Se cargan las nubes de keypoints
	rej_samp.setInputTarget(scene_keypoints);
	rej_samp.setInputSource(object_keypoints);
	
	// Se refinana las correspondencias para eliminar emparejamientos incorrectos
	rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
	
	cout << endl << "Emparejamientos calculados: " << matches_corrected->size () << endl;
	
	num_emparejamientos_correctos = matches_corrected->size();
	
	//
	// ----- CÁLCULO DE TRANSFORMADAS -----
	//
	
	// El método anterior de refinamiento ya tiene una matriz de transformación
	if(matches->size()>0){	
		trans = rej_samp.getBestTransformation();
		cout << trans << endl;
	}
	else
		cout << "No hay matriz de transformacion" << endl;

		
	//
	// ----- REFINAMIENTO DE LA TRANSFORMADA -----
	//
	
	transformPointCloud(*obj,*obj,trans);
	
	// Se refina la matriz de transformación para ajustar sus valores para hacer la transformación más precisa
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    // Se configuran los valores
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100); 

	
	// Se calcula la transformación refinada
    bestTrans = icp.getFinalTransformation();
    
    //pcl::transformPointCloud (*obj, *obj, bestTrans);
    
    icp.align(*obj);
     
    // Se calcula la precisión obtenida
    accuracy = icp.getFitnessScore();	
}

void pipeline::method_US_PFH(){

    /* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::UniformSampling<pcl::PointXYZRGBA> scene_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::UniformSampling<pcl::PointXYZRGBA> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setRadiusSearch(0.03);
    scene_detector.filter(*scene_cloud_keypoints);

    object_detector.setInputCloud(obj);    
    object_detector.setRadiusSearch(0.03);
    object_detector.filter(*object_cloud_keypoints);


    num_keypoints_scene = scene_cloud_keypoints->size();
    num_keypoints_obj   = object_cloud_keypoints->size();
    

    /* CALCULO DE DESCRIPTORES */

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(scene_cloud_keypoints);
    scene_normals.setRadiusSearch (0.03);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(object_cloud_keypoints);
    object_normals.setRadiusSearch (0.03);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> scene_descriptor;
    scene_descriptor.setInputCloud(scene_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setRadiusSearch(0.03);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> object_descriptor;
    object_descriptor.setInputCloud(object_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setRadiusSearch(0.03);
    object_descriptor.compute(*object_cloud_descriptor);


    /* EMPAREJAMIENTOS */

    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125,pcl::PFHSignature125> corresp;

    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches);    
    num_emparejamientos = matches->size();

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    //icp.setMaxCorrespondenceDistance(100);  
    //icp.setTransformationEpsilon(1e-10); 
    //icp.setEuclideanFitnessEpsilon(0.001); 
    //icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    //transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();

    //showMatches(scene_cloud_keypoints,object_cloud_keypoints,*matches_corrected);

}

void pipeline::method_US_FPFH(){

    /* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::UniformSampling<pcl::PointXYZRGBA> scene_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::UniformSampling<pcl::PointXYZRGBA> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setRadiusSearch(0.03);
    scene_detector.filter(*scene_cloud_keypoints);

    object_detector.setInputCloud(obj);    
    object_detector.setRadiusSearch(0.03);
    object_detector.filter(*object_cloud_keypoints);


    num_keypoints_scene = scene_cloud_keypoints->size();
    num_keypoints_obj   = object_cloud_keypoints->size();
    

    /* CALCULO DE DESCRIPTORES */

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(scene_cloud_keypoints);
    scene_normals.setRadiusSearch (0.03);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(object_cloud_keypoints);
    object_normals.setRadiusSearch (0.03);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> scene_descriptor;
    scene_descriptor.setInputCloud(scene_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setRadiusSearch(0.03);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> object_descriptor;
    object_descriptor.setInputCloud(object_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setRadiusSearch(0.03);
    object_descriptor.compute(*object_cloud_descriptor);


    /* EMPAREJAMIENTOS */

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> corresp;

    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches); 
    num_emparejamientos = matches->size();   

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    //icp.setMaxCorrespondenceDistance(100);  
    //icp.setTransformationEpsilon(1e-10); 
    //icp.setEuclideanFitnessEpsilon(0.001); 
    //icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    //transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();

    //showMatches(scene_cloud_keypoints,object_cloud_keypoints,*matches_corrected);

}

void pipeline::method_US_SHOT(){

    /* CALCULO DE KEYPOINTS */

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::UniformSampling<pcl::PointXYZRGBA> scene_detector;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_cloud_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::UniformSampling<pcl::PointXYZRGBA> object_detector;

    scene_detector.setInputCloud(scene);    
    scene_detector.setRadiusSearch(0.03);
    scene_detector.filter(*scene_cloud_keypoints);

    object_detector.setInputCloud(obj);    
    object_detector.setRadiusSearch(0.03);
    object_detector.filter(*object_cloud_keypoints);

    num_keypoints_scene = scene_cloud_keypoints->size();
    num_keypoints_obj   = object_cloud_keypoints->size();
    

    /* CALCULO DE DESCRIPTORES */

    pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> scene_normals;  
    scene_normals.setInputCloud(scene_cloud_keypoints);
    scene_normals.setKSearch(10);
    scene_normals.compute(*scene_cloud_normals);
    
    pcl::PointCloud<pcl::Normal>::Ptr object_cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> object_normals;  
    object_normals.setInputCloud(object_cloud_keypoints);
    object_normals.setKSearch(10);
    object_normals.compute(*object_cloud_normals);

    pcl::PointCloud<pcl::SHOT352>::Ptr scene_cloud_descriptor(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> scene_descriptor;
    scene_descriptor.setInputCloud(scene_cloud_keypoints);
    scene_descriptor.setInputNormals(scene_cloud_normals);
    scene_descriptor.setSearchSurface(scene);
    scene_descriptor.compute(*scene_cloud_descriptor);

    pcl::PointCloud<pcl::SHOT352>::Ptr object_cloud_descriptor(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> object_descriptor;
    object_descriptor.setInputCloud(object_cloud_keypoints);
    object_descriptor.setInputNormals(object_cloud_normals);
    object_descriptor.setSearchSurface(obj);
    object_descriptor.compute(*object_cloud_descriptor);


    /* EMPAREJAMIENTOS */

    pcl::registration::CorrespondenceEstimation<pcl::SHOT352,pcl::SHOT352> corresp;

    corresp.setInputSource(object_cloud_descriptor);
    corresp.setInputTarget(scene_cloud_descriptor);

    pcl::CorrespondencesPtr matches (new pcl::Correspondences ());
    pcl::CorrespondencesPtr matches_corrected (new pcl::Correspondences ());  

    corresp.determineCorrespondences(*matches); 
    num_emparejamientos = matches->size();   

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBA> rej_samp;
    
    rej_samp.setInputSource(object_cloud_keypoints);
    rej_samp.setInputTarget(scene_cloud_keypoints);    
    rej_samp.getRemainingCorrespondences(*matches, *matches_corrected);
    num_emparejamientos_correctos = matches_corrected->size();

    trans = rej_samp.getBestTransformation();

    transformPointCloud(*obj,*obj,trans);



    /*  REFINAR     */

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(obj);
    icp.setInputTarget(scene);
    //icp.setMaxCorrespondenceDistance(100);  
    //icp.setTransformationEpsilon(1e-10); 
    //icp.setEuclideanFitnessEpsilon(0.001); 
    //icp.setMaximumIterations(100); 

    bestTrans = icp.getFinalTransformation();

    icp.align(*obj);

    transformPointCloud(*obj,*obj,bestTrans);

    accuracy = icp.getFitnessScore();
    num_descriptors_scene = scene_cloud_descriptor->points.size();
    num_descriptors_obj   = object_cloud_descriptor->points.size();

    //showMatches(scene_cloud_keypoints,object_cloud_keypoints,*matches_corrected);

}












