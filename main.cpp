#include "functions.h"

using namespace std;
using namespace pcl;



int main(int argc, char** argv){

	boost::thread visualizer;
	string pcdFile;

	//Checking if any parameter has been introduced
	if(argc > 1)
		pcdFile = string(argv[1]);
	else
		pcdFile = "../nubes/scenes/snap_0point.pcd";

	//Loading PC
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcdFile, *pointCloud) == -1)	{
		cerr << "Error. Could not load the PCD File" <<endl;
		return -1;
	}

	showCloud(pointCloud);

	//Elimination of dominant planes of the scene

	/*
		Se van a hacer diferentes pruebas de diferentes metodos. 
		En primer luegar se va a emplear el modulo SEGMENTATION -> SACSEGMENTATION. 
		Esta clase deriva de PCLBase, de la que también se va a utilizar algunos metodos.

		- Primero se crea el objeto de la clase SACSegmentation
		- Se le pasa a dicho objeto la nube de puntos
		- Luego se seleccionan el modelo, metodo, numero máximo de iteraciones, 
		distancia umbral al modelo
		- Inicialización de los punteros a los coeficientes e inliers y segmentación y 
		checkeo de que se ha encontrado algun plano (de lo establecido en los parametros)
		en la nube de puntos cargada
		- Creación de objeto de filtrado
		- Extracción de los inliers


	*/
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		seg.setInputCloud(pointCloud);

		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setMaxIterations(1000);

		//seg.setOptimizeCoefficients(true);
		//Eigen::Vector3f axis = Eigen::Vector3f(1.0,1.0,1.0);
		//seg.setEpsAngle((60*CV_PI)/180);
		//seg.setAxis(axis);


		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers 			 (new pcl::PointIndices);
		seg.segment(*inliers, *coefficients);
		if(inliers->indices.size() == 0){
			cerr << "Could not estimate a planar model" << endl;
			return -1;
		}

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(pointCloud);
		extract.setIndices(inliers);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aux(new pcl::PointCloud<pcl::PointXYZRGBA>);
		extract.setNegative(false);
		extract.filter(*planes);


		extract.setNegative(true);
		extract.filter(*aux);

		cout << pointCloud->width << endl;
		cout << planes->width << endl;
		cout << aux->width << endl;


	showCloud(aux);


	return 0;
}
