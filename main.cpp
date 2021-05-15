#include "functions.h"

using namespace std;



int main(int argc, char** argv){

	boost::thread visualizer;
	string pcdFile;

	//Checking if any parameter has been introduced
	if(argc > 1)
		pcdFile = string(argv[1]);
	else
		pcdFile = "../nubes/scenes/snap_0point.pcd";

	//Loading PC
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdFile, *pointCloud) == -1)	{
		cerr << "Error. Could not load the PCD File" <<endl;
		return -1;
	}

	showCloud(pointCloud);


	return 0;
}
