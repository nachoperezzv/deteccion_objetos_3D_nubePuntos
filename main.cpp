#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>
#include <math.h>


using namespace std;

//Functions 
void 	showCloud		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);
void 	spawnViewer		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);


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

void showCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){
	boost::thread visualizer = boost::thread(spawnViewer, cloud);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void spawnViewer (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->addCoordinateSystem (1.0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (pointCloud, rgb, "id0");
    while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}