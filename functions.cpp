#include "functions.h"

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