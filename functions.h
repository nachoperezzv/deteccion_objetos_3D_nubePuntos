#ifndef FUNCTIONS_H
#define FUNCTIONS_H

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

void 	showCloud		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);
void 	spawnViewer		(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);


#endif


