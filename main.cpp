#include <iostream>
#include <fstream>

#include "utils/Eigen.h"
#include "imageLoader/VirtualSensor.h"
#include "SimpleMesh.h"
#include "icp/ICPOptimizer.h"
#include "PointCloud.h"

#define MAX_FRAME_NUM 150

int main()
{
	// path to the data folder
	std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = std::string("../output/mesh_");
	std::string filenameBaseOutMC = std::string("../output/MCmesh_");

	// initialize virtual sensor
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	sensor.activefilter(); // active the filter
	if (!sensor.init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!" << std::endl;
		return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	PointCloud target{sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight()};

	// TODO: set up optimizer

	return 0;
}
