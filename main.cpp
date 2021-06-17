#include <iostream>
#include <fstream>

#include "utils/Eigen.h"
#include "imageLoader/VirtualSensor.h"
#include "SimpleMesh.h"
#include "icp/ICPOptimizer.h"
#include "PointCloud.h"
#include "imageLoader/FreeImageHelper.h"

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
	if (!sensor.init(filenameIn, 5, 1.0f))
	{
		std::cout << "Failed to initialize the sensor!" << std::endl;
		return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	PointCloud target{sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight()};

	/// BEGIN: Test for the filter
	float *depth_map = sensor.getDepth();
	float *depth_map_filtered = sensor.getDepth(true);
	FreeImageU16F::SaveImageToFile(depth_map, "original.png", 640, 480, 1, true);
	FreeImageU16F::SaveImageToFile(depth_map_filtered, "filtered.png", 640, 480, 1, true);

	// std::cout << "original:\n";
	// for (size_t i = 0; i < sensor.getDepthImageHeight() * sensor.getColorImageWidth(); ++i)
	// {
	// 	if (!(i % sensor.getColorImageWidth()))
	// 	{
	// 		std::cout << std::endl;
	// 	}
	// 	std::cout << depth_map[i] << " ";
	// }
	// std::cout << "filtered:\n";
	// for (size_t i = 0; i < sensor.getDepthImageHeight() * sensor.getColorImageWidth(); ++i)
	// {
	// 	if (!(i % sensor.getColorImageWidth()))
	// 	{
	// 		std::cout << std::endl;
	// 	}
	// 	std::cout << depth_map_filtered[i] << " ";
	// }
	// END:Test for the filter

	// TODO: set up optimizer

	return 0;
}
