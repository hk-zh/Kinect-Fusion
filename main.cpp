#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "Volume.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include "FreeImageHelper.h"

#define MAX_FRAME_NUM 150
#define SAVE_RATE 5

int process_frame(int i, PointCloud &prevFrame, PointCloud &currentFrame, Volume &volume)
{
	//TODO:
	/*
     * ==========================================================
	 * |                      BEGIN                             |
	 * ==========================================================
     */

	// SETP 1: estimate Pose

	// STEP 2: Surface Reconstruction

	// STEP 3: Ray Casting

	/*
     * ==========================================================
	 * |                       END                              |
	 * ==========================================================
     */
	return 0;
};
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
	// TODO: initialize Volume
	Vector3d min(0, 0, 0);
	Vector3d max(0, 0, 0);
	Volume volume(min, max);
	/*
     * Configuration
     */

	/*
     * ==========================================================
	 * |                      BEGIN                             |
	 * ==========================================================
     */

	/*
     * ==========================================================
	 * |                       END                              |
	 * ==========================================================
     */

	/*
     * Process a first frame as a reference frame.
     * --> All next frames are tracked relatively to the first frame.
     */
	sensor.processNextFrame();
	const unsigned int depthWidth = sensor.getDepthImageWidth();
	const unsigned int depthHeight = sensor.getDepthImageHeight();
	PointCloud prevFrame;
	float *depth_map = sensor.getDepth();			   // get the depth map
	float *depth_map_filtered = sensor.getDepth(true); // get the filtered depth map

	/* if you want to virtualize the depth map and fitered depth map ,please use following code */
	// FreeImageU16F::SaveImageToFile(depth_map, "original.png", 640, 480, 1, true);
	// FreeImageU16F::SaveImageToFile(depth_map_filtered, "filtered.png", 640, 480, 1, true);

	// loop in our pipeline starts here
	int i = 0;
	while (i < MAX_FRAME_NUM && sensor.processNextFrame())
	{
		float *depth_map = sensor.getDepth(); // get the depth map

		// TODO: createFrame
		PointCloud currentFrame;

		process_frame(i, prevFrame, currentFrame, volume);
	}
	return 0;
}
