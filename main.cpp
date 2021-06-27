#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "Volume.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include "FreeImageHelper.h"
#include "TSDF.h"
#include "MarchingCubes.h"

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

	// unit: meter
	// parameters for the volume model
	Vector3i volSize (256, 256, 256);
	float volUnit = 0.004f;
	float truncation = 0.012f;
	float init_depth = 1.0f;

	TSDF volume(volSize, volUnit, truncation);

	sensor.processNextFrame();
	Matrix4f currentPos;
    currentPos.setIdentity();
    currentPos(0, 3) = static_cast<float>(volume.volSz.x()) / 2.0f * volume.volUnit;
    currentPos(1, 3) = static_cast<float>(volume.volSz.y()) / 2.0f * volume.volUnit;
    currentPos(2, 3) = static_cast<float>(volume.volSz.z()) / 2.0f * volume.volUnit - init_depth;
	volume.update(sensor, currentPos);

	// ================================================
	// Direct MC on the model
    Volume MC_vol(Vector3d(-0.1,-0.1,-0.1), Vector3d(1.1, 1.1, 1.1), 200, 200, 200);

    for (unsigned int x = 0; x < MC_vol.getDimX(); x++)
    {
        for (unsigned int y = 0; y < MC_vol.getDimY(); y++)
        {
            for (unsigned int z = 0; z < MC_vol.getDimZ(); z++)
            {
                int xx = volume.volSz.x() * x / MC_vol.getDimX();
                int yy = volume.volSz.y() * y / MC_vol.getDimY();
                int zz = volume.volSz.z() * z / MC_vol.getDimZ();
                double val = volume.getDepth(xx, yy, zz);
                bool valid = volume.getWeight(xx, yy, zz) > 0;
                MC_vol.set(x, y, z, val, valid);
            }
        }
    }

    // extract the zero iso-surface using marching cubes
    SimpleMesh mesh;
    for (unsigned int x = 0; x < MC_vol.getDimX() - 1; x++)
    {
        std::cerr << "Marching Cubes on slice " << x << " of " << MC_vol.getDimX() << std::endl;

        for (unsigned int y = 0; y < MC_vol.getDimY() - 1; y++)
        {
            for (unsigned int z = 0; z < MC_vol.getDimZ() - 1; z++)
            {
                ProcessVolumeCell(&MC_vol, x, y, z, 0.00f, &mesh);
            }
        }
    }

    // write mesh to file
    std::string filenameOut("meshSample.off");
    if (!mesh.writeMesh(filenameOut))
    {
        std::cout << "ERROR: unable to write output file!" << std::endl;
        return -1;
    }
    // Direct MC on the model for the demonstration of the reconstruction only. Normally should use Raycasting on the model -> Mesh -> MC
    // ================================================


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

//	/*
//     * Process a first frame as a reference frame.
//     * --> All next frames are tracked relatively to the first frame.
//     */
//	sensor.processNextFrame();
//	const unsigned int depthWidth = sensor.getDepthImageWidth();
//	const unsigned int depthHeight = sensor.getDepthImageHeight();
//	PointCloud prevFrame;
//	float *depth_map = sensor.getDepth();			   // get the depth map
//	float *depth_map_filtered = sensor.getDepth(true); // get the filtered depth map
//
//	/* if you want to virtualize the depth map and fitered depth map ,please use following code */
//	 FreeImageU16F::SaveImageToFile(depth_map, "original.png", depthWidth, depthHeight, 1, true);
//	 FreeImageU16F::SaveImageToFile(depth_map_filtered, "filtered.png", depthWidth, depthHeight, 1, true);


//	// loop in our pipeline starts here
//	int i = 0;
//	while (i < MAX_FRAME_NUM && sensor.processNextFrame())
//	{
//		float *depth_map = sensor.getDepth(); // get the depth map
//
//		// TODO: createFrame
//		PointCloud currentFrame;
//
//		process_frame(i, prevFrame, currentFrame, volume);
//	}
	return 0;
}
