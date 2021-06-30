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

#define USE_LINEAR_ICP 0

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

    // ==========================================================
    // Configuration
    // unit: meter
    // parameters for the volume model
    Vector3i volSize(256, 256, 256);
    float volUnit = 0.004f;
    float truncation = 0.012f;
    float init_depth = 1.0f;

    TSDF volume(volSize, volUnit, truncation);

    Matrix4f currentPos;
    currentPos.setIdentity();
    currentPos(0, 3) = static_cast<float>(volume.volSz.x()) / 2.0f * volume.volUnit;
    currentPos(1, 3) = static_cast<float>(volume.volSz.y()) / 2.0f * volume.volUnit;
    currentPos(2, 3) = static_cast<float>(volume.volSz.z()) / 2.0f * volume.volUnit - init_depth;

    std::unique_ptr<Vector3f[]> vertex_prediction = std::make_unique<Vector3f[]>(sensor.getDepthImageWidth() * sensor.getDepthImageHeight());
    std::unique_ptr<Vector3f[]> normal_prediction = std::make_unique<Vector3f[]>(sensor.getDepthImageWidth() * sensor.getDepthImageHeight());
    std::unique_ptr<Vector3f[]> vertex_current = std::make_unique<Vector3f[]>(sensor.getDepthImageWidth() * sensor.getDepthImageHeight());
    std::unique_ptr<Vector3f[]> normal_current = std::make_unique<Vector3f[]>(sensor.getDepthImageWidth() * sensor.getDepthImageHeight());

    // std::vector<Vector3f> vertex_prediction;
    // std::vector<Vector3f> normal_prediction;
    // std::vector<Vector3f> vertex_current;
    // std::vector<Vector3f> normal_current;
    // ==========================================================
    // Set up optimizer
    ICPOptimizer *optimizer = nullptr;
    if (USE_LINEAR_ICP)
    {
        optimizer = new LinearICPOptimizer();
    }
    else
    {
        optimizer = new CeresICPOptimizer();
    }
    optimizer->setMatchingMaxDistance(0.1f);
    optimizer->usePointToPlaneConstraints(true);
    optimizer->setNbOfIterations(10);
    std::vector<Matrix4f> estimatedPoses; // be used to store all estimate poses
    // ==========================================================

    // ==========================================================
    // MAIN LOOP
    const int number_of_frames = 1;
    for (int i = 0; i < number_of_frames; ++i)
    {
        sensor.processNextFrame();
        volume.get_current_info(sensor, vertex_current.get(), vertex_current.get());
        if (i != 0)
        {
            // currentPos = optimizer->estimatePose(vertex_current.get(), normal_current.get(), vertex_prediction.get(), normal_prediction.get());
        }
        volume.update(sensor, currentPos);
        volume.raycast(sensor, currentPos, vertex_prediction.get(), normal_prediction.get());

        // Test Raycast
        auto vertexValidate = [](const Vector3f &v)
        {
            return v(0) != MINF;
        };

        auto edgeValidate = [](const Vector3f &a, const Vector3f &b, float threshold)
        {
            float x_diff = a(0) - b(0);
            float y_diff = a(1) - b(1);
            float z_diff = a(2) - b(2);

            return x_diff * x_diff + y_diff * y_diff + z_diff * z_diff <= threshold * threshold;
        };

        auto faceWrite = [&vertexValidate, &edgeValidate](std::stringstream &ss, size_t x, size_t y, size_t z, const Vector3f *v, float threshold)
        {
            if (!vertexValidate(v[x]) || !vertexValidate(v[y]) || !vertexValidate(v[z]))
            {
                return false;
            }
            if (!edgeValidate(v[x], v[y], threshold) || !edgeValidate(v[y], v[z], threshold) || !edgeValidate(v[z], v[x], threshold))
            {
                return false;
            }
            ss << 3 << " " << x << " " << y << " " << z << std::endl;
            return true;
        };

        auto meshWriter = [&vertexValidate, &faceWrite](Vector3f *vertices, Vector3f *normals, unsigned int width, unsigned int height)
        {
            float edgeThreshold = 0.01f; // 1cm

            unsigned int nVertices = width * height;
            unsigned nFaces = 0;

            std::ofstream outFile("raycastResult.off");
            if (!outFile.is_open())
                return false;

            std::stringstream ss;
            for (size_t i = 0; i < nVertices; ++i)
            {
                if (!vertexValidate(vertices[i]))
                {
                    ss << "0 0 0 0 0 0\n";
                    continue;
                }
                ss << vertices[i](0) << " " << vertices[i](1) << " " << vertices[i](2) << " "
                   << normals[i](0) << " " << normals[i](1) << " " << normals[i](2) << std::endl;
            }

            for (size_t i = 0; i < height - 1; ++i)
            {
                for (size_t j = 0; j < width - 1; ++j)
                {
                    nFaces += faceWrite(ss, i * width + j, (i + 1) * width + j, i * width + j + 1, vertices, edgeThreshold);
                    nFaces += faceWrite(ss, i * width + j + 1, (i + 1) * width + j, (i + 1) * width + j + 1, vertices, edgeThreshold);
                }
            }

            outFile << "NOFF" << std::endl;
            outFile << nVertices << " " << nFaces << " 0" << std::endl;
            outFile << ss.str();
            outFile.close();
            return true;
        };

        meshWriter(vertex_prediction.get(), normal_prediction.get(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight());
        // End test
    }
    // ==========================================================

    // ==========================================================
    // Direct MC on the model
    Volume MC_vol(Vector3d(-0.1, -0.1, -0.1), Vector3d(1.1, 1.1, 1.1), 200, 200, 200);

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
    // ==========================================================

    // ==========================================================
    // Check filter
    //    sensor.processNextFrame();
    //	float *depth_map = sensor.getDepth();			   // get the depth map
    //	float *depth_map_filtered = sensor.getDepth(true); // get the filtered depth map
    //
    //	/* if you want to virtualize the depth map and fitered depth map ,please use following code */
    //	 FreeImageU16F::SaveImageToFile(depth_map, "original.png", depthWidth, depthHeight, 1, true);
    //	 FreeImageU16F::SaveImageToFile(depth_map_filtered, "filtered.png", depthWidth, depthHeight, 1, true);
    // ==========================================================

    return 0;
}
