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

#define USE_LINEAR_ICP 1
#define SAMPEL_FREQUENCE 8

void sample(std::vector<Vector3f> a1, Vector3f *b1, std::vector<Vector3f> a2, Vector3f *b2, unsigned int num_pixels)
{
    a1.reserve(std::floor((float)num_pixels / SAMPEL_FREQUENCE) + 1);
    a2.reserve(std::floor((float)num_pixels / SAMPEL_FREQUENCE) + 1);
    for (int i = 0; i < num_pixels; i = i + SAMPEL_FREQUENCE)
    {
        if (b1[i].allFinite() && b2[i].allFinite())
        {
            a1.push_back(b1[i]);
            a2.push_back(b2[i]);
        }
    }
}

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
    //volUnit = 0.006f
    //volUnit = 0.0045 core dumped
    //volUnit = 0.0050f core dumped
    //volUnit = 0.0058f worked but improve more
    //volUnit = 0.0070f worked more fine than 0.0058 --> best for coulur
    //volUnit = 0.0075f worked more fine than 0.0070
    //volUnit = 0.0080f core dumped 
    //volUnit = 0.0090f core dumped
    //volUnit = 0.0090f + TSDF 95 changed volsZ from volSz.x() to volSz.x()-5 it worked fine than 0.0070 --> worked fine but coulur changed inerestingly
    //volUnit = 0.01f worked fine
    //volUnit = 0.02f not worked well

    Vector3i volSize(256, 256, 256);
    float volUnit = 0.005f;
    float truncation = 0.012f;
    float init_depth = 1.0f;

    TSDF volume(volSize, volUnit, truncation);

    Matrix4f currentPos;
    currentPos.setIdentity();
    currentPos(0, 3) = static_cast<float>(volume.volSz.x()) / 2.0f * volume.volUnit;
    currentPos(1, 3) = static_cast<float>(volume.volSz.y()) / 2.0f * volume.volUnit;
    currentPos(2, 3) = static_cast<float>(volume.volSz.z()) / 2.0f * volume.volUnit - init_depth;

    unsigned int num_pixels = sensor.getDepthImageWidth() * sensor.getDepthImageHeight();
    std::unique_ptr<Vector3f[]> vertex_prediction = std::make_unique<Vector3f[]>(num_pixels);
    std::unique_ptr<Vector3f[]> normal_prediction = std::make_unique<Vector3f[]>(num_pixels);
    std::unique_ptr<Vector3f[]> vertex_current = std::make_unique<Vector3f[]>(num_pixels);
    std::unique_ptr<Vector3f[]> normal_current = std::make_unique<Vector3f[]>(num_pixels);

    std::vector<Vector3f> vertex_prediction_v;
    std::vector<Vector3f> normal_prediction_v;
    std::vector<Vector3f> vertex_current_v;
    std::vector<Vector3f> normal_current_v;
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
    const int number_of_frames = 10;
    for (int i = 0; i < number_of_frames; ++i)
    {
        sensor.processNextFrame();
        
        //get the current values of vertex_current and normal_current to save as a file in the project folder
        // If there is no get current infor nothing changed
        volume.get_current_info(sensor, currentPos, vertex_current.get(), normal_current.get());
        if (i != 0)
        {
            sample(vertex_prediction_v, vertex_prediction.get(), normal_prediction_v, normal_prediction.get(), num_pixels);
            sample(vertex_current_v, vertex_current.get(), normal_current_v, normal_current.get(), num_pixels);

            currentPos = optimizer->estimatePose(vertex_current_v, normal_current_v, vertex_prediction_v, normal_prediction_v, currentPos);
        }

        //update the weight and height of new tsdf
        // If there is no update everythings are black
        volume.update(sensor, currentPos);
        //raycasting and save the result into vertex_prediction and normal_prediction
        //TODO: fix the raycast method there is some noisy?
        volume.raycast(sensor, currentPos, vertex_prediction.get(), normal_prediction.get());

        FreeImageB::SaveImageToFile(normal_current.get(), "normal_current.png", sensor.getColorImageWidth(), sensor.getColorImageHeight(), false);
        FreeImageB::SaveImageToFile(normal_prediction.get(), "normal_prediction.png", sensor.getColorImageWidth(), sensor.getColorImageHeight(), false);

        FreeImageB::SaveImageToFile(vertex_current.get(), "vertex_current.png", sensor.getColorImageWidth(), sensor.getColorImageHeight(), false);
        FreeImageB::SaveImageToFile(vertex_prediction.get(), "vertex_prediction.png", sensor.getColorImageWidth(), sensor.getColorImageHeight(), false);
        
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

    //==========================================================
    // Depth width is 640?
     int depthWidth = 640;
     int depthHeight = 480;

    //
    //Check filter
    sensor.processNextFrame();
    float *depth_map = sensor.getDepth();			   // get the depth map
    float *depth_map_filtered = sensor.getDepth(true); // get the filtered depth map
    
    /* if you want to virtualize the depth map and fitered depth map ,please use following code */
    FreeImageU16F::SaveImageToFile(depth_map, "original.png", depthWidth, depthHeight, 1, true);
    FreeImageU16F::SaveImageToFile(depth_map_filtered, "filtered.png", depthWidth, depthHeight, 1, true);
    //==========================================================

    return 0;
}
