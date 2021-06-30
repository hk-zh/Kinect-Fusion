#ifndef KINECT_FUSION_TSDF_H
#define KINECT_FUSION_TSDF_H

#include "Eigen.h"
#include "VirtualSensor.h"
#include <memory>
#include <cassert>

class TSDF
{
    static const int MAX_WEIGHT = 120; // paying more attention to current frames when more than 4s of frames are seen

    std::unique_ptr<float[]> vol;
    std::unique_ptr<float[]> weight;

    void setDepth(int x, int y, int z, float v)
    {
        assert(x < volSz.x() && x >= 0 && y < volSz.y() && y >= 0 && z < volSz.z() && z >= 0);
        assert(abs(v) <= 1);
        vol[z * volSz.y() * volSz.x() + y * volSz.x() + x] = v;
    }

    void setWeight(int x, int y, int z, float v)
    {
        assert(x < volSz.x() && x >= 0 && y < volSz.y() && y >= 0 && z < volSz.z() && z >= 0);
        assert(abs(v) <= 1);
        weight[z * volSz.y() * volSz.x() + y * volSz.x() + x] = v;
    }

public:
    const Vector3i volSz;
    const float volUnit;
    const float truncation;

    TSDF(const Vector3i &volSz, float volUnit, float truncation) : volSz(volSz), volUnit(volUnit), truncation(truncation)
    {
        vol = std::make_unique<float[]>(volSz.x() * volSz.y() * volSz.z());
        weight = std::make_unique<float[]>(volSz.x() * volSz.y() * volSz.z());
    }

    void update(VirtualSensor &sensor, const Matrix4f &camera2world);

    void raycast(VirtualSensor &sensor, const Matrix4f &camera2world, Vector3f *vertex_prediction, Vector3f *normal_prediction);

    void get_current_info(VirtualSensor &sensor, Vector3f *vertex_current, Vector3f *normal_current);

    float getDepth(int x, int y, int z) const
    {
        assert(x < volSz.x() && x >= 0 && y < volSz.y() && y >= 0 && z < volSz.z() && z >= 0);
        return vol[z * volSz.y() * volSz.x() + y * volSz.x() + x];
    }

    float getWeight(int x, int y, int z) const
    {
        assert(x < volSz.x() && x >= 0 && y < volSz.y() && y >= 0 && z < volSz.z() && z >= 0);
        return weight[z * volSz.y() * volSz.x() + y * volSz.x() + x];
    }
};

#endif //KINECT_FUSION_TSDF_H
