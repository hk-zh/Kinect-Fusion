#ifndef KINECT_FUSION_TSDFVOLUMN_H
#define KINECT_FUSION_TSDFVOLUMN_H

#include "Eigen.h"
#include "VirtualSensor.h"
#include <memory>
#include <cassert>

class TsdfVolumn {
    static const int MAX_WEIGHT = 120; // paying more attention to current frames when more than 4s of frames are seen

    Vector3i volSz;
    float volUnit;
    const float truncation;
    std::unique_ptr<float[]> vol;
    std::unique_ptr<float[]> weight;

    float getDepth(int x, int y, int z) const {
        return vol[z * volSz.y() * volSz.x() + y * volSz.x() + x];
    }
    void setDepth(int x, int y, int z, float v) {
        assert(x < volSz.x() && x >= 0 && y < volSz.y() && y >= 0 && z < volSz.z() && z >= 0);
        assert(abs(v) <= 1);
        vol[z * volSz.y() * volSz.x() + y * volSz.x() + x] = v;
    }

    float getWeight(int x, int y, int z) const {
        return weight[z * volSz.y() * volSz.x() + y * volSz.x() + x];
    }
    void setWeight(int x, int y, int z, float v) {
        assert(x < volSz.x() && x >= 0 && y < volSz.y() && y >= 0 && z < volSz.z() && z >= 0);
        assert(abs(v) <= 1);
        weight[z * volSz.y() * volSz.x() + y * volSz.x() + x] = v;
    }

public:

    TsdfVolumn(const Vector3i& volSz, float volUnit, float truncation): volSz(volSz), volUnit(volUnit), truncation(truncation) {
        vol = std::make_unique<float[]>(volSz.x() * volSz.y() * volSz.z());
        weight = std::make_unique<float[]>(volSz.x() * volSz.y() * volSz.z());
    }

    void update(VirtualSensor sensor, const Matrix4f& camera2world);
};


#endif //KINECT_FUSION_TSDFVOLUMN_H
