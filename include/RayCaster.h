#pragma once

#include "Eigen.h"

class Raycast
{
public:
    bool surfacePrediction();

private:
    // TODO: add parameters
    Vector3d calculateRayDirection();
    bool calculatePointOnRay();
    Vector3d getVertexAtZeroCrossing();
    double getTSDFInterpolation();
    Vector3d calculateNormal();
};