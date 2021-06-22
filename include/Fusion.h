#pragma once
#include "Volume.h"
#include "PointCloud.h"
class Fusion
{
public:
    bool reconstructSurface(const PointCloud &currentFrame, const Volume &volume);

private:
    double Fusion::calculateLamdas(Eigen::Vector2i &cameraSpacePoint, Eigen::Matrix3d intrinsics);
    double Fusion::calculateSDF(double &lambda, Eigen::Vector3d &cameraPosition, double rawDepthValue);
};