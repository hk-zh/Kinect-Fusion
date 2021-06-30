#include "TSDF.h"
#include <exception>
#include <iostream>

void get_current_info(VirtualSensor &sensor, Vector3f *vertex_current, Vector3f *normal_current)
{
    float *depthMap = sensor.getDepth();
    const Matrix3f &depthIntrinsics = sensor.getDepthIntrinsics();
    const Matrix4f &depthExtrinsics = sensor.getDepthExtrinsics();
    const unsigned height = sensor.getDepthImageHeight();
    const unsigned width = sensor.getDepthImageWidth();
    float fovX = depthIntrinsics(0, 0);
    float fovY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);
    float maxDistance = 8.0f; // we set max distance to 8;
    const float maxDistanceHalved = maxDistance / 2.f;

    // Compute inverse depth extrinstics
    Matrix4f depthExtrinsicsInv = depthExtrinsics.inverse();
    Matrix3f rotationInv = depthExtrinsicsInv.block(0, 0, 3, 3);
    Vector3f translationInv = depthExtrinsicsInv.block(0, 3, 3, 1);

#pragma omp parallel for
    for (int v = 0; v < (int)height; ++v)
    {
        // For every pixel in a row.
        for (int u = 0; u < (int)width; ++u)
        {
            unsigned int idx = v * width + u; // linearized index
            float depth = depthMap[idx];
            if (depth == MINF)
            {
                vertex_current[idx] = Vector3f(MINF, MINF, MINF);
            }
            else
            {
                // Back-projection to camera space.
                vertex_current[idx] = rotationInv * Vector3f((u - cX) / fovX * depth, (v - cY) / fovY * depth, depth) + translationInv;
            }
        }
    }
#pragma omp parallel for
    for (int v = 1; v < (int)(height - 1); ++v)
    {
        for (int u = 1; u < (int)(width - 1); ++u)
        {
            unsigned int idx = v * width + u; // linearized index

            const float du = 0.5f * (depthMap[idx + 1] - depthMap[idx - 1]);
            const float dv = 0.5f * (depthMap[idx + width] - depthMap[idx - width]);
            if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistanceHalved || abs(dv) > maxDistanceHalved)
            {
                normal_current[idx] = Vector3f(MINF, MINF, MINF);
                continue;
            }

            // TODO: Compute the normals using central differences.
            int idx_up = (v - 1) * width + u;
            int idx_down = (v + 1) * width + u;
            int idx_left = v * width + u - 1;
            int idx_right = v * width + u + 1;
            normal_current[idx] = (vertex_current[idx_right] - vertex_current[idx_left]).cross(vertex_current[idx_up] - vertex_current[idx_down]); // Needs to be replaced.
            normal_current[idx].normalize();
        }
    }
}

void TSDF::update(VirtualSensor &sensor, const Matrix4f &camera2world)
{
    Matrix4f world2camera = camera2world.inverse();
    Matrix3f rotation = world2camera.block<3, 3>(0, 0);
    Vector3f translation = world2camera.block<3, 1>(0, 3);

    Matrix3f intrinsics = sensor.getDepthIntrinsics();
    float focal_x = intrinsics(0, 0);
    float focal_y = intrinsics(1, 1);
    float principle_x = intrinsics(0, 2);
    float principle_y = intrinsics(1, 2);

    auto camera2pixel = [&focal_x, &focal_y, &principle_x, &principle_y](const Vector3f &point)
    {
        return Vector2i{std::floor(point.x() * focal_x / point.z() + principle_x),
                        std::floor(point.y() * focal_y / point.z() + principle_y)};
    };
    auto pixel2camera = [&focal_x, &focal_y, &principle_x, &principle_y](const Vector2i &point)
    {
        return Vector3f{(static_cast<float>(point.x()) - principle_x) / focal_x,
                        (static_cast<float>(point.y()) - principle_y) / focal_y,
                        1.0f};
    };

    for (int x = 0; x < volSz.x(); ++x)
    {
        for (int y = 0; y < volSz.y(); ++y)
        {
            for (int z = 0; z < volSz.z(); ++z)
            {
                Vector3f pos_in_world{(static_cast<float>(x) + 0.5f) * volUnit,
                                      (static_cast<float>(y) + 0.5f) * volUnit,
                                      (static_cast<float>(z) + 0.5f) * volUnit};
                Vector3f pos_in_camera = rotation * pos_in_world + translation;
                if (pos_in_camera.z() <= 0)
                {
                    continue;
                }

                Vector2i pos_in_pixel = camera2pixel(pos_in_camera);
                if (pos_in_pixel.x() < 0 || pos_in_pixel.x() >= sensor.getDepthImageWidth() || pos_in_pixel.y() < 0 || pos_in_pixel.y() >= sensor.getDepthImageHeight())
                {
                    continue;
                }

                float raw_depth = sensor.getDepth(pos_in_pixel.x(), pos_in_pixel.y(), false);
                if (raw_depth == MINF)
                {
                    continue;
                }

                const float cos_theta = 1.0f / pixel2camera(pos_in_pixel).norm();

                float untruncated = raw_depth - cos_theta * pos_in_camera.norm();
                untruncated /= truncation;
                if (untruncated < -1.0f)
                {
                    continue;
                }
                const float truncated = std::min(1.0f, untruncated);

                float old_tsdf = getDepth(x, y, z);
                float old_weight = getWeight(x, y, z);

                // simple average works fine according to the paper
                float new_weight = old_weight + 1;
                float new_tsdf = (old_tsdf * old_weight + truncated) / new_weight;

                new_weight = new_weight > MAX_WEIGHT ? MAX_WEIGHT : new_weight;
                setDepth(x, y, z, new_tsdf);
                setWeight(x, y, z, new_weight);
            }
        }
    }
}

void TSDF::raycast(VirtualSensor &sensor, const Matrix4f &camera2world, Vector3f *vertex_prediction, Vector3f *normal_prediction)
{
    Matrix3f rotation = camera2world.block<3, 3>(0, 0);
    Vector3f translation = camera2world.block<3, 1>(0, 3);

    Matrix3f intrinsics = sensor.getDepthIntrinsics();
    float focal_x = intrinsics(0, 0);
    float focal_y = intrinsics(1, 1);
    float principle_x = intrinsics(0, 2);
    float principle_y = intrinsics(1, 2);

    auto pixel2camera = [&focal_x, &focal_y, &principle_x, &principle_y](const Vector2i &point)
    {
        return Vector3f{(static_cast<float>(point.x()) - principle_x) / focal_x,
                        (static_cast<float>(point.y()) - principle_y) / focal_y,
                        1.0f};
    };

    unsigned width = sensor.getDepthImageWidth();
    unsigned height = sensor.getDepthImageHeight();
    auto locator = [&width, &height](Vector3f *base, size_t x, size_t y)
    {
        if (!(x >= 0 && y >= 0 && y < width && x < height))
        {
            std::cout << x << " " << y << " " << height << " " << width << "\n";
            throw std::exception();
        }
        return static_cast<Vector3f *>(base + x * width + y);
    };

    auto invalidator = [&locator, &vertex_prediction, &normal_prediction](size_t x, size_t y)
    {
        *locator(vertex_prediction, x, y) = Vector3f(MINF, MINF, MINF);
        *locator(normal_prediction, x, y) = Vector3f(MINF, MINF, MINF);
    };

    auto trilinear_interpolation = [this](const Vector3f &point)
    {
        float base_x = (std::floor(point.x()) + 0.5f);
        float base_y = (std::floor(point.y()) + 0.5f);
        float base_z = (std::floor(point.z()) + 0.5f);

        float ratio_x = point.x() - base_x;
        ratio_x += static_cast<float>(ratio_x < 0);
        float ratio_y = point.y() - base_y;
        ratio_y += static_cast<float>(ratio_y < 0);
        float ratio_z = point.z() - base_z;
        ratio_z += static_cast<float>(ratio_z < 0);

        return this->getDepth(static_cast<int>(base_x), static_cast<int>(base_y), static_cast<int>(base_z)) * (1 - ratio_x) * (1 - ratio_y) * (1 - ratio_z) +
               this->getDepth(static_cast<int>(base_x + 1), static_cast<int>(base_y), static_cast<int>(base_z)) * (ratio_x) * (1 - ratio_y) * (1 - ratio_z) +
               this->getDepth(static_cast<int>(base_x), static_cast<int>(base_y + 1), static_cast<int>(base_z)) * (1 - ratio_x) * (ratio_y) * (1 - ratio_z) +
               this->getDepth(static_cast<int>(base_x), static_cast<int>(base_y), static_cast<int>(base_z + 1)) * (1 - ratio_x) * (1 - ratio_y) * (ratio_z) +
               this->getDepth(static_cast<int>(base_x + 1), static_cast<int>(base_y + 1), static_cast<int>(base_z)) * (ratio_x) * (ratio_y) * (1 - ratio_z) +
               this->getDepth(static_cast<int>(base_x + 1), static_cast<int>(base_y), static_cast<int>(base_z + 1)) * (ratio_x) * (1 - ratio_y) * (ratio_z) +
               this->getDepth(static_cast<int>(base_x), static_cast<int>(base_y + 1), static_cast<int>(base_z + 1)) * (1 - ratio_x) * (ratio_y) * (ratio_z) +
               this->getDepth(static_cast<int>(base_x + 1), static_cast<int>(base_y + 1), static_cast<int>(base_z + 1)) * (ratio_x) * (ratio_y) * (ratio_z);
    };

    for (int x = 0; x < sensor.getDepthImageHeight(); ++x)
    {
        for (int y = 0; y < sensor.getDepthImageWidth(); ++y)
        {
            Vector3f unitRay = (rotation * pixel2camera(Vector2i(x, y))).normalized();

            float vol_max_x = static_cast<float>(volSz.x()) * volUnit;
            float vol_max_y = static_cast<float>(volSz.y()) * volUnit;
            float vol_max_z = static_cast<float>(volSz.z()) * volUnit;

            float ray_len_st_x = ((unitRay.x() > 0 ? 0 : vol_max_x) - translation.x()) / unitRay.x();
            float ray_len_st_y = ((unitRay.y() > 0 ? 0 : vol_max_y) - translation.y()) / unitRay.y();
            float ray_len_st_z = ((unitRay.z() > 0 ? 0 : vol_max_z) - translation.z()) / unitRay.z();
            float ray_len_st = std::max(0.0f, std::max(ray_len_st_x, std::max(ray_len_st_y, ray_len_st_z)));

            float ray_len_ed_x = ((unitRay.x() > 0 ? vol_max_x : 0) - translation.x()) / unitRay.x();
            float ray_len_ed_y = ((unitRay.y() > 0 ? vol_max_y : 0) - translation.y()) / unitRay.y();
            float ray_len_ed_z = ((unitRay.z() > 0 ? vol_max_z : 0) - translation.z()) / unitRay.z();
            float ray_len_ed = std::min(ray_len_ed_x, std::min(ray_len_ed_y, ray_len_ed_z));

            if (ray_len_ed <= ray_len_st)
            {
                invalidator(x, y);
                continue;
            }

            Vector3f grid = ((unitRay * (ray_len_st + volUnit)) + translation) / volUnit;
            float val = getDepth(static_cast<int>(grid.x()), static_cast<int>(grid.y()), static_cast<int>(grid.z()));
            float last_val;

            int loop_limit = static_cast<int>((ray_len_ed - ray_len_st) / (truncation / 2.0f));
            for (size_t i = 0; i <= loop_limit; ++i)
            {
                float ray_len = ray_len_st + volUnit + static_cast<float>(i) * (truncation / 2.0f);
                grid = ((unitRay * (ray_len + truncation / 2.0f)) + translation) / volUnit;

                if (grid.x() < 1 || grid.x() >= static_cast<float>(volSz.x()) - 1 ||
                    grid.y() < 1 || grid.y() >= static_cast<float>(volSz.y()) - 1 ||
                    grid.z() < 1 || grid.z() >= static_cast<float>(volSz.z()) - 1)
                {
                    continue;
                }
                last_val = val;
                val = getDepth(static_cast<int>(grid.x()), static_cast<int>(grid.y()), static_cast<int>(grid.z()));

                if (last_val < 0 && val > 0)
                {
                    invalidator(x, y);
                    break;
                }
                if (last_val > 0 && val < 0)
                {
                    float ray_len_interpolation = ray_len + truncation / 2.0f * (last_val / (val - last_val));
                    Vector3f vertex = unitRay * ray_len_interpolation + translation;
                    Vector3f grid_interpolation = vertex / volUnit;
                    if (grid_interpolation.x() < 1 || grid_interpolation.x() >= static_cast<float>(volSz.x()) - 1 ||
                        grid_interpolation.y() < 1 || grid_interpolation.y() >= static_cast<float>(volSz.y()) - 1 ||
                        grid_interpolation.z() < 1 || grid_interpolation.z() >= static_cast<float>(volSz.z()) - 1)
                    {
                        invalidator(x, y);
                        break;
                    }

                    Vector3f normal;
                    Vector3f neighbor = grid_interpolation;
                    --neighbor.x();
                    if (neighbor.x() < 0)
                    {
                        invalidator(x, y);
                        break;
                    }
                    float x_minus = trilinear_interpolation(neighbor);

                    neighbor = grid_interpolation;
                    ++neighbor.x();
                    if (neighbor.x() >= static_cast<float>(volSz.x()))
                    {
                        invalidator(x, y);
                        break;
                    }
                    float x_plus = trilinear_interpolation(neighbor);

                    normal.x() = (x_plus - x_minus);

                    neighbor = grid_interpolation;
                    --neighbor.y();
                    if (neighbor.y() < 0)
                    {
                        invalidator(x, y);
                        break;
                    }
                    float y_minus = trilinear_interpolation(neighbor);

                    neighbor = grid_interpolation;
                    ++neighbor.y();
                    if (neighbor.y() >= static_cast<float>(volSz.y()))
                    {
                        invalidator(x, y);
                        break;
                    }
                    float y_plus = trilinear_interpolation(neighbor);

                    normal.y() = (y_plus - y_minus);

                    neighbor = grid_interpolation;
                    --neighbor.z();
                    if (neighbor.z() < 0)
                    {
                        invalidator(x, y);
                        break;
                    }
                    float z_minus = trilinear_interpolation(neighbor);

                    neighbor = grid_interpolation;
                    ++neighbor.z();
                    if (neighbor.z() >= static_cast<float>(volSz.y()))
                    {
                        invalidator(x, y);
                        break;
                    }
                    float z_plus = trilinear_interpolation(neighbor);

                    normal.z() = (z_plus - z_minus);

                    if (normal.norm() == 0)
                    {
                        invalidator(x, y);
                        break;
                    }
                    normal.normalize();

                    *locator(vertex_prediction, x, y) = vertex;
                    *locator(normal_prediction, x, y) = normal;
                    break;
                }
                invalidator(x, y);
            }
        }
    }
}
