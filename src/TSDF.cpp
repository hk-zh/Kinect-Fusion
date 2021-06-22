#include "TSDF.h"

void TSDF::update(VirtualSensor& sensor, const Matrix4f& camera2world) {
    Matrix4f world2camera = camera2world.inverse();
    Matrix3f rotation = world2camera.block<3, 3>(0, 0);
    Vector3f translation = world2camera.block<3, 1>(0, 3);

    Matrix3f intrinsics = sensor.getDepthIntrinsics();
    float focal_x = intrinsics(0, 0);
    float focal_y = intrinsics(1, 1);
    float principle_x = intrinsics(0, 2);
    float principle_y = intrinsics(1, 2);

    auto camera2pixel = [&focal_x, &focal_y, &principle_x, &principle_y](const Vector3f& point) {
        return Vector2i{std::floor(point.x() * focal_x / point.z() + principle_x),
                        std::floor(point.y() * focal_y / point.z() + principle_y)
        };
    };
    auto pixel2camera = [&focal_x, &focal_y, &principle_x, &principle_y](const Vector2i& point) {
        return Vector3f{(static_cast<float>(point.x()) - principle_x) / focal_x,
                        (static_cast<float>(point.y()) - principle_y) / focal_y,
                        1.0f
        };
    };

    for (int x = 0; x < volSz.x(); ++x) {
        for (int y = 0; y < volSz.y(); ++y) {
            for (int z = 0; z < volSz.z(); ++z) {
                Vector3f pos_in_world {(static_cast<float>(x) + 0.5f) * volUnit,
                                       (static_cast<float>(y) + 0.5f) * volUnit,
                                       (static_cast<float>(z) + 0.5f) * volUnit};
                Vector3f pos_in_camera = rotation * pos_in_world + translation;
                if (pos_in_camera.z() <= 0) {
                    continue;
                }

                Vector2i pos_in_pixel = camera2pixel(pos_in_camera);
                if (pos_in_pixel.x() < 0 || pos_in_pixel.x() >= sensor.getDepthImageWidth()
                || pos_in_pixel.y() < 0 || pos_in_pixel.y() >= sensor.getDepthImageHeight()) {
                    continue;
                }

                float raw_depth = sensor.getDepth(pos_in_pixel.x(), pos_in_pixel.y(), false);
                if (raw_depth == MINF) {
                    continue;
                }

                const float cos_theta = 1.0f / pixel2camera(pos_in_pixel).norm();

                float untruncated = raw_depth - cos_theta * pos_in_camera.norm();
                untruncated /= truncation;
                if (untruncated < -1.0f) {
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
