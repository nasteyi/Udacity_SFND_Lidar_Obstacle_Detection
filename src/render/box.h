#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry>

struct BoxQ {
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length{0};
    float cube_width{0};
    float cube_height{0};
};
struct Box {
    float x_min{0};
    float y_min{0};
    float z_min{0};
    float x_max{0};
    float y_max{0};
    float z_max{0};
};
#endif