#ifndef LIBSYNEXENS3_ROS_TYPES_H
#define LIBSYNEXENS3_ROS_TYPES_H

#include "libsynexens3/libsynexens3.h"
enum sy3_color_resolution_t
{
    SY3_COLOR_RESOLUTION_OFF,
    // SY3_COLOR_RESOLUTION_640x480P,
    SY3_COLOR_RESOLUTION_1920x1080P,
};

enum sy3_depth_resolution_t
{
    SY3_DEPTH_RESOLUTION_OFF,
    SY3_DEPTH_RESOLUTION_320x240P,
    SY3_DEPTH_RESOLUTION_640x480P,
};

struct sy3_config_mode_t
{
    bool enable_depth;
    bool enable_ir;
    bool enable_rgb;
    sy3_color_resolution_t rgb_mode;
    sy3_depth_resolution_t depth_mode;
};

template<class T>
struct Point3D
{
  T x;
  T y;
  T z;
};

using point3d_t = Point3D<double>;
using point3f_t = Point3D<float>;

#endif // LIBSYNEXENS3_ROS_TYPES_H