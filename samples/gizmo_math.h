#ifndef GIZMO_MATH_H
#define GIZMO_MATH_H

#include <cmath>
#include <algorithm>

#include <math/vec3.h>

namespace gizmo {

using namespace filament;
using namespace filament::math;



struct line3
{
    double3 origin;
    double3 direction;

    line3() { 
        origin = double3(0, 0, 0);
        direction = double3(0, 0, 1);
    }
    line3(const double3& originIn, const double3& directionIn) {
        origin = originIn;
        direction = directionIn;
    }

    inline double3 point_at(double parameter) const { 
        return origin + parameter * direction; 
    }

    inline double project(const double3& point) const { 
        return dot((point - origin), direction);
    }

    inline double3 nearest_point(const double3& point) const { 
        return point_at(project(point));
    }

    inline double distance(const double3& point) const {
        return distance(nearest_point(point) - point);
    }
};



struct ray3
{
    double3 origin;
    double3 direction;

    ray3() { 
        origin = double3(0, 0, 0);
        direction = double3(0, 0, 1);
    }
    ray3(const double3& originIn, const double3& directionIn) {
        origin = originIn;
        direction = directionIn;
    }

    inline double3 point_at(double parameter) const { 
        return origin + parameter * direction; 
    }

    inline double project(const double3& point) const { 
        return std::clamp( dot((point - origin), direction), 
            0.0, std::numeric_limits<double>::max());
    }

    inline double3 nearest_point(const double3& point) const { 
        return point_at(project(point));
    }

    inline double distance(const double3& point) const {
        return distance(nearest_point(point) - point);
    }
};

struct segment3 {
    double3 origin;
    double3 direction;
    double extent;

    segment3() { 
        origin = double3(0, 0, 0);
        direction = double3(1, 0, 0);
        extent = 1.0;
    }
    segment3(const double3& startPt, const double3& endPt)
    {
        origin = (endPt + startPt) * 0.5;
        direction = endPt - startPt;
        extent = length(direction) / 2.0;
        direction = normalize(direction);
    }

    inline double3 point_at(double parameter) const { 
        return origin + parameter * direction; 
    }

    inline double3 point_interp(double unit_parameter) const { 
        return origin + ((unit_parameter-0.5)*2.0) * extent * direction;
    }

    inline double project(const double3& point) const { 
        return std::clamp( dot((point - origin), direction), -extent, extent);
    }

    inline double3 nearest_point(const double3& point) const { 
        return point_at(project(point));
    }

    inline double distance(const double3& point) const {
        return distance(nearest_point(point) - point);
    }
};


struct plane3
{
    double3 normal;
    double d;

    plane3(double3 normalIn, double3 point)
    { 
        normal = normalIn;
        d = -dot(point, normal);
    }
};






inline double vector_angle_deg(const double3& dir0, const double3& dir1) {
    double d = std::clamp(dot(dir0, dir1), -1.0, 1.0);
    return std::acos(d) * math::d::RAD_TO_DEG;
}



inline double3 ray_plane_intersection(const ray3& ray, const plane3& plane) {
    double div = dot(ray.direction, plane.normal);
    // if (MathUtil.EpsilonEqual(div, 0, MathUtil.ZeroTolerance)) return Vector3d.Invalid;
    double ray_t = -(dot(ray.origin, plane.normal) + plane.d) / div;
    return ray.point_at(ray_t);
}




// based on WildMagic5 via geometry3Sharp
inline double ray_segment_distance_sqr(const ray3& ray, const segment3& segment,
    double& ray_param_t, double& segment_param_t)
{
    double3 diff = ray.origin - segment.origin;
    double a01 = -dot(ray.direction, segment.direction);
    double b0 = dot(diff, ray.direction);
    double b1 = dot(- diff, segment.direction);
    double c = length2(diff);
    double det = std::abs(1 - a01 * a01);
    double s0, s1, sqrDist, extDet;

    if (det >= std::numeric_limits<double>::epsilon() ) {
        // not parallel.
        s0 = a01 * b1 - b0;
        s1 = a01 * b0 - b1;
        extDet = segment.extent * det;

        if (s0 >= 0) {
            if (s1 >= -extDet) {
                if (s1 <= extDet) { // region 0
                    // Minimum at interior points of Ray and Segment.
                    double invDet = (1) / det;
                    s0 *= invDet;
                    s1 *= invDet;
                    sqrDist = s0 * (s0 + a01 * s1 + (2) * b0) + s1 * (a01 * s0 + s1 + (2) * b1) + c;
                } else { // region 1 
                    s1 = segment.extent;
                    s0 = -(a01 * s1 + b0);
                    if (s0 > 0) {
                        sqrDist = -s0 * s0 + s1 * (s1 + (2) * b1) + c;
                    } else {
                        s0 = 0;
                        sqrDist = s1 * (s1 + (2) * b1) + c;
                    }
                }
            } else { // region 5
                s1 = -segment.extent;
                s0 = -(a01 * s1 + b0);
                if (s0 > 0) {
                    sqrDist = -s0 * s0 + s1 * (s1 + (2) * b1) + c;
                } else {
                    s0 = 0;
                    sqrDist = s1 * (s1 + (2) * b1) + c;
                }
            }
        } else {
            if (s1 <= -extDet) { // region 4
                s0 = -(-a01 * segment.extent + b0);
                if (s0 > 0) {
                    s1 = -segment.extent;
                    sqrDist = -s0 * s0 + s1 * (s1 + (2) * b1) + c;
                } else {
                    s0 = 0;
                    s1 = -b1;
                    if (s1 < -segment.extent) {
                        s1 = -segment.extent;
                    } else if (s1 > segment.extent) {
                        s1 = segment.extent;
                    }
                    sqrDist = s1 * (s1 + (2) * b1) + c;
                }
            } else if (s1 <= extDet) { // region 3
                s0 = 0;
                s1 = -b1;
                if (s1 < -segment.extent) {
                    s1 = -segment.extent;
                } else if (s1 > segment.extent) {
                    s1 = segment.extent;
                }
                sqrDist = s1 * (s1 + (2) * b1) + c;
            } else { // region 2
                s0 = -(a01 * segment.extent + b0);
                if (s0 > 0) {
                    s1 = segment.extent;
                    sqrDist = -s0 * s0 + s1 * (s1 + (2) * b1) + c;
                } else {
                    s0 = 0;
                    s1 = -b1;
                    if (s1 < -segment.extent) {
                        s1 = -segment.extent;
                    } else if (s1 > segment.extent) {
                        s1 = segment.extent;
                    }
                    sqrDist = s1 * (s1 + (2) * b1) + c;
                }
            }
        }
    } else {
        // Ray and Segment are parallel.
        if (a01 > 0) {
            s1 = -segment.extent;   // Opposite direction vectors.
        } else {
            s1 = segment.extent;    // Same direction vectors.
        }

        s0 = -(a01 * s1 + b0);
        if (s0 > 0) {
            sqrDist = -s0 * s0 + s1 * (s1 + (2) * b1) + c;
        } else {
            s0 = 0;
            sqrDist = s1 * (s1 + (2) * b1) + c;
        }
    }

    ray_param_t = s0;
    segment_param_t = s1;

    if (sqrDist < 0)    // account for numerical round-off errors.
        sqrDist = 0;
    return sqrDist;
}

inline double ray_line_distance_sqr(const ray3& ray, const line3& line,
    double& ray_param_t, double& line_param_t)
{
    double3 diff = line.origin - ray.origin;
    double a01 = -dot(line.direction, ray.direction);
    double b0 = dot(diff, line.direction);
    double c = length2(diff);
    double det = std::abs(1.0 - a01 * a01);
    double b1, s0, s1, sqrDist;

    if (det >= std::numeric_limits<double>::epsilon() ) {
        b1 = -dot(diff,ray.direction);
        s1 = a01 * b0 - b1;

        if (s1 >= (double)0) {
            // Two interior points are closest, one on line and one on ray.
            double invDet = ((double)1) / det;
            s0 = (a01 * b1 - b0) * invDet;
            s1 *= invDet;
            sqrDist = s0 * (s0 + a01 * s1 + ((double)2) * b0) +
                s1 * (a01 * s0 + s1 + ((double)2) * b1) + c;
        } else {
            // Origin of ray and interior point of line are closest.
            s0 = -b0;
            s1 = (double)0;
            sqrDist = b0 * s0 + c;
        }
    } else {
        // Lines are parallel, closest pair with one point at ray origin.
        s0 = -b0;
        s1 = (double)0;
        sqrDist = b0 * s0 + c;
    }

    line_param_t = s0;
    ray_param_t = s1;

    // Account for numerical round-off errors.
    if (sqrDist < (double)0) {
        sqrDist = (double)0;
    }
    return sqrDist;
}



inline filament::Box compute_pointset_bounds(const float* positions, int count,
        int stride_in_bytes) {
    int stride = stride_in_bytes / sizeof(float);
    filament::math::float3 boxmin, boxmax;
    boxmin = boxmax = float3(positions[0], positions[1], positions[2]);
    int cur_idx = stride;
    for (int k = 1; k < count; ++k) {
        float3 pos(positions[cur_idx], positions[cur_idx + 1], positions[cur_idx + 2]);
        cur_idx += stride;
        for (int j = 0; j < 3; ++j) {
            boxmin[j] = std::min(boxmin[j], pos[j]);
            boxmax[j] = std::max(boxmin[j], pos[j]);
        }
    }
    filament::Box bounds;
    bounds.set(boxmin, boxmax);
    return bounds;
}




} // namespace gizmo

#endif
