#ifndef GIZMO_MATH_H
#define GIZMO_MATH_H

#include <cmath>
#include <algorithm>

#include <math/vec3.h>
#include <filament/Box.h>

namespace gizmo {

using namespace filament;
using namespace filament::math;


//
// Necessary math types and functions for implementing well-behaved gizmos
// Currently depends on filament double2, double3 and quat, and related operations
//


struct line3
{
    double3 origin;
    double3 direction;

    inline line3() { 
        origin = double3(0, 0, 0);
        direction = double3(0, 0, 1);
    }
    inline line3(const double3& originIn, const double3& directionIn) {
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

    inline ray3() { 
        origin = double3(0, 0, 0);
        direction = double3(0, 0, 1);
    }
    inline ray3(const double3& originIn, const double3& directionIn) {
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

    inline segment3() { 
        origin = double3(0, 0, 0);
        direction = double3(1, 0, 0);
        extent = 1.0;
    }
    inline segment3(const double3& startPt, const double3& endPt)
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

    inline plane3(double3 normalIn, double3 point)
    { 
        normal = normalize(normalIn);
        d = -dot(point, normal);
    }
};


struct frame3
{
    double3 origin;
    quat orientation;

    inline frame3(double3 originIn, quat orientationIn)
    { 
        origin = originIn;
        orientation = orientationIn;
    }
    inline frame3(double3 originIn, double3 axisZ) {
        origin = originIn;
        orientation = quat::fromDirectedRotation(double3(0, 0, 1), axisZ);
    }

    inline double3 AxisX() const { return orientation * double3(1, 0, 0); }
    inline double3 AxisY() const { return orientation * double3(0, 1, 0); }
    inline double3 AxisZ() const { return orientation * double3(0, 0, 1); }
    inline double3 Axis(int index) const {
        double3 n(0, 0, 0);
        n[index] = 1;
        return orientation * n;
    }

    inline double3 project(double3 point, int axis) const { 
        double3 n = Axis(axis);
        double t = dot((point - origin), n);
        return point - t * n;
    }

    inline double2 project2(double3 point, int axis) const { 
        double3 l = project(point, axis) - origin;
        if (axis == 0) {
            return double2( dot(l, Axis(1)), dot(l, Axis(2)) );
        } else if (axis == 1) {
            return double2(dot(l, Axis(0)), dot(l, Axis(2)));
        } else {
            return double2(dot(l, Axis(0)), dot(l, Axis(1)));
        }
    }

    inline double3 planeIntersection(ray3 ray, int axis) const;
};



inline double vector_angle_deg(const double3& dir0, const double3& dir1) {
    double d = std::clamp(dot(dir0, dir1), -1.0, 1.0);
    return std::acos(d) * math::d::RAD_TO_DEG;
}


// Returns two vectors perpendicular to n, as efficiently as possible.
// Duff et al method, from https://graphics.pixar.com/library/OrthonormalB/paper.pdf
inline void make_perp_vectors(double3 n, double3& b1, double3& b2) {
    if (n.z < 0.0) {
        double a = 1.0 / (1.0 - n.z);
        double b = n.x * n.y * a;
        b1 = double3(1.0 - n.x * n.x * a, -b, n.x);
        b2 = double3(b, n.y * n.y * a - 1.0, -n.y);
    } else {
        double a = 1.0 / (1.0 + n.z);
        double b = -n.x * n.y * a;
        b1 = double3(1.0 - n.x * n.x * a, b, -n.x);
        b2 = double3(b, 1.0 - n.y * n.y * a, -n.y);
    }
}



inline double3 ray_plane_intersection(const ray3& ray, const plane3& plane) {
    double div = dot(ray.direction, plane.normal);
    // if (MathUtil.EpsilonEqual(div, 0, MathUtil.ZeroTolerance)) return Vector3d.Invalid;
    double ray_t = -(dot(ray.origin, plane.normal) + plane.d) / div;
    return ray.point_at(ray_t);
}

double3 frame3::planeIntersection(ray3 ray, int axis) const {
    plane3 plane(Axis(axis), origin);
    return ray_plane_intersection(ray, plane);
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



inline double ray_circle_angleRad(const ray3& ray, const frame3& circleFrame) 
{
    double3 plane_pos = circleFrame.planeIntersection(ray, 2);
    double2 plane_uv = circleFrame.project2(plane_pos, 2);
    double angleRad = std::atan2(plane_uv.y, plane_uv.x);
    return angleRad;
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
