#ifndef GIZMO_H
#define GIZMO_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat4.h>


#include "gizmo_math.h"


namespace gizmo {


enum EGizmoElementType
{
    LineSegment = 0,
    Circle = 1
};


enum EGizmoElement
{
    None        = 0,
    TranslateX  = 1,
    TranslateY  = 2,
    TranslateZ  = 3,
    RotateAroundX = 4,
    RotateAroundY = 5,
    RotateAroundZ = 6,
};


class GizmoConstants
{
public:
    GizmoConstants() = delete;
    
    // colors are ARGB
    // todo should be video-white, not rgb-white  (and srgb?)
    static constexpr uint32_t GizmoWhite = 0xff000000;
    static constexpr uint32_t GizmoRed = 0xff0000ff;
    static constexpr uint32_t GizmoGreen = 0xff00ff00;
    static constexpr uint32_t GizmoBlue = 0xffff0000;

    static constexpr uint32_t GizmoHoverColor = 0xff00ffff;

    static constexpr double GizmoVisualAngleFOVFraction = 0.075;

    // hit-testing threshold, measured in visual angle.
    // ie the visual angle between the eye-ray and the ray to the gizmo nearest/hit point is
    // measured and must be less than this value. Using visual angle instead of pixel-distance has
    // some benefits (ie works in VR, DPI independent, etc) and some trade-offs (FOV and resolution-dependent)
    // TODO figure out a way to make this resolution-independent...or switch to pixel-threshold
    static constexpr double HitTestVisualAngleThreshDeg = 0.5;
};


struct GizmoHitResult {
    int hit_identifier = (int) EGizmoElement::None;
    double hit_distance = std::numeric_limits<double>::max();
    double hit_ray_param = std::numeric_limits<double>::max();
};


// predecls for GizmoBindings definions
class BaseGizmo;
class GizmoElement;


// state information used during gizmo capture/udpate
// (not all fields are used for all gizmo types)
struct GizmoCaptureState {
    BaseGizmo* gizmo = nullptr;
    GizmoElement* capturingElement = nullptr;

    mat4 start_transform;
    double3 start_origin;
    quat start_orientation;
    line3 world_line;
    double start_param = 0;

    void initialize(BaseGizmo* gizmoIn, GizmoElement* elementIn) {
        gizmo = gizmoIn;
        capturingElement = elementIn;
    }

    bool is_valid() const { return gizmo != nullptr && capturingElement != nullptr; }

    void reset() { 
        gizmo = nullptr;
        capturingElement = nullptr; 
    }
};



struct GizmoBindings
{
    std::function<void(double3 initialParams, GizmoCaptureState& captureState)>
        onBeginChange;

    std::function<void(double3 paramDeltas, GizmoCaptureState& captureState)> 
        onParameterUpdate;

    std::function<void(GizmoCaptureState& captureState)>
        onEndChange;

    GizmoBindings() 
    {
        onBeginChange = [](double3, GizmoCaptureState&) {};
        onParameterUpdate = [](double3, GizmoCaptureState&) {};
        onEndChange = [](GizmoCaptureState&) {};
    }
};

// GizmoElement is a part of a gizmo, line an axis-widget or circle-widget in a TRS gizmo.
// Not all fields will be used for all gizmo types
class GizmoElement
{
public:
    EGizmoElementType type = EGizmoElementType::LineSegment;
    int identifier = 0;
    double3 axis = double3(1, 0, 0);
    double4 params = double4(0, 1, 0, 0);
    bool bIsWorldSpace = false;

    uint32_t color = GizmoConstants::GizmoWhite;

    // bindings for behavior of this element
    GizmoBindings bindings;


    inline static GizmoElement MakeLineElement(int identifierIn, double3 axisIn, double2 extentsIn,
            uint32_t colorIn)
    {
        GizmoElement e;
        e.type = EGizmoElementType::LineSegment;
        e.identifier = identifierIn;
        e.axis = axisIn;
        e.params = double4(extentsIn.x, extentsIn.y, 0, 0);
        e.color = colorIn;
        return e;
    }

    inline static GizmoElement MakeCircleElement(int identifierIn, double3 axisIn, double radiusIn,
            uint32_t colorIn)
    {
        GizmoElement e;
        e.type = EGizmoElementType::Circle;
        e.identifier = identifierIn;
        e.axis = axisIn;
        e.params = double4(radiusIn, 0, 0, 0);
        e.color = colorIn;
        return e;
    }

};


// BaseGizmo is a collection of GizmoElements with a 3D frame
class BaseGizmo {
public:
    std::vector<GizmoElement> Elements;

    int hoveredIdentifier = 0;

    // 3D frame, ie location and rotation
    // GizmoElement fields are generally in local space, interpreted relative to this frame
    double3 origin = double3(0, 0, 0);
    quat orientation = quat(1, 0,0,0);

    // local-space GizmoElement dimensions are scaled by this amount in world space.
    // Mainly intended to be used to dynamically resize the gizmo based on projected area
    // (eg maintain constant size as the user zooms in and out)
    double view_scale = 1.0f;

public:

    GizmoElement* findElement(int elementID) {
        for (GizmoElement& elem : Elements) {
            if ( elem.identifier == elementID )
                return &elem;
        }
        return nullptr;
    }

    inline mat4 get_view_transform() const
    { 
        mat3 rotation(orientation);
        mat3 scale(view_scale);
        return mat4{ scale*rotation, origin };
    }

    inline mat4 get_frame_transform() const
    {
        mat3 rotation(orientation);
        return mat4{ rotation, origin };
    }
};










//////////////////////
// Hit Testing
//////////////////////




inline GizmoHitResult gizmo_hit_test(BaseGizmo& gizmo, const ray3& hit_ray) 
{
    mat4 transform = gizmo.get_frame_transform();
    double4 world_origin = transform * double4(0, 0, 0, 1);

    int hit_element = -1;
    double nearest_dist_sqr = std::numeric_limits<double>::max();
    double hit_ray_param = 0;

    // todo this can be factored into parts and the functions could be external

    auto test_line_element = [&](const GizmoElement& LineElement) {
        double3 a = gizmo.view_scale * (LineElement.params.x * LineElement.axis);
        double3 b = gizmo.view_scale * (LineElement.params.y * LineElement.axis);
        double4 seg_start = transform * double4(a.x, a.y, a.z, 1);
        double4 seg_end = transform * double4(b.x, b.y, b.z, 1);

        segment3 world_seg(seg_start.xyz, seg_end.xyz);
        double ray_param_t = 0, seg_param_t = 0;
        double distsqr = ray_segment_distance_sqr(hit_ray, world_seg, ray_param_t, seg_param_t);
        if (distsqr > nearest_dist_sqr) {
            return;
        }

        double3 nearest_dir = normalize(world_seg.point_at(seg_param_t) - hit_ray.origin);
        double visual_angle = vector_angle_deg(hit_ray.direction, nearest_dir);
        if (visual_angle > GizmoConstants::HitTestVisualAngleThreshDeg) {
            return;
        }

        if (hit_ray_param < ray_param_t) {
            hit_ray_param = ray_param_t;
            hit_element = LineElement.identifier;
            nearest_dist_sqr = distsqr;
        }
    };


    auto test_circle_element = [&](const GizmoElement& CircleElement) {
        double4 origin = transform * double4(0, 0, 0, 1);
        double4 normal = transform * double4(CircleElement.axis.x, CircleElement.axis.y,
                                             CircleElement.axis.z, 0.0);
        plane3 hitPlane(normal.xyz, origin.xyz);

        double3 hit_pt = ray_plane_intersection(hit_ray, hitPlane);
        // TODO check for parallel...

        double local_radius = CircleElement.params.x;
        double world_radius = gizmo.view_scale * local_radius;
        double hit_dist = length(hit_pt - origin.xyz) - world_radius;
        if (hit_dist * hit_dist > nearest_dist_sqr) {
            return;
        }

        double3 circle_pt = origin.xyz + world_radius * normalize(hit_pt - origin.xyz);
        double ray_param_t = hit_ray.project(circle_pt);

        double3 nearest_dir = normalize(circle_pt - hit_ray.origin);
        double visual_angle = vector_angle_deg(hit_ray.direction, nearest_dir);
        if (visual_angle > GizmoConstants::HitTestVisualAngleThreshDeg) {
            return;
        }

        if (hit_ray_param < ray_param_t) {
            hit_ray_param = ray_param_t;
            hit_element = CircleElement.identifier;
            nearest_dist_sqr = hit_dist * hit_dist;
        }     
    };

    for (const GizmoElement& Element : gizmo.Elements)
    {
        if (Element.type == EGizmoElementType::LineSegment) {
            test_line_element(Element);
        } else if (Element.type == EGizmoElementType::Circle) {
            test_circle_element(Element);
        }
    }


    if (hit_element >= 0) {
        return GizmoHitResult{ hit_element, std::sqrt(nearest_dist_sqr), hit_ray_param };
    } else {
        return GizmoHitResult();
    }
}



inline void begin_element_capture(const ray3& hit_ray, double hit_ray_param,
        GizmoCaptureState& captureState) 
{
    if (captureState.is_valid() == false) return;
    GizmoElement& element = *captureState.capturingElement;
    BaseGizmo& gizmo = *captureState.gizmo;

    mat4 transform = gizmo.get_frame_transform();
    double4 world_origin = transform * double4(0, 0, 0, 1);

    double3 start_params = double3(0, 0, 0);
    if (element.type == EGizmoElementType::LineSegment) 
    {
        double4 local_axis(element.axis, 0.0);
        double4 world_axis = normalize(transform * local_axis);
        captureState.world_line = line3(world_origin.xyz, world_axis.xyz);
        captureState.start_param = captureState.world_line.project(hit_ray.point_at(hit_ray_param));
        start_params = double3(captureState.start_param, 0, 0);
    } 
    else if (element.type == EGizmoElementType::Circle)
    {
        double4 local_axis(element.axis, 0.0);
        double4 world_axis = normalize(transform * local_axis);
        frame3 worldFrame(world_origin.xyz, world_axis.xyz);
        captureState.world_line = line3(world_origin.xyz, world_axis.xyz);
        captureState.start_param = ray_circle_angleRad(hit_ray, worldFrame);
        start_params = double3(captureState.start_param, 0, 0);
    }

    captureState.start_transform = transform;
    captureState.start_origin = gizmo.origin;
    captureState.start_orientation = gizmo.orientation;

    element.bindings.onBeginChange(start_params, captureState);
}

inline void update_element_capture(
    const ray3& update_ray, 
    GizmoCaptureState& captureState) 
{
    if (captureState.is_valid() == false) return;
    GizmoElement& element = *captureState.capturingElement;
    BaseGizmo& gizmo = *captureState.gizmo;

    double3 params_delta = double3(0, 0, 0);
    if (element.type == EGizmoElementType::LineSegment) {
        double ray_param_t, line_param_t;
        double distsqr =
                ray_line_distance_sqr(update_ray, captureState.world_line, ray_param_t, line_param_t);
        double delta_param = line_param_t - captureState.start_param;
        params_delta = double3(delta_param, 0, 0);
    } 
    else if (element.type == EGizmoElementType::Circle) 
    {
        frame3 worldFrame(captureState.world_line.origin, 
                quat::fromDirectedRotation(double3(0, 0, 1), captureState.world_line.direction));
        double newAngleRad = ray_circle_angleRad(update_ray, worldFrame);
        double delta_angle = newAngleRad - captureState.start_param;
        params_delta = double3(delta_angle, 0, 0);
    }

    element.bindings.onParameterUpdate(params_delta, captureState);
}


inline void end_element_capture(GizmoCaptureState& captureState) 
{
    if (captureState.is_valid() == false) return;
    GizmoElement& element = *captureState.capturingElement;
    element.bindings.onEndChange(captureState);
}


// standard TRS gizmo parameter update functions, to use for binding

static void axis_translation_update(double3 paramDeltas, GizmoCaptureState& captureState) 
{
    double3 translation = paramDeltas.x * captureState.world_line.direction;
    captureState.gizmo->origin = captureState.start_origin + translation;
}
static void axis_rotation_update(double3 paramDeltas, GizmoCaptureState& captureState) 
{
    quat delta_rotation = quat::fromAxisAngle(captureState.world_line.direction, paramDeltas.x);
    captureState.gizmo->orientation = delta_rotation * captureState.start_orientation;
}






//////////////////////
// Gizmo Construction
//////////////////////


inline BaseGizmo create_standard_TRS_gizmo() 
{
    BaseGizmo TRSGizmo;

    GizmoBindings translationBindings;
    translationBindings.onParameterUpdate = &axis_translation_update;

    GizmoElement TranslateX = GizmoElement::MakeLineElement((int) EGizmoElement::TranslateX,
            double3(1, 0, 0), double2(0, 1), GizmoConstants::GizmoRed);
    TranslateX.bindings = translationBindings;

    GizmoElement TranslateY = GizmoElement::MakeLineElement((int) EGizmoElement::TranslateY,
            double3(0, 1, 0), double2(0, 1), GizmoConstants::GizmoGreen);
    TranslateY.bindings = translationBindings;

    GizmoElement TranslateZ = GizmoElement::MakeLineElement((int) EGizmoElement::TranslateZ,
            double3(0, 0, 1), double2(0, 1), GizmoConstants::GizmoBlue);
    TranslateZ.bindings = translationBindings;

    GizmoBindings rotationBindings;
    rotationBindings.onParameterUpdate = &axis_rotation_update;

    GizmoElement RotateY = GizmoElement::MakeCircleElement((int) EGizmoElement::RotateAroundY,
            double3(0, 1, 0), 2.5, GizmoConstants::GizmoGreen);
    RotateY.bindings = rotationBindings;

    TRSGizmo.Elements.push_back(TranslateX);
    TRSGizmo.Elements.push_back(TranslateY);
    TRSGizmo.Elements.push_back(TranslateZ);
    TRSGizmo.Elements.push_back(RotateY);

    return TRSGizmo;
}







//////////////////////
// camera-related
//////////////////////

struct GizmoCameraInfo {
    double3 position;
    double3 left;
    double3 forward;
    double3 up;

    double horzFOV;
    double vertFOV;

    double viewWidth;
    double viewHeight;
};

inline GizmoCameraInfo extract_camera_info(View* view) 
{
    const filament::Camera& camera = view->getCamera();

    GizmoCameraInfo camInfo;
    camInfo.position = camera.getPosition();
    camInfo.left = camera.getLeftVector();
    camInfo.forward = camera.getForwardVector();
    camInfo.up = camera.getUpVector();
    camInfo.horzFOV = camera.getFieldOfViewInDegrees(Camera::Fov::HORIZONTAL);
    camInfo.vertFOV = camera.getFieldOfViewInDegrees(Camera::Fov::VERTICAL);

    camInfo.viewWidth = view->getViewport().width;
    camInfo.viewHeight = view->getViewport().height;

    return camInfo;
}

inline double calc_view_scaling_factor(const double3& worldOrigin, const GizmoCameraInfo& camInfo)
{
    double target_view_angle = camInfo.vertFOV * GizmoConstants::GizmoVisualAngleFOVFraction;
    double3 origin_to_eye = normalize(worldOrigin - camInfo.position);
    double3 left_to_eye = normalize((worldOrigin + camInfo.left) - camInfo.position);
    double angle = std::acos(dot(origin_to_eye, left_to_eye)) * math::d::RAD_TO_DEG;
    // perhaps should average left/right up/down angles to stabilize
    // also this is nonlinear so it won't stay completely same size...
    double scale_t = target_view_angle / angle;

    // maintain constancy as window is resized (720 is arbitrary here)
    double viewportScale = 720 / camInfo.viewHeight;

    return scale_t * viewportScale;
}


inline ray3 construct_eye_ray(const double2& mousePosition, const GizmoCameraInfo& camInfo)
{
    // this mess is to construct an eye ray for hit-testing at the cursor position, w/ filament camera.
    // can be simplified significantly    
    double3 centerPos = camInfo.position + (1.0 * camInfo.forward);
    // left/right are backwards??
    double3 rightPos = centerPos + std::tan(camInfo.horzFOV * 0.5 * math::d::DEG_TO_RAD) * camInfo.left;
    double3 leftPos = centerPos - std::tan(camInfo.horzFOV * 0.5 * math::d::DEG_TO_RAD) * camInfo.left;
    double3 dlr = segment3(leftPos, rightPos).point_interp(mousePosition.x / camInfo.viewWidth);
    double3 deltax = dlr - centerPos;
    double3 upPos = centerPos + std::tan(camInfo.vertFOV * 0.5 * math::d::DEG_TO_RAD) * camInfo.up;
    double3 downPos = centerPos - std::tan(camInfo.vertFOV * 0.5 * math::d::DEG_TO_RAD) * camInfo.up;
    double3 dud = segment3(downPos, upPos).point_interp((1.0 - mousePosition.y / camInfo.viewHeight));
    double3 deltay = dud - centerPos;
    double3 combinedPos = centerPos + deltax + deltay;
    double3 ray2_dir = normalize(combinedPos - camInfo.position);
    double3 ray2_origin = camInfo.position;
    ray3 hit_ray = ray3(ray2_origin, ray2_dir);
    return hit_ray;
}








}

#endif
