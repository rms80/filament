#ifndef GIZMO_H
#define GIZMO_H

#include <cmath>
#include <algorithm>
#include <limits>

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat4.h>

#include <filament/VertexBuffer.h>
#include <filament/IndexBuffer.h>
#include <utils/EntityManager.h>
#include <filament/RenderableManager.h>
#include <filament/Engine.h>

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

    static constexpr double GizmoVisualAngleFOVFraction = 0.1;

    // hit-testing threshold, measured in visual angle.
    // ie the visual angle between the eye-ray and the ray to the gizmo nearest/hit point is
    // measured and must be less than this value. Using visual angle instead of pixel-distance has
    // some benefits (ie works in VR, DPI independent, etc) and some trade-offs (FOV dependent)
    static constexpr double HitTestVisualAngleThreshDeg = 0.5;
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

    // 3D frame, ie location and rotation
    // GizmoElement fields are generally in local space, interpreted relative to this frame
    double3 origin = double3(0, 0, 0);
    quat orientation = quat(1, 0,0,0);

    // local-space GizmoElement dimensions are scaled by this amount in world space.
    // Mainly intended to be used to dynamically resize the gizmo based on projected area
    // (eg maintain constant size as the user zooms in and out)
    double view_scale = 1.0f;

public:

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



struct GizmoRenderVertex {
    filament::math::float3 position;
    uint32_t color;
};


struct GizmoRenderState
{
    std::vector<GizmoRenderVertex> vertex_data;
    filament::VertexBuffer* vertex_buffer = nullptr;
    std::vector<uint32_t> index_data;
    filament::IndexBuffer* index_buffer = nullptr;

    filament::Box bounds;

    filament::Material* material = nullptr;
    utils::Entity renderable;
};




//////////////////////
// Gizmo Construction
//////////////////////


inline BaseGizmo create_standard_TRS_gizmo() 
{
    BaseGizmo TRSGizmo;

    GizmoElement TranslateX = GizmoElement::MakeLineElement((int) EGizmoElement::TranslateX,
            double3(1, 0, 0), double2(0, 1), GizmoConstants::GizmoRed);
    GizmoElement TranslateY = GizmoElement::MakeLineElement((int) EGizmoElement::TranslateY,
            double3(0, 1, 0), double2(0, 1), GizmoConstants::GizmoGreen);
    GizmoElement TranslateZ = GizmoElement::MakeLineElement((int) EGizmoElement::TranslateZ,
            double3(0, 0, 1), double2(0, 1), GizmoConstants::GizmoBlue);

    GizmoElement RotateY = GizmoElement::MakeCircleElement((int) EGizmoElement::RotateAroundY,
            double3(0, 1, 0), 2.5, GizmoConstants::GizmoGreen);

    TRSGizmo.Elements.push_back(TranslateX);
    TRSGizmo.Elements.push_back(TranslateY);
    TRSGizmo.Elements.push_back(TranslateZ);
    TRSGizmo.Elements.push_back(RotateY);

    return TRSGizmo;
}



//////////////////////
// Rendering
//////////////////////


inline void build_gizmo_render_state(BaseGizmo& gizmo, GizmoRenderState& renderState,
        filament::Engine& engine) 
{
    for (const GizmoElement& Element: gizmo.Elements) {
        if (Element.type == EGizmoElementType::LineSegment) {
            int n = (int) renderState.vertex_data.size();
            GizmoRenderVertex start = { Element.params.x * Element.axis, Element.color };
            renderState.vertex_data.push_back(start);
            GizmoRenderVertex end = { Element.params.y * Element.axis, Element.color };
            renderState.vertex_data.push_back(end);
            renderState.index_data.push_back(n);
            renderState.index_data.push_back(n + 1);      
        }
        else if (Element.type == EGizmoElementType::Circle)
        {
            const int steps = 32;
            double3 tanx, tany;
            make_perp_vectors(Element.axis, tanx, tany);
            double radius = Element.params.x;
            int start_idx = (int) renderState.vertex_data.size();
            for (int i = 0; i < steps; ++i) {
                double theta = ((double) i / (double) steps) * (2.0 * math::F_PI);
                double x = radius * std::cos(theta), y = radius * std::sin(theta);
                GizmoRenderVertex circle_pt{ x * tanx + y * tany, Element.color };
                renderState.vertex_data.push_back(circle_pt);
                renderState.index_data.push_back(start_idx + i);
                renderState.index_data.push_back(start_idx + ((i + 1) % steps));
            }
        }
    }

    renderState.bounds = compute_pointset_bounds((const float*) renderState.vertex_data.data(),
            renderState.vertex_data.size(), sizeof(GizmoRenderVertex));

    renderState.vertex_buffer = filament::VertexBuffer::Builder()
                    .vertexCount(renderState.vertex_data.size())
                    .bufferCount(1)
                    .attribute(filament::VertexAttribute::POSITION, 0,
                            filament::VertexBuffer::AttributeType::FLOAT3, 0,
                            sizeof(GizmoRenderVertex))
                    .attribute(VertexAttribute::COLOR, 0, VertexBuffer::AttributeType::UBYTE4,
                            sizeof(filament::math::float3), sizeof(GizmoRenderVertex))
                    .normalized(VertexAttribute::COLOR)
                    .build(engine);

    renderState.vertex_buffer->setBufferAt(engine, 0,
            filament::backend::BufferDescriptor(renderState.vertex_data.data(),
                    renderState.vertex_data.size() * sizeof(GizmoRenderVertex)));

    renderState.index_buffer = filament::IndexBuffer::Builder()
                        .indexCount(renderState.index_data.size())
                        .bufferType(IndexBuffer::IndexType::UINT)
                        .build(engine);

    renderState.index_buffer->setBuffer(engine,
            IndexBuffer::BufferDescriptor(renderState.index_data.data(),
                    renderState.index_data.size() * sizeof(uint32_t)));

    renderState.material = Material::Builder()
                    //.package(RESOURCES_SANDBOXUNLIT_DATA, RESOURCES_SANDBOXUNLIT_SIZE)
                    .package(RESOURCES_BAKEDCOLOR_DATA, RESOURCES_BAKEDCOLOR_SIZE)
                    .build(engine);
    renderState.material->getDefaultInstance()->setDepthCulling(false);

    renderState.renderable = utils::EntityManager::get().create();
    filament::RenderableManager::Builder(1)
            .boundingBox(renderState.bounds)
            .geometry(0, filament::RenderableManager::PrimitiveType::LINES,
                    renderState.vertex_buffer,
                    renderState.index_buffer, 0, renderState.index_data.size())
            .material(0, renderState.material->getDefaultInstance())
            .priority(6)
            .culling(false)
            .build(engine, renderState.renderable);
}


inline void destroy_gizmo(BaseGizmo& gizmo, GizmoRenderState& renderState,
        filament::Engine& engine) { 
    engine.destroy(renderState.renderable);
    engine.destroy(renderState.material);
}



//////////////////////
// Hit Testing
//////////////////////


// state information used during gizmo capture/udpate
// (not all fields are used for all gizmo types)
struct GizmoCaptureState 
{
    int capture_element = EGizmoElement::None;
    mat4 start_transform;
    double3 start_origin;
    quat start_orientation;
    line3 world_line;
    double start_param = 0;
};



struct GizmoHitResult
{
    int hit_identifier = (int)EGizmoElement::None;
    double hit_distance = std::numeric_limits<double>::max();
    double hit_ray_param = std::numeric_limits<double>::max();
};


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





}

#endif
