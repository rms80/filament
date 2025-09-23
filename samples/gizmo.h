#ifndef GIZMO_H
#define GIZMO_H

#include <cmath>
#include <algorithm>

#include <math/vec3.h>

#include "gizmo_math.h"


namespace gizmo {


    struct GizmoVertex {
        filament::math::float3 position;
        uint32_t color;
    };


    enum EGizmoElement
    {
        None        = 0,
        TranslateX  = 1 << 1,
        TranslateY  = 1 << 2,
        TranslateZ  = 1 << 3
    };


    class TRSGizmo {
    public:
        filament::math::double3 origin = double3(0, 0, 0);
        filament::math::quat orientation = quat();
        double view_scale = 1.0f;

        std::vector<GizmoVertex> vertex_data;
        filament::VertexBuffer* vertex_buffer = nullptr;

        std::vector<uint32_t> index_data;
        filament::IndexBuffer* index_buffer = nullptr;

        filament::Box bounds;

        filament::Material* material = nullptr;
        utils::Entity renderable;


        double visual_angle_thresh_deg = 0.5;
        EGizmoElement capture_element = EGizmoElement::None;
        mat4 start_transform;
        double3 start_origin;
        quat start_orientation;
        line3 world_line;
        double start_param = 0;

    public:

        mat4 get_view_transform() const
        { 
            mat3 rotation(orientation);
            mat3 scale(view_scale);
            return mat4{ scale*rotation, origin };
        }

        mat4 get_frame_transform() const
        {
            mat3 rotation(orientation);
            return mat4{ rotation, origin };
        }

    };




    inline TRSGizmo create_gizmo(filament::Engine& engine)
    { 
        TRSGizmo gizmo;

        uint32_t red = 0xff0000ff, green = 0xff00ff00, blue = 0xffff0000;

        gizmo.vertex_data = {
            { float3(0, 0, 0), red },        { float3(1, 0, 0), red },
            { float3(0, 0, 0), green },      { float3(0, 1, 0), green },
            { float3(0, 0, 0), blue },       { float3(0, 0, 1), blue }
        };
        gizmo.index_data = { 0, 1, 2, 3, 4, 5 };
        gizmo.bounds = compute_pointset_bounds((const float*)gizmo.vertex_data.data(),
                gizmo.vertex_data.size(), sizeof(GizmoVertex));

        gizmo.vertex_buffer = filament::VertexBuffer::Builder()
                        .vertexCount(gizmo.vertex_data.size())
                        .bufferCount(1)
                        .attribute(filament::VertexAttribute::POSITION, 0,
                                filament::VertexBuffer::AttributeType::FLOAT3, 0, sizeof(GizmoVertex))
                        .attribute(VertexAttribute::COLOR, 0, VertexBuffer::AttributeType::UBYTE4,
                                sizeof(filament::math::float3), sizeof(GizmoVertex))
                        .normalized(VertexAttribute::COLOR)
                        .build(engine);

        gizmo.vertex_buffer->setBufferAt(engine, 0,
                filament::backend::BufferDescriptor(gizmo.vertex_data.data(),
                        gizmo.vertex_data.size() * sizeof(GizmoVertex)));

        gizmo.index_buffer = filament::IndexBuffer::Builder()
                                     .indexCount(gizmo.index_data.size())
                                     .bufferType(IndexBuffer::IndexType::UINT)
                                     .build(engine);

        gizmo.index_buffer->setBuffer(engine, IndexBuffer::BufferDescriptor(gizmo.index_data.data(),
                                                      gizmo.index_data.size() * sizeof(uint32_t)));

        gizmo.material = Material::Builder()
                                 //.package(RESOURCES_SANDBOXUNLIT_DATA, RESOURCES_SANDBOXUNLIT_SIZE)
                                 .package(RESOURCES_BAKEDCOLOR_DATA, RESOURCES_BAKEDCOLOR_SIZE)
                                 .build(engine);
        gizmo.material->getDefaultInstance()->setDepthCulling(false);

        gizmo.renderable = utils::EntityManager::get().create();
        filament::RenderableManager::Builder(1)
                .boundingBox(gizmo.bounds)
                .geometry(0, filament::RenderableManager::PrimitiveType::LINES, gizmo.vertex_buffer,
                        gizmo.index_buffer, 0, gizmo.index_data.size())
                .material(0, gizmo.material->getDefaultInstance())
                .priority(6)
                .culling(false)
                .build(engine, gizmo.renderable);

        return gizmo;
    }


    inline void destroy_gizmo(TRSGizmo& gizmo, filament::Engine& engine) 
    { 
        engine.destroy(gizmo.renderable);
        engine.destroy(gizmo.material);
    }



    inline EGizmoElement gizmo_hit_test(TRSGizmo& gizmo, const ray3& hit_ray, double& ray_parameter)
    {
        mat4 transform = gizmo.get_frame_transform();
        double4 world_origin = transform * double4(0, 0, 0, 1);

        for (int i = 0; i <= 2; ++i)
        {
            EGizmoElement elem = static_cast<EGizmoElement>(1 << (i + 1));

            double4 axis_end = double4(0, 0, 0, 1);
            axis_end[i] = gizmo.view_scale;
            axis_end = transform * axis_end;

            segment3 world_seg(world_origin.xyz, axis_end.xyz);
            double ray_param_t = 0, seg_param_t = 0;
            double distsqr = ray_segment_distance_sqr(hit_ray, world_seg, ray_param_t, seg_param_t);

            double3 nearest_dir = normalize(world_seg.point_at(seg_param_t) - hit_ray.origin);
            double visual_angle = vector_angle_deg(hit_ray.direction, nearest_dir);
            if (visual_angle < gizmo.visual_angle_thresh_deg) {
                ray_parameter = ray_param_t;
                return elem;
            }
        }

        ray_parameter = std::numeric_limits<double>::max();
        return EGizmoElement::None;
    }





}

#endif
