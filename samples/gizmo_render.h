#ifndef GIZMO_RENDER_H
#define GIZMO_RENDER_H

#include <filament/VertexBuffer.h>
#include <filament/IndexBuffer.h>
#include <utils/EntityManager.h>
#include <filament/RenderableManager.h>
#include <filament/Engine.h>

#include "gizmo_math.h"
#include "gizmo.h"


namespace gizmo {


//////////////////////
// Rendering
//////////////////////



struct GizmoRenderVertex {
    filament::math::float3 position;
    uint32_t color;
};


struct GizmoRenderState {
    std::vector<GizmoRenderVertex> vertex_data;
    filament::VertexBuffer* vertex_buffer = nullptr;
    std::vector<uint32_t> index_data;
    filament::IndexBuffer* index_buffer = nullptr;

    filament::Box bounds;

    filament::Material* material = nullptr;
    utils::Entity renderable;
};



inline void rebuild_gizmo_render_state(BaseGizmo& gizmo, GizmoRenderState& renderState,
        filament::Engine& engine) 
{
    // destroy existing resources
    if (renderState.renderable.isNull() == false) {
        engine.destroy(renderState.renderable);
        renderState.vertex_data.clear();
        renderState.index_data.clear();
    }

    for (const GizmoElement& Element : gizmo.Elements) {
        bool bIsHovered = (Element.identifier == gizmo.hoveredIdentifier);
        uint32_t elementColor = (bIsHovered) ? GizmoConstants::GizmoHoverColor : Element.color;

        if (Element.type == EGizmoElementType::LineSegment) {
            int n = (int) renderState.vertex_data.size();
            GizmoRenderVertex start = { Element.params.x * Element.axis, elementColor };
            renderState.vertex_data.push_back(start);
            GizmoRenderVertex end = { Element.params.y * Element.axis, elementColor };
            renderState.vertex_data.push_back(end);
            renderState.index_data.push_back(n);
            renderState.index_data.push_back(n + 1);
        } else if (Element.type == EGizmoElementType::Circle) {
            const int steps = 32;
            double3 tanx, tany;
            make_perp_vectors(Element.axis, tanx, tany);
            double radius = Element.params.x;
            int start_idx = (int) renderState.vertex_data.size();
            for (int i = 0; i < steps; ++i) {
                double theta = ((double) i / (double) steps) * (2.0 * math::F_PI);
                double x = radius * std::cos(theta), y = radius * std::sin(theta);
                GizmoRenderVertex circle_pt{ x * tanx + y * tany, elementColor };
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

    renderState.renderable = utils::EntityManager::get().create();
    filament::RenderableManager::Builder(1)
            .boundingBox(renderState.bounds)
            .geometry(0, filament::RenderableManager::PrimitiveType::LINES,
                    renderState.vertex_buffer, renderState.index_buffer, 0,
                    renderState.index_data.size())
            .material(0, renderState.material->getDefaultInstance())
            .priority(6)
            .culling(false)
            .build(engine, renderState.renderable);
}



inline void build_gizmo_render_state(BaseGizmo& gizmo, GizmoRenderState& renderState,
        filament::Engine& engine) {

    renderState.material =
            Material::Builder()
                    //.package(RESOURCES_SANDBOXUNLIT_DATA, RESOURCES_SANDBOXUNLIT_SIZE)
                    .package(RESOURCES_BAKEDCOLOR_DATA, RESOURCES_BAKEDCOLOR_SIZE)
                    .build(engine);
    renderState.material->getDefaultInstance()->setDepthCulling(false);

    rebuild_gizmo_render_state(gizmo, renderState, engine);
}


inline void update_gizmo_render_state(BaseGizmo& gizmo, GizmoRenderState& renderState,
        filament::Engine& engine) 
{
    rebuild_gizmo_render_state(gizmo, renderState, engine);
}



inline void destroy_gizmo(BaseGizmo& gizmo, GizmoRenderState& renderState,
        filament::Engine& engine) { 
    engine.destroy(renderState.renderable);
    engine.destroy(renderState.material);
}


}

#endif
