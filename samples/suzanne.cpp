/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common/arguments.h"

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <filament/VertexBuffer.h>
#include <filament/IndexBuffer.h>
#include <filament/Camera.h>
#include <filament/Frustum.h>

#include <utils/EntityManager.h>
#include <utils/Log.h>

#include <filameshio/MeshReader.h>

#include <ktxreader/Ktx2Reader.h>

#include <filamentapp/Config.h>
#include <filamentapp/FilamentApp.h>
#include <filamentapp/IBL.h>

#include <getopt/getopt.h>

#include <utils/Path.h>

#include <stb_image.h>
#include <imgui.h>

#include <iostream>

#include "generated/resources/resources.h"
#include "generated/resources/monkey.h"

// gizmo system
#include "gizmo_math.h"
#include "gizmo.h"


using namespace filament;
using namespace ktxreader;
using namespace filament::math;
using namespace gizmo;

struct App {
    Material* material;
    MaterialInstance* materialInstance;
    filamesh::MeshReader::Mesh mesh;
    mat4f transform;
    Texture* albedo;
    Texture* normal;
    Texture* roughness;
    Texture* metallic;
    Texture* ao;
   
};


struct GizmoSystem {
    BaseGizmo gizmo;
    GizmoRenderState renderState;
    GizmoCaptureState activeCapture;
};



static const char* IBL_FOLDER = "assets/ibl/lightroom_14b";

static void printUsage(char* name) {
    std::string exec_name(utils::Path(name).getName());
    std::string usage(
            "SHOWCASE renders a Suzanne model with compressed textures.\n"
            "Usage:\n"
            "    SHOWCASE [options]\n"
            "Options:\n"
            "   --help, -h\n"
            "       Prints this message\n\n"
            "API_USAGE"
    );
    const std::string from("SHOWCASE");
    for (size_t pos = usage.find(from); pos != std::string::npos; pos = usage.find(from, pos)) {
        usage.replace(pos, from.length(), exec_name);
    }
    const std::string apiUsage("API_USAGE");
    for (size_t pos = usage.find(apiUsage); pos != std::string::npos; pos = usage.find(apiUsage, pos)) {
        usage.replace(pos, apiUsage.length(), samples::getBackendAPIArgumentsUsage());
    }
    std::cout << usage;
}

static int handleCommandLineArguments(int argc, char* argv[], Config* config) {
    static constexpr const char* OPTSTR = "ha:";
    static const struct option OPTIONS[] = {
            { "help",         no_argument,       nullptr, 'h' },
            { "api",          required_argument, nullptr, 'a' },
            { nullptr, 0, nullptr, 0 }
    };
    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, OPTSTR, OPTIONS, &option_index)) >= 0) {
        std::string arg(optarg ? optarg : "");
        switch (opt) {
            default:
            case 'h':
                printUsage(argv[0]);
                exit(0);
            case 'a':
                config->backend = samples::parseArgumentsForBackend(arg);
                break;
        }
    }
    return optind;
}

static Texture* loadNormalMap(Engine* engine, const uint8_t* normals, size_t nbytes) {
    int w, h, n;
    unsigned char* data = stbi_load_from_memory(normals, nbytes, &w, &h, &n, 3);
    Texture* normalMap = Texture::Builder()
            .width(uint32_t(w))
            .height(uint32_t(h))
            .levels(0xff)
            .format(Texture::InternalFormat::RGB8)
            .usage(Texture::Usage::DEFAULT | Texture::Usage::GEN_MIPMAPPABLE)
            .build(*engine);
    Texture::PixelBufferDescriptor buffer(data, size_t(w * h * 3),
            Texture::Format::RGB, Texture::Type::UBYTE,
            (Texture::PixelBufferDescriptor::Callback) &stbi_image_free);
    normalMap->setImage(*engine, 0, std::move(buffer));
    normalMap->generateMipmaps(*engine);
    return normalMap;
}



// this app-state information is extracted from ImGUi in imgui_callback() and used below
static filament::math::double2 ViewportSize;
static filament::math::double2 MousePosition;

// called when gizmo mousedown hit-testing has passed and we want to begin a gizmo interaction
void begin_capture(GizmoSystem& gizmoSystem, const ray3& hit_ray, double ray_parameter, int hit_element)
{
    BaseGizmo& gizmo = gizmoSystem.gizmo;
    GizmoCaptureState& captureState = gizmoSystem.activeCapture;

    captureState.capture_element = hit_element;

    mat4 transform = gizmo.get_frame_transform();
    double4 world_origin = transform * double4(0, 0, 0, 1);

    if (captureState.capture_element >= EGizmoElement::TranslateX &&
            captureState.capture_element <= EGizmoElement::TranslateZ) 
    {
        double4 axis((hit_element == (int) EGizmoElement::TranslateX) ? gizmo.view_scale : 0.0,
                (hit_element == (int) EGizmoElement::TranslateY) ? gizmo.view_scale : 0.0,
                (hit_element == (int) EGizmoElement::TranslateZ) ? gizmo.view_scale : 0.0, 0.0);
        double4 world_axis = normalize(transform * axis);
        captureState.world_line = line3(world_origin.xyz, world_axis.xyz);
        captureState.start_param = captureState.world_line.project(hit_ray.point_at(ray_parameter));
    }
    else if (captureState.capture_element >= EGizmoElement::RotateAroundX &&
               captureState.capture_element <= EGizmoElement::RotateAroundZ)
    {
        // gross
        double4 axis((hit_element == (int) EGizmoElement::RotateAroundX) ? 1 : 0.0,
                (hit_element == (int) EGizmoElement::RotateAroundY) ? 1 : 0.0,
                (hit_element == (int) EGizmoElement::RotateAroundZ) ? 1 : 0.0, 0.0);
        double4 world_axis = normalize(transform * axis);
        
        frame3 worldFrame(world_origin.xyz, quat::fromDirectedRotation(double3(0, 0, 1), world_axis.xyz));
        captureState.world_line = line3(world_origin.xyz, world_axis.xyz);
        captureState.start_param = ray_circle_angleRad(hit_ray, worldFrame);
    }

    captureState.start_transform = transform;
    captureState.start_origin = gizmo.origin;
    captureState.start_orientation = gizmo.orientation;
}

// called every frame while gizmo is capturing
void update_capture(GizmoSystem& gizmoSystem, Engine* engine, const ray3& update_ray) 
{ 
    BaseGizmo& gizmo = gizmoSystem.gizmo;
    GizmoCaptureState& captureState = gizmoSystem.activeCapture;

    if (captureState.capture_element >= EGizmoElement::TranslateX &&
            captureState.capture_element <= EGizmoElement::TranslateZ) {
        double ray_param_t, line_param_t;
        double distsqr =
                ray_line_distance_sqr(update_ray, captureState.world_line, ray_param_t, line_param_t);

        double dp = line_param_t - captureState.start_param;
        double3 translation = dp * captureState.world_line.direction;

        gizmo.origin = captureState.start_origin + translation;
    } 
    else if (captureState.capture_element >= EGizmoElement::RotateAroundX &&
               captureState.capture_element <= EGizmoElement::RotateAroundZ) 
    {
        frame3 worldFrame(captureState.world_line.origin, 
                quat::fromDirectedRotation(double3(0, 0, 1), captureState.world_line.direction));
        double newAngleRad = ray_circle_angleRad(update_ray, worldFrame);

        double dt = newAngleRad - captureState.start_param;
        quat rotation = quat::fromAxisAngle(captureState.world_line.direction, dt);
        gizmo.orientation = rotation * captureState.start_orientation;
    }
}

// called when gizmo capture terminates on mouseup 
void end_capture(GizmoSystem& gizmoSystem) { 
    gizmoSystem.activeCapture.capture_element = EGizmoElement::None;
}

// gizmo capture state machine state
enum EGizmoCaptureState
{
    NotCapturing,
    CapturePending,
    Capturing,
    EndCapturePending
};
EGizmoCaptureState gizmo_capture_state = EGizmoCaptureState::NotCapturing;

static int last_gizmo_hit_element = EGizmoElement::None;
static double2 gizmo_capture_start_pos;     // unused
static double2 gizmo_last_pos;              // unused


// per-frame gizmo processing  (largely should be factored into gizmo.h or utilities)
void tick_gizmo(GizmoSystem& gizmoSystem, App& app, Engine* engine, View* view, double now) 
{
    BaseGizmo& gizmo = gizmoSystem.gizmo;
    const GizmoCaptureState& captureStaste = gizmoSystem.activeCapture;

    // extra filament camera info
    const filament::Camera& camera = view->getCamera();
    double3 camPos = camera.getPosition();
    double3 camLeft = camera.getLeftVector();
    double3 camForward = camera.getForwardVector();
    double3 camUp = camera.getUpVector();
    // seems like filament app always uses vertical fov for scaling
    double camHFOV = camera.getFieldOfViewInDegrees(Camera::Fov::HORIZONTAL);
    double camVFOV = camera.getFieldOfViewInDegrees(Camera::Fov::VERTICAL);

    // figure out worldspace scaling that keeps gizmo a consistent size regardless of camera position

    // gizmo target visual angle is a fraction of fov
    // (todo consider viewport size)
    double target_view_angle = camVFOV * GizmoConstants::GizmoVisualAngleFOVFraction; 
    double3 origin_to_eye = normalize(gizmo.origin - camPos);
    double3 left_to_eye = normalize((gizmo.origin + camLeft) - camPos);
    double angle = std::acos(dot(origin_to_eye, left_to_eye)) * math::d::RAD_TO_DEG;
    // perhaps should average left/right up/down angles to stabilize
    // also this is nonlinear so it won't stay completely same size...
    double scale_t = target_view_angle / angle;

    // update gizmo transform w/ new view scale
    auto& tcm = engine->getTransformManager();
    TransformManager::Instance gizmoTransformHandle = tcm.getInstance(gizmoSystem.renderState.renderable);
    gizmo.view_scale = scale_t;

    // this mess is to construct an eye ray for hit-testing at the cursor position, w/ filament camera.
    // can be simplified significantly    
    double3 centerPos = camPos + (1.0 * camForward);
    // left/right are backwards??
    double3 rightPos = centerPos + std::tan(camHFOV * 0.5 * math::d::DEG_TO_RAD) * camLeft;
    double3 leftPos = centerPos - std::tan(camHFOV * 0.5 * math::d::DEG_TO_RAD) * camLeft;
    double3 dlr = segment3(leftPos, rightPos).point_interp(MousePosition.x / ViewportSize.x);
    double3 deltax = dlr - centerPos;
    double3 upPos = centerPos + std::tan(camVFOV * 0.5 * math::d::DEG_TO_RAD) * camUp;
    double3 downPos = centerPos - std::tan(camVFOV * 0.5 * math::d::DEG_TO_RAD) * camUp;
    double3 dud = segment3(downPos, upPos).point_interp((1.0 - MousePosition.y / ViewportSize.y));
    double3 deltay = dud - centerPos;
    double3 combinedPos = centerPos + deltax + deltay;
    double3 ray2_dir = normalize(combinedPos - camPos);
    double3 ray2_origin = camPos;
    ray3 hit_ray = ray3(ray2_origin, ray2_dir);


    // gizmo-capture state machine
    if (gizmo_capture_state == EGizmoCaptureState::Capturing)
    {
        if (captureStaste.capture_element != EGizmoElement::None)
            update_capture(gizmoSystem, engine, hit_ray);
    } 
    else if (gizmo_capture_state == EGizmoCaptureState::EndCapturePending)
    {
        end_capture(gizmoSystem);
        gizmo_capture_state = EGizmoCaptureState::NotCapturing;
    } 
    else 
    {
        GizmoHitResult hitResult = gizmo_hit_test(gizmo, hit_ray);
        if (gizmo_capture_state == EGizmoCaptureState::CapturePending) 
        {
            if (hitResult.hit_identifier > 0) {
                begin_capture(gizmoSystem, hit_ray, hitResult.hit_ray_param, hitResult.hit_identifier);
                gizmo_capture_state = EGizmoCaptureState::Capturing;
            } else {
                gizmo_capture_state = EGizmoCaptureState::NotCapturing;
                last_gizmo_hit_element = EGizmoElement::None;
            }
        } 
        else {
            last_gizmo_hit_element = hitResult.hit_identifier;
        }
    }

    gizmo_last_pos = MousePosition;

    // update the gizmo and bound mesh every frame (only actually needed if capturing)
    mat4 new_view_transform = gizmo.get_view_transform();
    tcm.setTransform(gizmoTransformHandle, new_view_transform);
    mat4 new_target_transform = gizmo.get_frame_transform();
    TransformManager::Instance meshTransformHandle = tcm.getInstance(app.mesh.renderable);
    tcm.setTransform(meshTransformHandle, new_target_transform);
}


// extract mouse state from ImGUI, and disable FilamentApp
// camera controls if gizmo is capturing input (ie hacks)
void imgui_callback(filament::Engine* engine, filament::View* view)
{
    static bool left_mouse_down_state = false;

    ImVec2 WindowSize = ImGui::GetMainViewport()->Size;
    ViewportSize = double2(WindowSize.x, WindowSize.y);
    ImVec2 MousePos = ImGui::GetMousePos();
    if (MousePos.x >= 0 && MousePos.x <= WindowSize.x)
        MousePosition = double2(MousePos.x, MousePos.y);

    bool left_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    if (left_down == true && left_mouse_down_state == false)        // pressed
    {
        if (gizmo_capture_state == EGizmoCaptureState::NotCapturing
            && last_gizmo_hit_element != EGizmoElement::None )
        {
            FilamentApp::get().SetCameraControlsEnabled(false);
            gizmo_capture_state = EGizmoCaptureState::CapturePending;
            gizmo_capture_start_pos = MousePosition;
        }

        left_mouse_down_state = true;
    }
    else if (left_down == false && left_mouse_down_state == true)   // released
    {
        if (gizmo_capture_state == EGizmoCaptureState::Capturing)
        {
            gizmo_capture_state = EGizmoCaptureState::EndCapturePending;
            FilamentApp::get().SetCameraControlsEnabled(true);
        } 
        else
            gizmo_capture_state == EGizmoCaptureState::NotCapturing;

        left_mouse_down_state = false;
    }

    // test window
    ImGui::SetNextWindowSize(ImVec2(0, 0)); 
    ImGui::Begin("Controls");
    ImGui::Text("window size %.2f %.2f", WindowSize.x, WindowSize.y);
    ImGui::NewLine();
    ImGui::Text("mouse pos %.2f %.2f", MousePos.x-WindowSize.x/2, MousePos.y-WindowSize.y/2);
    ImGui::End();
}




int main(int argc, char** argv) {
    Config config;
    config.title = "suzanne";
    config.iblDirectory = FilamentApp::getRootAssetsPath() + IBL_FOLDER;

    handleCommandLineArguments(argc, argv, &config);

    App app;
    GizmoSystem gizmoSystem;
    
    auto setup = [config, &app, &gizmoSystem](Engine* engine, View* view, Scene* scene) {
        auto& tcm = engine->getTransformManager();
        auto& rcm = engine->getRenderableManager();
        auto& em = utils::EntityManager::get();

        Ktx2Reader reader(*engine);

        reader.requestFormat(Texture::InternalFormat::DXT3_SRGBA);
        reader.requestFormat(Texture::InternalFormat::DXT3_RGBA);

        // Uncompressed formats are lower priority, so they get added last.
        reader.requestFormat(Texture::InternalFormat::SRGB8_A8);
        reader.requestFormat(Texture::InternalFormat::RGBA8);

        constexpr auto sRGB = Ktx2Reader::TransferFunction::sRGB;
        constexpr auto LINEAR = Ktx2Reader::TransferFunction::LINEAR;

        app.albedo = reader.load(MONKEY_ALBEDO_DATA, MONKEY_ALBEDO_SIZE, sRGB);
        app.ao = reader.load(MONKEY_AO_DATA, MONKEY_AO_SIZE, LINEAR);
        app.metallic = reader.load(MONKEY_METALLIC_DATA, MONKEY_METALLIC_SIZE, LINEAR);
        app.roughness = reader.load(MONKEY_ROUGHNESS_DATA, MONKEY_ROUGHNESS_SIZE, LINEAR);

#if !defined(NDEBUG)
        using namespace utils;
        slog.i << "Resolved format for albedo: " << app.albedo->getFormat() << io::endl;
        slog.i << "Resolved format for ambient occlusion: " << app.ao->getFormat() << io::endl;
        slog.i << "Resolved format for metallic: " << app.metallic->getFormat() << io::endl;
        slog.i << "Resolved format for roughness: " << app.roughness->getFormat() << io::endl;
#endif

        app.normal = loadNormalMap(engine, MONKEY_NORMAL_DATA, MONKEY_NORMAL_SIZE);
        TextureSampler sampler(TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
                TextureSampler::MagFilter::LINEAR);

        // Instantiate material.
        app.material = Material::Builder()
                .package(RESOURCES_TEXTUREDLIT_DATA, RESOURCES_TEXTUREDLIT_SIZE).build(*engine);
        app.materialInstance = app.material->createInstance();
        app.materialInstance->setParameter("albedo", app.albedo, sampler);
        app.materialInstance->setParameter("ao", app.ao, sampler);
        app.materialInstance->setParameter("metallic", app.metallic, sampler);
        app.materialInstance->setParameter("normal", app.normal, sampler);
        app.materialInstance->setParameter("roughness", app.roughness, sampler);

        auto ibl = FilamentApp::get().getIBL()->getIndirectLight();
        ibl->setIntensity(100000);
        ibl->setRotation(mat3f::rotation(0.5f, float3{ 0, 1, 0 }));

        // Add geometry into the scene.
        app.mesh = filamesh::MeshReader::loadMeshFromBuffer(engine, MONKEY_SUZANNE_DATA, nullptr,
                nullptr, app.materialInstance);
        TransformManager::Instance ti = tcm.getInstance(app.mesh.renderable);
        mat4f initial_transform = tcm.getWorldTransform(ti);
        app.transform = mat4f{ mat3f(1), float3(0, 0, -4) } * initial_transform;
        rcm.setCastShadows(rcm.getInstance(app.mesh.renderable), true);
        scene->addEntity(app.mesh.renderable);
        tcm.setTransform(ti, app.transform);

        // create TRS gizmo and position it where the mesh is centered, at (0,0,-4)  (could get from app.transfrom...)
        gizmoSystem.gizmo = create_standard_TRS_gizmo();
        build_gizmo_render_state(gizmoSystem.gizmo, gizmoSystem.renderState, *engine);
        TransformManager::Instance gizmoTransformHandle =
                tcm.getInstance(gizmoSystem.renderState.renderable);
        gizmoSystem.gizmo.view_scale = 3;
        gizmoSystem.gizmo.origin = float3(0, 0, -4);
        scene->addEntity(gizmoSystem.renderState.renderable);
        tcm.setTransform(gizmoTransformHandle, gizmoSystem.gizmo.get_view_transform());


        // unlit material with constant color...
        //filament::Material* lineMaterial = Material::Builder()
        //                .package(RESOURCES_SANDBOXUNLIT_DATA, RESOURCES_SANDBOXUNLIT_SIZE)
        //                .build(*engine);
        //filament::MaterialInstance* lineMaterialInstance = lineMaterial->createInstance();
        //const filament::LinearColor color{ 1.0f, 0.0f, 0.0f };
        //lineMaterialInstance->setParameter("baseColor", RgbType::LINEAR, color);
        //lineMaterialInstance->setParameter("emissive", RgbType::LINEAR,
        //        color * 10.0f);
        //lineMaterialInstance->setCullingMode(MaterialInstance::CullingMode::NONE);

    };

    auto cleanup = [&app, &gizmoSystem](Engine* engine, View*, Scene*) {
        engine->destroy(app.mesh.renderable);
        engine->destroy(app.materialInstance);
        engine->destroy(app.material);
        engine->destroy(app.albedo);
        engine->destroy(app.normal);
        engine->destroy(app.roughness);
        engine->destroy(app.metallic);
        engine->destroy(app.ao);
        destroy_gizmo(gizmoSystem.gizmo, gizmoSystem.renderState, *engine);
    };

    FilamentApp::get().animate([&gizmoSystem, &app](Engine* e, View* v, double now) {
        tick_gizmo(gizmoSystem, app, e, v, now);
    });
    FilamentApp::get().run(config, setup, cleanup, imgui_callback);

    return 0;
}
