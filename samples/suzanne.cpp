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




static filament::math::double2 ViewportSize;
static filament::math::double2 MousePosition;




void begin_capture(TRSGizmo& gizmo, const ray3& hit_ray, double ray_parameter, EGizmoElement hit_element)
{
    gizmo.capture_element = hit_element;

    mat4 transform = gizmo.get_frame_transform();
    double4 world_origin = transform * double4(0, 0, 0, 1);
    double4 axis(
        (hit_element == EGizmoElement::TranslateX)?gizmo.view_scale:0.0, 
        (hit_element == EGizmoElement::TranslateY)?gizmo.view_scale:0.0,
        (hit_element == EGizmoElement::TranslateZ)?gizmo.view_scale:0.0, 0.0 );
    double4 world_axis = normalize(transform * axis);
    gizmo.world_line = line3(world_origin.xyz, world_axis.xyz);
    gizmo.start_param = gizmo.world_line.project( hit_ray.point_at(ray_parameter) );
    gizmo.start_transform = transform;
    gizmo.start_origin = gizmo.origin;
    gizmo.start_orientation = gizmo.orientation;
}

void update_capture(TRSGizmo& gizmo, Engine* engine, const ray3& update_ray)
{ 
    double ray_param_t, line_param_t;
    double distsqr = ray_line_distance_sqr(update_ray, gizmo.world_line, ray_param_t, line_param_t);

    double dp = line_param_t - gizmo.start_param;
    double3 translation = dp * gizmo.world_line.direction;

    gizmo.origin = gizmo.start_origin + translation;
}

void end_capture(TRSGizmo& gizmo)
{ 
    gizmo.capture_element = EGizmoElement::None;
}


enum EGizmoCaptureState
{
    NotCapturing,
    CapturePending,
    Capturing,
    EndCapturePending
};
EGizmoCaptureState gizmo_capture_state = EGizmoCaptureState::NotCapturing;

static EGizmoElement last_gizmo_hit_element = EGizmoElement::None;
static double2 gizmo_capture_start_pos;
static double2 gizmo_last_pos;

void tick_gizmo(TRSGizmo& gizmo, App& app, Engine* engine, View* view, double now) 
{
    const filament::Camera& camera = view->getCamera();
    double3 camPos = camera.getPosition();
    double3 camLeft = camera.getLeftVector();
    double3 camForward = camera.getForwardVector();
    double3 camUp = camera.getUpVector();
    // seems like filament app always uses vertical fov for scaling
    double camHFOV = camera.getFieldOfViewInDegrees(Camera::Fov::HORIZONTAL);
    double camVFOV = camera.getFieldOfViewInDegrees(Camera::Fov::VERTICAL);

    double target_view_angle = camVFOV * 0.1; // gizmo target visual angle is a fraction of fov
                                              // (should fold in viewport size...)
    double3 origin_to_eye = normalize(gizmo.origin - camPos);
    double3 left_to_eye = normalize((gizmo.origin + camLeft) - camPos);
    double angle = std::acos(dot(origin_to_eye, left_to_eye)) * math::d::RAD_TO_DEG;
    // perhaps should average left/right up/down angles to stabilize
    // also this is nonlinear so it won't stay completely same size...
    double scale_t = target_view_angle / angle;

    // update gizmo transform w/ new view scale
    auto& tcm = engine->getTransformManager();
    TransformManager::Instance gizmoTransformHandle = tcm.getInstance(gizmo.renderable);
    gizmo.view_scale = scale_t;


    // construct eye ray
    // this isn't quite right...works at middle and edge of screen but not in between (??)
    // (is it due to nonlinearity of tangent??)
    //double ty = ( (1.0 - MousePosition.y / ViewportSize.y) - 0.5);  // mousecoords are y-down
    //double vert_angle = ty * camVFOV;
    //double h = std::tan(vert_angle * math::d::DEG_TO_RAD);
    //double tx = (MousePosition.x / ViewportSize.x - 0.5);   // value in range [-0.5, 0.5] because half of fov is on each side
    //double horz_angle = tx * camHFOV;
    //double w = std::tan(horz_angle * math::d::DEG_TO_RAD);
    //double3 view_pos = camPos + (1.0 * camForward) + (h * camUp) + (w * camLeft);

    //double3 ray_dir = normalize(view_pos - camPos);
    //double3 ray_origin = camPos;
    //ray3 hit_ray( ray_origin, ray_dir );


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


    // ray intersection
    //plane3 zplane(double3(0, 0, 1), double3(0, 0, -4));
    //double3 plane_hit_pos = ray_plane_intersection(hit_ray, zplane);
    //gizmo.origin = plane_hit_pos;


    if (gizmo_capture_state == EGizmoCaptureState::Capturing)
    {
        if (gizmo.capture_element != EGizmoElement::None)
            update_capture(gizmo, engine, hit_ray);
    } 
    else if (gizmo_capture_state == EGizmoCaptureState::EndCapturePending)
    {
        end_capture(gizmo);
        gizmo_capture_state = EGizmoCaptureState::NotCapturing;
    } 
    else 
    {
        double ray_param_t = 0;
        EGizmoElement hitElement = gizmo_hit_test(gizmo, hit_ray, ray_param_t);
        if (gizmo_capture_state == EGizmoCaptureState::CapturePending) 
        {
            begin_capture(gizmo, hit_ray, ray_param_t, hitElement);
            gizmo_capture_state = EGizmoCaptureState::Capturing;
        } 
        else {
            last_gizmo_hit_element = hitElement;
        }
    }

    gizmo_last_pos = MousePosition;

    //if (last_gizmo_hit_element != EGizmoElement::None)
    //    gizmo.view_scale *= 2;

    mat4 new_view_transform = gizmo.get_view_transform();
    tcm.setTransform(gizmoTransformHandle, new_view_transform);
    mat4 new_target_transform = gizmo.get_frame_transform();
    TransformManager::Instance meshTransformHandle = tcm.getInstance(app.mesh.renderable);
    tcm.setTransform(meshTransformHandle, new_target_transform);
}


static bool left_mouse_down_state = false;

void imgui_callback(filament::Engine* engine, filament::View* view)
{
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




    ImGui::SetNextWindowSize(ImVec2(0, 0)); 
    ImGui::Begin("Controls");
    ImGui::Button("something!");
    ImGui::NewLine();
    ImGui::Text("window size %.2f %.2f", WindowSize.x, WindowSize.y);
    ImGui::NewLine();
    ImGui::Text("mouse pos %.2f %.2f", MousePos.x-WindowSize.x/2, MousePos.y-WindowSize.y/2);

    //ImGui::SliderInt("Emitters", &app.ui.emitterCount, EMITTER_COUNT_MIN, EMITTER_COUNT_MAX);
    //ImGui::Checkbox("Freeze Particles", &app.ui.particlesFrozen);
    //ImGui::BeginDisabled(app.ui.particlesFrozen);
    //ImGui::Checkbox("Freeze Emitters", &app.ui.freezeEmitters);
    //ImGui::EndDisabled();
    //ImGui::SliderFloat("Gravity", &app.ui.gravityStrength, 0.0f, GRAVITY_STRENGTH_MAX, "%.2f G");
    //ImGui::Checkbox("Enable Lights", &app.ui.lightsEnabled);
    //ImGui::Checkbox("Moonlight", &app.ui.moonlightEnabled);

    //bool fireworks = (app.ui.emitterMode == App::EmitterMode::FIREWORKS);
    //if (ImGui::Checkbox("Fireworks Mode", &fireworks)) {
    //    app.ui.emitterMode = fireworks ? App::EmitterMode::FIREWORKS : App::EmitterMode::CONTINUOUS;
    //}

    //if (app.ui.emitterMode == App::EmitterMode::FIREWORKS) {
    //    ImGui::SliderFloat("Delay", &app.ui.fireworksDelay, FIREWORKS_DELAY_MIN,
    //            FIREWORKS_DELAY_MAX);
    //}
    ImGui::End();
}



int main(int argc, char** argv) {
    Config config;
    config.title = "suzanne";
    config.iblDirectory = FilamentApp::getRootAssetsPath() + IBL_FOLDER;

    handleCommandLineArguments(argc, argv, &config);

    App app;
    TRSGizmo gizmo;
    
    auto setup = [config, &app, &gizmo](Engine* engine, View* view, Scene* scene) {
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

        // create gizmo

        gizmo = create_gizmo(*engine);
        TransformManager::Instance gizmoTransformHandle = tcm.getInstance(gizmo.renderable);
        gizmo.view_scale = 3;
        gizmo.origin = float3(0, 0, -4);
        scene->addEntity(gizmo.renderable);
        tcm.setTransform(gizmoTransformHandle, gizmo.get_view_transform());


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

    auto cleanup = [&app, &gizmo](Engine* engine, View*, Scene*) {
        engine->destroy(app.mesh.renderable);
        engine->destroy(app.materialInstance);
        engine->destroy(app.material);
        engine->destroy(app.albedo);
        engine->destroy(app.normal);
        engine->destroy(app.roughness);
        engine->destroy(app.metallic);
        engine->destroy(app.ao);
        destroy_gizmo(gizmo, *engine);
    };

    FilamentApp::get().animate([&gizmo, &app](Engine* e, View* v, double now) { tick_gizmo(gizmo, app, e, v, now); });
    FilamentApp::get().run(config, setup, cleanup, imgui_callback);

    return 0;
}
