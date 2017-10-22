/* Copyright (c) 2011-2017, Cyrille Favreau
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Cyrille Favreau <cyrille_favreau@hotmail.com>
 *
 * This file is part of Sol-R <https://github.com/cyrillefavreau/Sol-R>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

// Project
#include "Scene.h"
#include <common/Utils.h>

// SoL-R
#include <io/FileMarshaller.h>

// OpenGL
#include <opengl/rtgl.h>

// System
#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

// Sixense
#ifdef USE_SIXENSE
#ifdef _DEBUG
#pragma comment(lib, "sixensed_s_x64.lib")
#pragma comment(lib, "sixense_utilsd_s_x64.lib")
#else
#pragma comment(lib, "sixense_s_x64.lib")
#pragma comment(lib, "sixense_utils_s_x64.lib")
#endif // _DEBUG

#define SIXENSE_STATIC_LIB
#define SIXENSE_UTILS_STATIC_LIB
#include <sixense.h>
#include <sixense_math.hpp>
#ifdef WIN32
#include <sixense_utils/mouse_pointer.hpp>
#endif
#include <sixense_utils/button_states.hpp>
#include <sixense_utils/controller_manager/controller_manager.hpp>
#include <sixense_utils/derivatives.hpp>
#include <sixense_utils/event_triggers.hpp>
#endif // USE_SIXENSE

using namespace solr;

const int gTotalPathTracingIterations = 2000;

#ifdef USE_SIXENSE
// flags that the controller manager system can set to tell the graphics system
// to draw the instructions
// for the player
static bool controller_manager_screen_visible = true;
std::string controller_manager_text_string;

// This is the callback that gets registered with the
// sixenseUtils::controller_manager. It will get called each time the user
// completes
// one of the setup steps so that the game can update the instructions to the
// user. If the engine supports texture mapping, the
// controller_manager can prove a pathname to a image file that contains the
// instructions in graphic form.
// The controller_manager serves the following functions:
//  1) Makes sure the appropriate number of controllers are connected to the
//  system. The number of required controllers is designaged by the
//     game type (ie two player two controller game requires 4 controllers, one
//     player one controller game requires one)
//  2) Makes the player designate which controllers are held in which hand.
//  3) Enables hemisphere tracking by calling the Sixense API call
//  sixenseAutoEnableHemisphereTracking. After this is completed full 360 degree
//     tracking is possible.
void controller_manager_setup_callback(sixenseUtils::ControllerManager::setup_step step)
{
    if (sixenseUtils::getTheControllerManager()->isMenuVisible())
    {
        // Turn on the flag that tells the graphics system to draw the instruction
        // screen instead of the controller information. The game
        // should be paused at this time.
        controller_manager_screen_visible = true;

        // Ask the controller manager what the next instruction string should be.
        controller_manager_text_string = sixenseUtils::getTheControllerManager()->getStepString();

        // We could also load the supplied controllermanager textures using the
        // filename: sixenseUtils::getTheControllerManager()->getTextureFileName();
    }
    else
    {
        // We're done with the setup, so hide the instruction screen.
        controller_manager_screen_visible = false;
    }
}
#endif // USE_SIXENSE

/*
________________________________________________________________________________

________________________________________________________________________________
 */
Scene::Scene(const std::string &name)
    : m_name(name)
    , m_nbHDRI(0)
    , m_currentModel(0)
{
    m_nbPrimitives = 0;
    m_nbBoxes = 0;

    // Kernel
    m_gpuKernel = 0;

    m_cornellBoxType = 0;
    m_groundHeight = -2500.f; // Ground altitude
    m_maxPathTracingIterations = gTotalPathTracingIterations;

    // Materials
    m_nbMaterials = 0;

    // Primitives
    m_nbPrimitives = 0;

    // Animation
    m_rotationCenter.x = 0.f;
    m_rotationCenter.y = 0.f;
    m_rotationCenter.z = 0.f;

    m_rotationAngles.x = 0.f;
    m_rotationAngles.y = 0.f;
    m_rotationAngles.z = 0.f;

#ifdef USE_KINECT
    m_skeletonPrimitiveIndex = -1;
    m_skeletonSize = 1500.f;
    m_skeletonThickness = 150.f;

    m_skeletonPosition.x = 0.f;
    m_skeletonPosition.y = -1800.f;
    m_skeletonPosition.z = 0.f;

    m_skeletonOldPosition.x = 0.f;
    m_skeletonOldPosition.y = 0.f;
    m_skeletonOldPosition.z = 0.f;

    m_skeletonKinectSpace = 30.f;
    m_skeletonKinectSize = 10000.f;
    m_skeletonKinectStep = 16;
    m_skeletonKinectNbSpherePerBox = 20;

// m_previewViewPos = m_gpuKernel->getViewPos();
#endif // USE_KINECT
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
Scene::~Scene(void)
{
    if (m_gpuKernel)
        m_gpuKernel->cleanup();

#ifdef USE_SIXENSE
    // Sixense
    sixenseExit();
#endif // USE_SIXENSE
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
void Scene::initialize(solr::GPUKernel *kernel, const int width, const int height)
{
    LOG_INFO(1, "--------------------------------------------------------------------------------");
    LOG_INFO(1, "Scene...............: " << m_name);

    m_gpuKernel = kernel;
    m_gpuKernel->initBuffers();
    m_gpuKernel->resetAll();
    m_gpuKernel->setFrame(0);

    SceneInfo &sceneInfo = m_gpuKernel->getSceneInfo();
    // Scene
    sceneInfo.size.x = width;
    sceneInfo.size.y = height;
    sceneInfo.graphicsLevel = (sceneInfo.cameraType == ctVolumeRendering) ? glNoShading : glFull;
    sceneInfo.nbRayIterations = 3;
    sceneInfo.transparentColor = 0.f;
    sceneInfo.viewDistance = 50000.f;
    sceneInfo.shadowIntensity = 1.0f;
    sceneInfo.eyeSeparation = 380.f;
    sceneInfo.backgroundColor.x = 0.0f;
    sceneInfo.backgroundColor.y = 0.0f;
    sceneInfo.backgroundColor.z = 0.0f;
    sceneInfo.backgroundColor.w = 0.5f;
    sceneInfo.renderBoxes = 0;
    sceneInfo.pathTracingIteration = 0;
    sceneInfo.maxPathTracingIterations = gTotalPathTracingIterations;
    sceneInfo.frameBufferType = ftRGB;
    sceneInfo.timestamp = 0;
    sceneInfo.atmosphericEffect = aeNone;
    sceneInfo.cameraType = ctPerspective;
    sceneInfo.doubleSidedTriangles = false;
    sceneInfo.extendedGeometry = true;
    sceneInfo.advancedIllumination = aiNone;
    sceneInfo.draftMode = false;
    sceneInfo.skyboxRadius = static_cast<int>(sceneInfo.viewDistance * 0.9f);
    sceneInfo.skyboxMaterialId = SKYBOX_SPHERE_MATERIAL;
    sceneInfo.gradientBackground = 0;
    sceneInfo.geometryEpsilon = 0.05f;
    sceneInfo.rayEpsilon = 0.05f;

    // HDRI
    Strings filters;
    filters.push_back(".bmp");
    filters.push_back(".jpg");
    loadTextures(std::string(DEFAULT_MEDIA_FOLDER) + "/hdri", filters);
    m_nbHDRI = m_gpuKernel->getNbActiveTextures();
    LOG_INFO(1, "HDRI textures......: " << m_nbHDRI);

    // Textures
    loadTextures(std::string(DEFAULT_MEDIA_FOLDER) + "/textures", filters);
    LOG_INFO(1, "Material textures..: " << m_gpuKernel->getNbActiveTextures() - m_nbHDRI);

    // Materials
    createRandomMaterials(false, false);

    // Initialize Scene
    doInitialize();

    // Lights TODO
    doAddLights();

    // Cornel Box
    addCornellBox(m_cornellBoxType);

#ifdef USE_KINECT
    // Kinect
    createSkeleton();
#endif // USE_KINECT

    m_nbBoxes = m_gpuKernel->compactBoxes(true);

    doPostInitialize();

#ifdef USE_SIXENSE
    // --------------------------------------------------------------------------------
    // Init sixense
    // --------------------------------------------------------------------------------
    LOG_INFO(1, "---------------------------");
    LOG_INFO(1, "Initializing Sixense (o_o)y");
    LOG_INFO(1, "---------------------------");
    sixenseInit();
    // Init the controller manager. This makes sure the controllers are present,
    // assigned to left and right hands, and that
    // the hemisphere calibration is complete.
    sixenseUtils::getTheControllerManager()->setGameType(sixenseUtils::ControllerManager::ONE_PLAYER_TWO_CONTROLLER);
    sixenseUtils::getTheControllerManager()->registerSetupCallback(controller_manager_setup_callback);

    m_modelPosition.x = 0.f;
    m_modelPosition.y = 0.f;
    m_modelPosition.z = 0.f;

    m_modelTranslation.x = 0.f;
    m_modelTranslation.y = 0.f;
    m_modelTranslation.z = 0.f;

    m_modelRotationAngle.x = 0.f;
    m_modelRotationAngle.y = 0.f;
    m_modelRotationAngle.z = 0.f;
#endif // USE_SIXENSE
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
void Scene::animate()
{
    doAnimate();
    m_rotationAngles.x = 0.f;
    m_rotationAngles.y = 0.f;
    m_rotationAngles.z = 0.f;
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
void Scene::rotatePrimitives(const vec3f &rotationCenter, const vec4f &angles)
{
    m_gpuKernel->rotatePrimitives(rotationCenter, angles);
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
void Scene::render(const bool &animate)
{
    animateSkeleton();

    m_gpuKernel->render_begin(0);

    if (animate)
        doAnimate();

    m_gpuKernel->render_end();
}

/*
________________________________________________________________________________

Create Textures
________________________________________________________________________________
 */
void Scene::loadTextures(const std::string &path, const Strings &filters)
{
    const Strings textureFiles = getFilesFromFolder(path, filters);
    for (std::vector<std::string>::const_iterator it = textureFiles.begin(); it != textureFiles.end(); ++it)
        if (m_gpuKernel->loadTextureFromFile(m_gpuKernel->getNbActiveTextures(), *it))
            LOG_INFO(3, "[Slot " << m_gpuKernel->getNbActiveTextures() - 1 << "] Texture " << *it
                                 << " successfully loaded");
}

/*
________________________________________________________________________________

Create Random Materials
________________________________________________________________________________
 */
void Scene::createRandomMaterials(bool update, bool lightsOnly)
{
    int nbMaterialTexturePacks = (m_gpuKernel->getNbActiveTextures() - m_nbHDRI) / 5;
    int start(0);
    int end(NB_MAX_MATERIALS);
    if (update)
        start = 1000;

    // Materials
    for (int i(start); i < end; ++i)
    {
        vec4f specular = make_vec4f();
        specular.x = 0.1f;
        specular.y = 200.f;
        specular.z = 0.f;
        specular.w = 0.f;

        float reflection = 0.f;
        float refraction = 0.f;
        float transparency = 0.f;
        int refractionPower = 5;

        int diffuseTextureId = TEXTURE_NONE;
        int normalTextureId = TEXTURE_NONE;
        int bumpTextureId = TEXTURE_NONE;
        int specularTextureId = TEXTURE_NONE;
        int reflectionTextureId = TEXTURE_NONE;
        int transparencyTextureId = TEXTURE_NONE;
        int ambientOcclusionTextureId = TEXTURE_NONE;
        vec4f innerIllumination =
            make_vec4f(0.f, m_gpuKernel->getSceneInfo().viewDistance * 10, m_gpuKernel->getSceneInfo().viewDistance);
        bool procedural = false;
        bool wireframe = false;
        int wireframeDepth = 0;
        float r, g, b, gloss;
        r = 0.2f + rand() % 600 / 1000.f;
        g = 0.2f + rand() % 600 / 1000.f;
        b = 0.2f + rand() % 600 / 1000.f;
        gloss = 0.f;
        float opacity = 0.f;

        switch (i)
        {
        case 1011:
            r = 0.5f;
            g = 0.5f;
            b = 0.5f;
            break;
        case 1010:
            r = 0.5f;
            g = 0.5f;
            b = 0.5f;
            break;

        case 1050:
            reflection = 1.f;
            if (rand() % 2 == 0)
            {
                refraction = 1.01f + 0.1f * (rand() % refractionPower);
                transparency = 0.5f + rand() % 500 / 1000.f;
            }
            break;
        case 1051:
            reflection = 0.5f + rand() % 500 / 1000.f;
            break;

        case 1081:
            r = 0.5f;
            g = 0.5f;
            b = 0.5f;
            innerIllumination.x = 0.25f;
            break;

        case 1098:
            r = 0.5f;
            g = 0.5f;
            b = 0.5f;
            reflection = 0.f;
            break;
        case 1099:
            r = 0.5f;
            g = 0.5f;
            b = 0.5f;
            reflection = 0.1f;
            break;

        case SKYBOX_SPHERE_MATERIAL:
            reflection = 0.f;
            diffuseTextureId = 0;
            break;
        case SKYBOX_GROUND_MATERIAL:
        {
            reflection = 0.25f;
            transparency = 0.f;
            refraction = 0.f;
            r = g = b = 0.1f;
            gloss = 0.2f;
        }
        break;

        // Sky Box
        case SKYBOX_FRONT_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            wireframe = true;
            diffuseTextureId = 0;
            break;
        case SKYBOX_RIGHT_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            wireframe = true;
            diffuseTextureId = 1;
            break;
        case SKYBOX_BACK_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            wireframe = true;
            diffuseTextureId = 2;
            break;
        case SKYBOX_LEFT_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            wireframe = true;
            diffuseTextureId = 3;
            break;
        case SKYBOX_TOP_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            wireframe = true;
            diffuseTextureId = 4;
            break;
        case SKYBOX_BOTTOM_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            wireframe = true;
            diffuseTextureId = 5;
            break;

        // Cornell Box
        case CORNELLBOX_FRONT_MATERIAL:
            r = 127.f / 255.f;
            g = 127.f / 255.f;
            b = 127.f / 255.f;
            break;
        case CORNELLBOX_RIGHT_MATERIAL:
            r = 154.f / 255.f;
            g = 94.f / 255.f;
            b = 64.f / 255.f;
            break;
        case CORNELLBOX_BACK_MATERIAL:
            r = 92.f / 255.f;
            g = 93.f / 255.f;
            b = 150.f / 255.f;
            break;
        case CORNELLBOX_LEFT_MATERIAL:
            r = 92.f / 255.f;
            g = 150.f / 255.f;
            b = 93.f / 255.f;
            break;

        // Fractals
        case MANDELBROT_MATERIAL:
            r = 127.f / 255.f;
            g = 127.f / 255.f;
            b = 127.f / 255.f;
            diffuseTextureId = TEXTURE_MANDELBROT;
            break;
        case JULIA_MATERIAL:
            r = 154.f / 255.f;
            g = 94.f / 255.f;
            b = 64.f / 255.f;
            diffuseTextureId = TEXTURE_JULIA;
            break;

#ifdef USE_KINECT
        // KINECT
        case KINECT_COLOR_MATERIAL:
            r = 0.f;
            g = 0.f;
            b = 0.f;
            diffuseTextureId = KINECT_COLOR_TEXTURE;
            break;
#endif // USE_KINECT

        // Basic reflection
        case BASIC_REFLECTION_MATERIAL_001:
            r = 1.f;
            g = 1.f;
            b = 1.f;
            reflection = 1.f;
            refraction = 1.02f;
            transparency = 0.9f;
            break;
        case BASIC_REFLECTION_MATERIAL_002:
            reflection = 0.9f;
            break;
        case BASIC_REFLECTION_MATERIAL_003:
            r = 0.5f;
            g = 1.0f;
            b = 0.7f;
            reflection = 0.f;
            diffuseTextureId = 6;
            bumpTextureId = 8;
            break;
        case BASIC_REFLECTION_MATERIAL_004:
            reflection = 1.f;
            refraction = 1.1f + 0.1f * (rand() % refractionPower);
            transparency = 0.8f;
            break;
        case BASIC_REFLECTION_MATERIAL_005:
            r = 1.f;
            g = 0.f;
            b = 0.f;
            reflection = 0.5f;
            break;
        case BASIC_REFLECTION_MATERIAL_006:
            r = 0.f;
            g = 1.f;
            b = 1.f;
            reflection = 0.5f;
            break;

        // White
        case WHITE_MATERIAL:
            r = 1.f;
            g = 1.f;
            b = 1.f;
            break;

        // Wireframe
        case LIGHT_MATERIAL_001:
            r = 1.f;
            g = 0.f;
            b = 0.f;
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_002:
            r = 0.f;
            g = 1.f;
            b = 0.f;
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_003:
            r = 0.f;
            g = 0.f;
            b = 1.f;
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_004:
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_005:
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_006:
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_007:
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_008:
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_009:
            innerIllumination.x = 0.5f;
            break;
        case LIGHT_MATERIAL_010:
            innerIllumination.x = 0.5f;
            break;
        case DEFAULT_LIGHT_MATERIAL:
            r = 1.f;
            g = 1.f;
            b = 1.f;
            innerIllumination.x = 2.f;
            break;
        case METAL_MATERIAL:
            r = 0.3f;
            g = 0.3f;
            b = 0.3f;
            reflection = 0.5f;
            specular.x = 1.f;
            specular.y = 50.f;
            gloss = 0.5f;
            break;

        default:
            if (i >= RANDOM_MATERIALS_OFFSET)
            {
                switch (rand() % 10)
                {
                case 0:
                {
                    specular.x = 1.f;
                    specular.y = 234.f;
                    reflection = 0.5f + rand() % 100 / 200.f;
                    break;
                }
                case 1:
                {
                    transparency = 0.7f;
                    refraction = 1.1f;
                    reflection = 1.f;
                    specular.x = 1.f;
                    specular.y = 200.f;
                    break;
                }
                case 2:
                {
                    if (nbMaterialTexturePacks > 0)
                    {
                        int index = m_nbHDRI + 5 * (rand() % nbMaterialTexturePacks);
                        for (int t(0); t < 5; ++t)
                        {
                            int idx = index + t;
                            TextureInfo ti = m_gpuKernel->getTextureInformation(idx);
                            switch (ti.type)
                            {
                            case tex_bump:
                                bumpTextureId = idx;
                                break;
                            case tex_normal:
                                normalTextureId = idx;
                                break;
                            case tex_diffuse:
                                diffuseTextureId = idx;
                                break;
                            case tex_specular:
                                specularTextureId = idx;
                                break;
                            case tex_ambient_occlusion:
                                ambientOcclusionTextureId = idx;
                                break;
                            case tex_reflective:
                                reflectionTextureId = idx;
                                break;
                            case tex_transparent:
                                transparencyTextureId = idx;
                                break;
                            }
                        }
                        specular.x = 0.5f;
                        specular.y = 100.f;
                    }
                    break;
                }
                }
            }
        }

        if (update)
            m_nbMaterials = i;
        else
            m_nbMaterials = m_gpuKernel->addMaterial();

        m_gpuKernel->setMaterial(m_nbMaterials, r, g, b, gloss, reflection, refraction, procedural, wireframe,
                                 wireframeDepth, transparency, opacity, diffuseTextureId, normalTextureId,
                                 bumpTextureId, specularTextureId, reflectionTextureId, transparencyTextureId,
                                 ambientOcclusionTextureId, specular.x, specular.y, specular.w, innerIllumination.x,
                                 innerIllumination.y, innerIllumination.z, false);
    }
}

/*
________________________________________________________________________________

Create Molecule Materials
________________________________________________________________________________
 */
void Scene::createMoleculeMaterials(bool update)
{
    vec4f specular;
    // Materials
    specular.x = 0.0f;
    specular.y = 0.0f;
    specular.z = 0.0f;
    specular.w = 1.0f;
    for (int i(0); i < NB_MAX_MATERIALS - 30; ++i)
    {
        specular.x = (i >= 20 && (i / 20) % 2 == 1) ? 0.2f : 0.2f;
        specular.y = (i >= 20 && (i / 20) % 2 == 1) ? 500.f : 500.0f;
        specular.z = 0.f;
        specular.w = 0.1f;

        vec4f innerIllumination = make_vec4f(0.f, 2.f, 100000.f);
        float reflection = 0.f;

        // Transparency & refraction
        float refraction = (i >= 20 && i < 40) ? 1.6f : 0.f;
        float transparency = (i >= 20 && i < 40) ? 0.7f : 0.f;
        bool fastTransparency = (i >= 20 && i < 40);
        float opacity = 0.f;

        int diffuseTextureId = TEXTURE_NONE;
        int bumpTextureId = TEXTURE_NONE;
        int normalTextureId = TEXTURE_NONE;
        int specularTextureId = TEXTURE_NONE;
        int reflectionTextureId = TEXTURE_NONE;
        int transparencyTextureId = TEXTURE_NONE;
        int ambientOcclusionTextureId = TEXTURE_NONE;
        float r, g, b;
        float gloss = 0.f;
        bool procedural = false;

        r = 0.5f + rand() % 40 / 100.f;
        g = 0.5f + rand() % 40 / 100.f;
        b = 0.5f + rand() % 40 / 100.f;

        // Proteins
        switch (i % 10)
        {
        case 0:
            r = 0.8f;
            g = 0.7f;
            b = 0.7f;
            break;
        case 1:
            r = 0.7f;
            g = 0.7f;
            b = 0.7f;
            break; // C Gray
        case 2:
            r = 174.f / 255.f;
            g = 174.f / 255.f;
            b = 233.f / 255.f;
            break; // N Blue
        case 3:
            r = 0.9f;
            g = 0.4f;
            b = 0.4f;
            break; // O
        case 4:
            r = 0.9f;
            g = 0.9f;
            b = 0.9f;
            break; // H White
        case 5:
            r = 0.0f;
            g = 0.5f;
            b = 0.6f;
            break; // B
        case 6:
            r = 0.0f;
            g = 0.0f;
            b = 0.7f;
            break; // F
        case 7:
            r = 0.8f;
            g = 0.6f;
            b = 0.3f;
            break; // P
        case 8:
            r = 241.f / 255.f;
            g = 196.f / 255.f;
            b = 107.f / 255.f;
            break; // S Yellow
        case 9:
            r = 0.9f;
            g = 0.3f;
            b = 0.3f;
            break; // V
        }
        m_nbMaterials = m_gpuKernel->addMaterial();
        m_gpuKernel->setMaterial(m_nbMaterials, r, g, b, gloss, reflection, refraction, procedural, false, 0,
                                 transparency, opacity, diffuseTextureId, normalTextureId, bumpTextureId,
                                 specularTextureId, reflectionTextureId, transparencyTextureId,
                                 ambientOcclusionTextureId, specular.x, specular.y, specular.w, innerIllumination.x,
                                 innerIllumination.y, innerIllumination.z, fastTransparency);
    }
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
void Scene::saveToFile()
{
    solr::FileMarshaller fm;
    std::string filename(m_name);
    filename += ".irt";
    fm.saveToFile(*m_gpuKernel, filename);
}

/*
________________________________________________________________________________

________________________________________________________________________________
 */
void Scene::loadFromFile(const float scale)
{
    solr::FileMarshaller fm;
    std::string filename(m_name);
    filename += ".irt";
    fm.loadFromFile(*m_gpuKernel, filename, make_vec4f(), scale);
}

void Scene::addCornellBox(int boxType)
{
    LOG_INFO(3, "Adding Cornell Box");
    LOG_INFO(3, "Ground height = " << m_groundHeight);
    vec4f skyBoxSize = make_vec4f(20000.f, 20000.f, 20000.f);
    const vec2f groundSize = make_vec2f(10000.f, 10000.f);
    float groundHeight = m_groundHeight;
    float repeats = 10.f;

    switch (boxType)
    {
    case 2:
    case 3:
    case 4:
    {
        // Tiles
        float tiles(1.f);
        int matFront = SKYBOX_FRONT_MATERIAL;
        int matRight = SKYBOX_RIGHT_MATERIAL;
        int matBack = SKYBOX_BACK_MATERIAL;
        int matLeft = SKYBOX_LEFT_MATERIAL;
        int matTop = SKYBOX_TOP_MATERIAL;
        int matBottom = SKYBOX_BOTTOM_MATERIAL;
        switch (boxType)
        {
        case 1:
            // Cornell Box
            skyBoxSize.x = 20000.f;
            skyBoxSize.y = 20000.f;
            skyBoxSize.z = 20000.f;
            matFront = CORNELLBOX_FRONT_MATERIAL;
            matRight = CORNELLBOX_RIGHT_MATERIAL;
            matLeft = CORNELLBOX_LEFT_MATERIAL;
            matBack = CORNELLBOX_BACK_MATERIAL;
            matTop = CORNELLBOX_TOP_MATERIAL;
            matBottom = CORNELLBOX_BOTTOM_MATERIAL;
            break;
        case 2:
            // Mandelbrot Box
            skyBoxSize.x = 20000.f;
            skyBoxSize.y = 20000.f;
            skyBoxSize.z = 20000.f;
            matFront = MANDELBROT_MATERIAL;
            matRight = JULIA_MATERIAL;
            matLeft = MANDELBROT_MATERIAL;
            matBack = JULIA_MATERIAL;
            matTop = MANDELBROT_MATERIAL;
            matBottom = JULIA_MATERIAL;
            break;
        case 3:
            // uniform skybox
            tiles = 8.f;
            skyBoxSize.x = 20000.f;
            skyBoxSize.y = 20000.f;
            skyBoxSize.z = 20000.f;
            matFront = SKYBOX_GROUND_MATERIAL;  // CORNELLBOX_FRONT_MATERIAL;
            matRight = SKYBOX_GROUND_MATERIAL;  // CORNELLBOX_RIGHT_MATERIAL;
            matLeft = SKYBOX_GROUND_MATERIAL;   // CORNELLBOX_LEFT_MATERIAL;
            matBack = SKYBOX_GROUND_MATERIAL;   // CORNELLBOX_BACK_MATERIAL;
            matTop = SKYBOX_GROUND_MATERIAL;    // CORNELLBOX_TOP_MATERIAL;
            matBottom = SKYBOX_GROUND_MATERIAL; // CORNELLBOX_BOTTOM_MATERIAL;
            break;
        case 4:
            // SkyBox
            skyBoxSize.x = m_gpuKernel->getSceneInfo().viewDistance * 0.2f;
            skyBoxSize.y = m_gpuKernel->getSceneInfo().viewDistance * 0.2f;
            skyBoxSize.z = m_gpuKernel->getSceneInfo().viewDistance * 0.2f;
            skyBoxSize.x = 20000.f;
            skyBoxSize.y = 20000.f;
            skyBoxSize.z = 20000.f;
            break;
        }

        // Front
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
#ifdef _USE_KINECT
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
#else
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
#endif // USE_KINECT
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matFront);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
#ifdef _USE_KINECT
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
#else
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
#endif // USE_KINECT
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matFront);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // right
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glNormal3f(-1.f, 0.f, 0.f);
        glNormal3f(-1.f, 0.f, 0.f);
        glNormal3f(-1.f, 0.f, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matRight);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glNormal3f(-1.f, 0.f, 0.f);
        glNormal3f(-1.f, 0.f, 0.f);
        glNormal3f(-1.f, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matRight);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // Back
        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glNormal3f(0.f, 0.f, 1.f);
        glNormal3f(0.f, 0.f, 1.f);
        glNormal3f(0.f, 0.f, 1.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matBack);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glNormal3f(0.f, 0.f, 1.f);
        glNormal3f(0.f, 0.f, 1.f);
        glNormal3f(0.f, 0.f, 1.f);
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matBack);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // Left
        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glNormal3f(1.f, 0.f, 0.f);
        glNormal3f(1.f, 0.f, 0.f);
        glNormal3f(1.f, 0.f, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matLeft);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glNormal3f(1.f, 0.f, 0.f);
        glNormal3f(1.f, 0.f, 0.f);
        glNormal3f(1.f, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matLeft);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // Top
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glNormal3f(0.f, -1.f, 0.f);
        glNormal3f(0.f, -1.f, 0.f);
        glNormal3f(0.f, -1.f, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matTop);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(skyBoxSize.x, skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glNormal3f(0.f, -1.f, 0.f);
        glNormal3f(0.f, -1.f, 0.f);
        glNormal3f(0.f, -1.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matTop);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // Bottom
        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glNormal3f(0.f, 1.f, 0.f);
        glNormal3f(0.f, 1.f, 0.f);
        glNormal3f(0.f, 1.f, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(tiles, 0.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matBottom);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, -skyBoxSize.y - groundHeight, -skyBoxSize.z);
        glNormal3f(0.f, 1.f, 0.f);
        glNormal3f(0.f, 1.f, 0.f);
        glNormal3f(0.f, 1.f, 0.f);
        glTexCoord3f(tiles, tiles, 0.f);
        glTexCoord3f(0.f, tiles, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, matBottom);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        break;
    }
    case 5:
    {
        for (int i(0); i < 25; ++i)
        {
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
            m_gpuKernel->setPrimitive(m_nbPrimitives, rand() % 20000 - 10000.f, rand() % 20000 - 10000.f,
                                      rand() % 20000 - 10000.f, 500.f + rand() % 2000, 0.f, 0.f, 1000 + rand() % 30);
        }
        break;
    }
    case 6:
    {
        // Ground
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptCheckboard);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, groundHeight, 0.f, groundSize.x / 4.f, 0.f, groundSize.y / 4,
                                  RANDOM_MATERIALS_OFFSET);

        // Columns
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
        m_gpuKernel->setPrimitive(m_nbPrimitives, -5000.f, groundHeight - 1000.f, 5000.f, -4000.f, 8000.f, 5000.f,
                                  2000.f, 0.f, 0.f, RANDOM_MATERIALS_OFFSET + 1);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 5000.f, groundHeight - 1000.f, 5000.f, 4000.f, 8000.f, 5000.f, 2000.f,
                                  0.f, 0.f, RANDOM_MATERIALS_OFFSET + 2);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // Spheres
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, -5000.f, groundHeight + 1000.f, -5000.f, 1000.f, 0.f, 0.f,
                                  RANDOM_MATERIALS_OFFSET + 3);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 5000.f, groundHeight + 1000.f, -5000.f, 1000.f, 0.f, 0.f,
                                  RANDOM_MATERIALS_OFFSET + 4);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        // Walls
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptXYPlane);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, groundHeight + groundSize.y, groundSize.x, groundSize.x,
                                  groundSize.y, 0.f, RANDOM_MATERIALS_OFFSET + 5);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptYZPlane);
        m_gpuKernel->setPrimitive(m_nbPrimitives, -groundSize.x, groundHeight + groundSize.y, 0.f, 0.f, groundSize.x,
                                  groundSize.y, RANDOM_MATERIALS_OFFSET + 6);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptYZPlane);
        m_gpuKernel->setPrimitive(m_nbPrimitives, groundSize.x, groundHeight + groundSize.y, 0.f, 0.f, groundSize.x,
                                  groundSize.y, RANDOM_MATERIALS_OFFSET + 7);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        break;
    }
    case 7:
    {
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, 0.f, 0.f, skyBoxSize.x, 0.f, 0.f, SKYBOX_SPHERE_MATERIAL);
        const vec2f vt0 = make_vec2f(0.f, 0.f);
        const vec2f vt1 = make_vec2f(1.f, 1.f);
        const vec2f vt2 = make_vec2f(0.f, 0.f);
        m_gpuKernel->setPrimitiveTextureCoordinates(m_nbPrimitives, vt0, vt1, vt2);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        break;
    }
    case 8:
    {
        repeats = 8.f;
        for (int i(0); i < 15; ++i)
        {
            float radius = 700.f + float(rand() % 1000);
            int material = BASIC_REFLECTION_MATERIAL_002;
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
            m_gpuKernel->setPrimitive(m_nbPrimitives, rand() % int(skyBoxSize.x) - skyBoxSize.x / 2,
                                      m_groundHeight + radius, rand() % int(skyBoxSize.z) - skyBoxSize.z / 2, radius,
                                      0.f, 0.f, material);
            m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        }
        break;
    }
    case 9:
    {
        const float ratio = 9.f / 16.f;
        skyBoxSize.x = 50000.f;
        skyBoxSize.y = 50000.f;
        skyBoxSize.z = 5000.f;
        const int material = SKYBOX_SPHERE_MATERIAL;
        repeats = 3.f;
        glBegin(GL_TRIANGLES);
        glVertex3f(skyBoxSize.x, ratio * groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, ratio * groundHeight, skyBoxSize.z);
        glVertex3f(-skyBoxSize.x, skyBoxSize.y * ratio * 2.f + groundHeight, skyBoxSize.z);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
#ifdef _USE_KINECT
        glTexCoord3f(repeats, repeats, 0.f);
        glTexCoord3f(0.f, repeats, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
#else
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(repeats, 0.f, 0.f);
        glTexCoord3f(repeats, repeats, 0.f);
#endif // USE_KINECT
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, material);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        glBegin(GL_TRIANGLES);
        glVertex3f(-skyBoxSize.x, 2.f * ratio * skyBoxSize.y + groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, 2.f * ratio * skyBoxSize.y + groundHeight, skyBoxSize.z);
        glVertex3f(skyBoxSize.x, groundHeight, skyBoxSize.z);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
        glNormal3f(0.f, 0.f, -1.f);
#ifdef _USE_KINECT
        glTexCoord3f(0.f, 0.f, 0.f);
        glTexCoord3f(repeats, 0.f, 0.f);
        glTexCoord3f(repeats, repeats, 0.f);
#else
        glTexCoord3f(repeats, repeats, 0.f);
        glTexCoord3f(0.f, repeats, 0.f);
        glTexCoord3f(0.f, 0.f, 0.f);
#endif // USE_KINECT
        m_nbPrimitives = glEnd();
        m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, material);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        repeats = 40.f;
    }
    case 10:
    {
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, groundHeight - 1000000, 0.f, 1000000.f, 0.f, 0.f,
                                  SKYBOX_GROUND_MATERIAL);
    }
    }

    if (boxType == 1 || boxType == 2 || (boxType >= 3 && boxType <= 9))
    {
        int material = SKYBOX_GROUND_MATERIAL;
        // Ground
        {
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptTriangle);
            m_gpuKernel->setPrimitive(m_nbPrimitives, -groundSize.x, groundHeight, -groundSize.y, groundSize.x,
                                      groundHeight, -groundSize.y, groundSize.x, groundHeight, groundSize.y, 0.f, 0.f,
                                      0.f, material);
            const vec3f n = make_vec3f(0.f, 1.f, 0.f);
            m_gpuKernel->setPrimitiveNormals(m_nbPrimitives, n, n, n);
            const vec2f vt0 = make_vec2f();
            const vec2f vt1 = make_vec2f(repeats, 0.f);
            const vec2f vt2 = make_vec2f(repeats, repeats);
            m_gpuKernel->setPrimitiveTextureCoordinates(m_nbPrimitives, vt0, vt1, vt2);
            m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        }
        {
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptTriangle);
            m_gpuKernel->setPrimitive(m_nbPrimitives, groundSize.x, groundHeight, groundSize.y, -groundSize.x,
                                      groundHeight, groundSize.y, -groundSize.x, groundHeight, -groundSize.y, 0.f, 0.f,
                                      0.f, material);
            const vec3f n = make_vec3f(0.f, 1.f, 0.f);
            m_gpuKernel->setPrimitiveNormals(m_nbPrimitives, n, n, n);
            const vec2f vt0 = make_vec2f(repeats, repeats);
            const vec2f vt1 = make_vec2f(0.f, repeats);
            const vec2f vt2 = make_vec2f(0.f, 0.f);
            m_gpuKernel->setPrimitiveTextureCoordinates(m_nbPrimitives, vt0, vt1, vt2);
            m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
        }
    }
}

/*
________________________________________________________________________________

Worms
________________________________________________________________________________
 */

void Scene::createWorm(const vec3f &center, int boxId, int material)
{
    // Worm on the right hand side
    int m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y, center.z, 800.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);

    // Eyes
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 400.f, center.y + 500.f, center.z - 400.f, 150.f, 0.f, 0.f,
                              12);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 400.f, center.y + 500.f, center.z - 400.f, 150.f, 0.f, 0.f,
                              12);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 400.f, center.y + 500.f, center.z - 400.f, 200.f, 0.f, 0.f,
                              13);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 400.f, center.y + 500.f, center.z - 400.f, 200.f, 0.f, 0.f,
                              13);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);

    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 700.f, center.z, 600.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 1200.f, center.z + 200.f, 600.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 1500.f, center.z + 500.f, 600.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 1500.f, center.z + 1000.f, 600.f, 0.f, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 1500.f, center.z + 1500.f, 600.f, 0.f, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 1300.f, center.z + 2000.f, 400.f, 0.f, 0.f,
                              material);

    // Arms
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 700.f, center.z, center.x - 500.f, center.y - 1200.f,
                              center.z - 1000.f, 100.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 500.f, center.y - 1200.f, center.z - 1000.f, 300.f, 0.f, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x, center.y - 700.f, center.z, center.x + 500.f, center.y - 1200.f,
                              center.z - 1000.f, 100.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 500.f, center.y - 1200.f, center.z - 1000.f, 300.f, 0.f, 0.f,
                              material);
}

/*
________________________________________________________________________________

Create doggy
________________________________________________________________________________
 */
void Scene::createDog(const vec3f &center, int material, float size, int boxid)
{
    // Body
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1000, center.y, center.z, center.x + 1000, center.y, center.z,
                              size, 0, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1000, center.y, center.z, size, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1000, center.y, center.z, size, 0.f, 0.f, material);

    // Legs
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1500, center.y - 2000, center.z - 1000, center.x - 1000,
                              center.y, center.z, size, 0, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1500, center.y - 2000, center.z + 1000, center.x - 1000,
                              center.y, center.z, size, 0, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1500, center.y - 2000, center.z - 1000, center.x + 1000,
                              center.y, center.z, size, 0, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1500, center.y - 2000, center.z + 1000, center.x + 1000,
                              center.y, center.z, size, 0, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1500, center.y - 2000, center.z - 1000, size, 0, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1500, center.y - 2000, center.z + 1000, size, 0, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1500, center.y - 2000, center.z - 1000, size, 0, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1500, center.y - 2000, center.z + 1000, size, 0, 0.f,
                              material);

    // Tail
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1500, center.y + 1000, center.z, center.x - 1000, center.y,
                              center.z, size * 0.8f, 0, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x - 1500, center.y + 1000, center.z, size * 0.8f, 0, 0.f,
                              material);

    // Head
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1000.f, center.y, center.z, center.x + 1500.f,
                              center.y + 1000.f, center.z, size, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1500.f, center.y + 1500.f, center.z, size * 2.f, 0.f, 0.f,
                              material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 1500.f, center.y + 1500.f, center.z, center.x + 2500.f,
                              center.y + 1000.f, center.z, size * 2.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, center.x + 2500.f, center.y + 1000.f, center.z, size * 2.f, 0.f, 0.f,
                              material);
}

void Scene::renderText()
{
}

SceneInfo &Scene::getSceneInfo()
{
    return m_gpuKernel->getSceneInfo();
}

std::string &Scene::getName()
{
    return m_name;
}

void Scene::createSkeleton()
{
#ifdef USE_KINECT
    vec4f skyBoxSize = {1600.f, 1200.f, 0.f};
    vec4f skyBoxPosition = {3000.f, m_groundHeight - 450.f, 2500.f};
    glBegin(GL_TRIANGLES);
    glVertex3f(skyBoxPosition.x + skyBoxSize.x, skyBoxPosition.y - skyBoxSize.y - m_groundHeight,
               skyBoxPosition.z + skyBoxSize.z);
    glVertex3f(skyBoxPosition.x - skyBoxSize.x, skyBoxPosition.y - skyBoxSize.y - m_groundHeight,
               skyBoxPosition.z + skyBoxSize.z);
    glVertex3f(skyBoxPosition.x - skyBoxSize.x, skyBoxPosition.y + skyBoxSize.y - m_groundHeight,
               skyBoxPosition.z + skyBoxSize.z);
    glNormal3f(0.f, 0.f, -1.f);
    glNormal3f(0.f, 0.f, -1.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord3f(1.f, 1.f, 0.f);
    glTexCoord3f(0.f, 1.f, 0.f);
    glTexCoord3f(0.f, 0.f, 0.f);
    m_nbPrimitives = glEnd();
    m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, KINECT_COLOR_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

    glBegin(GL_TRIANGLES);
    glVertex3f(skyBoxPosition.x - skyBoxSize.x, skyBoxPosition.y + skyBoxSize.y - m_groundHeight,
               skyBoxPosition.z + skyBoxSize.z);
    glVertex3f(skyBoxPosition.x + skyBoxSize.x, skyBoxPosition.y + skyBoxSize.y - m_groundHeight,
               skyBoxPosition.z + skyBoxSize.z);
    glVertex3f(skyBoxPosition.x + skyBoxSize.x, skyBoxPosition.y - skyBoxSize.y - m_groundHeight,
               skyBoxPosition.z + skyBoxSize.z);
    glNormal3f(0.f, 0.f, -1.f);
    glNormal3f(0.f, 0.f, -1.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord3f(0.f, 0.f, 0.f);
    glTexCoord3f(1.f, 0.f, 0.f);
    glTexCoord3f(1.f, 1.f, 0.f);
    m_nbPrimitives = glEnd();
    m_gpuKernel->setPrimitiveMaterial(m_nbPrimitives, KINECT_COLOR_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

    for (int i(0); i < 20; ++i)
    {
        int primitive = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(primitive, 0.f, 0.f, 0.f, m_skeletonSize, 0.f, 0.f, 0);
        if (i == 0)
            m_skeletonPrimitiveIndex = primitive;
    }
#endif // USE_KINECT
}
/*
________________________________________________________________________________

animateSkeleton
________________________________________________________________________________
 */
void Scene::animateSkeleton()
{
#ifdef USE_KINECT
    m_skeletonPosition.y = m_groundHeight;
    int hr = m_gpuKernel->updateSkeletons(m_skeletonPrimitiveIndex,
                                          m_skeletonPosition,             // Position
                                          m_skeletonSize,                 // Skeleton size
                                          m_skeletonThickness, 40,        // Default material
                                          m_skeletonThickness * 2.0f, 41, // Head size and material
                                          m_skeletonThickness * 1.5f, 42, // Hands size and material
                                          m_skeletonThickness * 1.8f, 43  // Feet size and material
                                          );
    m_gpuKernel->getSceneInfo().pathTracingIteration = 0;

    if (hr == S_OK)
    {
        vec3f position;
        // Head
        if (m_gpuKernel->getSkeletonPosition(NUI_SKELETON_POSITION_HEAD, position))
        {
            vec4f amplitude;
            amplitude.x = (position.x * m_skeletonKinectSize - m_gpuKernel->getViewPos().x) / 2.f;
            amplitude.y = (position.y * m_skeletonKinectSize - m_gpuKernel->getViewPos().y) / 2.f;
            amplitude.z = ((100.f + 200.f - position.z * m_skeletonKinectSize) - m_gpuKernel->getViewPos().z) / 2.f;

            m_gpuKernel->getViewPos().x += amplitude.x;
            m_gpuKernel->getViewPos().y += amplitude.y;
            m_gpuKernel->getViewPos().z += amplitude.z;

            m_gpuKernel->getViewDir().x += amplitude.x;
            m_gpuKernel->getViewDir().y += amplitude.y;
            m_gpuKernel->getViewDir().z += amplitude.z;

            float a = (position.x - m_skeletonOldPosition.x);
            if (fabs(a) <= 1.f)
                m_gpuKernel->getViewAngles().y -= asin(a);
            a = (position.y - m_skeletonOldPosition.y);
            if (fabs(a) <= 1.f)
                m_gpuKernel->getViewAngles().x += -asin(a);

            m_skeletonOldPosition = position;
        }
    }
#endif // USE_KINECT

#ifdef USE_SIXENSE
    // Sixense
    int left_index = 0; // TODO:
    // sixenseUtils::getTheControllerManager()->getIndex(
    // sixenseUtils::ControllerManager::P1L );
    int right_index = 1; // TODO:
    // sixenseUtils::getTheControllerManager()->getIndex(
    // sixenseUtils::ControllerManager::P1R );
    // Go through each of the connected systems
    for (int base = 0; base < sixenseGetMaxBases(); base++)
    {
        sixenseSetActiveBase(base);
        sixenseAllControllerData acd;
        static sixenseUtils::ButtonStates left_states, right_states;

        // Get the latest controller data
        sixenseGetAllNewestData(&acd);
        // For each possible controller
        for (int cont = 0; cont < sixenseGetMaxControllers(); cont++)
        {
            // See if it's enabled
            if (sixenseIsControllerEnabled(cont))
            {
                LOG_INFO(3, "Controller " << cont << ", left=" << left_index << ", right=" << right_index);
                if (cont == left_index)
                {
                    // if this is the left controller
                    vec4f angles;
                    angles.x = PI * (m_modelRotationAngle.x - acd.controllers[cont].rot_quat[0]);
                    angles.y = PI * (m_modelRotationAngle.y - acd.controllers[cont].rot_quat[1]);
                    angles.z = PI * (m_modelRotationAngle.z + acd.controllers[cont].rot_quat[2]);

                    vec3f translation;
                    translation.x = 10.f * (acd.controllers[cont].pos[0] - m_modelTranslation.x);
                    translation.y = 10.f * (acd.controllers[cont].pos[1] - m_modelTranslation.y);
                    translation.z = 10.f * (m_modelTranslation.z - acd.controllers[cont].pos[2]);

                    m_gpuKernel->getSceneInfo().pathTracingIteration = 0;
                    m_gpuKernel->rotatePrimitives(m_modelPosition, angles);
                    m_gpuKernel->translatePrimitives(translation);
                    m_gpuKernel->compactBoxes(false);

                    m_modelRotationAngle.x = acd.controllers[cont].rot_quat[0];
                    m_modelRotationAngle.y = acd.controllers[cont].rot_quat[1];
                    m_modelRotationAngle.z = -acd.controllers[cont].rot_quat[2];

                    m_modelTranslation.x = acd.controllers[cont].pos[0];
                    m_modelTranslation.y = acd.controllers[cont].pos[1];
                    m_modelTranslation.z = acd.controllers[cont].pos[2];

                    m_modelPosition.x += translation.x;
                    m_modelPosition.y += translation.y;
                    m_modelPosition.z += translation.z;

                    // Buttons
                    left_states.update(&acd.controllers[cont]);
                    if (left_states.buttonJustPressed(SIXENSE_BUTTON_2))
                        createRandomMaterials(true, false);
                    if (left_states.buttonJustPressed(SIXENSE_BUTTON_3))
                        m_gpuKernel->getSceneInfo().graphicsLevel =
                            static_cast<GraphicsLevel>((m_gpuKernel->getSceneInfo().graphicsLevel + 1) % 5);
                    if (left_states.buttonJustPressed(SIXENSE_BUTTON_4))
                    {
                        m_gpuKernel->getPostProcessingInfo().type++;
                        m_gpuKernel->getPostProcessingInfo().type = m_gpuKernel->getPostProcessingInfo().type % 4;
                    }

                    // Joystick / Camera
                    m_gpuKernel->getViewPos().x += 100.f * acd.controllers[cont].joystick_x;
                    m_gpuKernel->getViewDir().x += 100.f * acd.controllers[cont].joystick_x;
                    m_gpuKernel->getViewPos().z += 100.f * acd.controllers[cont].joystick_y;
                    m_gpuKernel->getViewDir().z += 100.f * acd.controllers[cont].joystick_y;
                }

                if (cont == right_index)
                {
                    m_gpuKernel->getSceneInfo().eyeSeparation += 50.f * acd.controllers[cont].joystick_x;
                    m_gpuKernel->getViewDir().z += 50.f * acd.controllers[cont].joystick_y;
                }
            }
        }
    }
#endif // USE_SIXENSE

#ifdef USE_KINECT
    // Reconstruct acceleration structures
    m_gpuKernel->compactBoxes(false);
#endif // USE_KINECT
}
