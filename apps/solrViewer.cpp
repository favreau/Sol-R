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

// OpenGL Graphics Includes
#include "../solr/Logging.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

// Includes
#ifdef WIN32
#include <windows.h>
#else
#include <math.h>
#include <sstream>
#endif

// SolR
#include <solr/images/jpge.h>

// Project
#include <scenes/animation/AnimationScene.h>
#include <scenes/animation/MetaballsScene.h>
#include <scenes/animation/WaterScene.h>
#include <scenes/experiments/CornellBoxScene.h>
#include <scenes/experiments/CubesScene.h>
#include <scenes/experiments/CylinderScene.h>
#include <scenes/experiments/DoggyStyleScene.h>
#include <scenes/experiments/GalleryScene.h>
#include <scenes/experiments/GraphScene.h>
#include <scenes/experiments/PerpetualMotionScene.h>
#include <scenes/experiments/TransparentScene.h>
#include <scenes/experiments/XmasScene.h>
#include <scenes/experiments/Year2013.h>
#include <scenes/games/SpindizzyScene.h>
#include <scenes/maths/FractalScene.h>
#include <scenes/maths/FractalsScene.h>
#include <scenes/maths/TrefoilKnotScene.h>
#include <scenes/meshes/ObjScene.h>
#include <scenes/meshes/TrianglesScene.h>
#include <scenes/science/HypercubeScene.h>
#include <scenes/science/MoleculeScene.h>
#include <scenes/science/SwcScene.h>
#include <scenes/science/TesseractScene.h>

// Ray-tracing Kernel
solr::GPUKernel *gKernel = solr::SingletonKernel::kernel();

// General Settings
const int TARGET_FPS = 60;
const int REFRESH_DELAY = 1000 / TARGET_FPS; // ms

// ----------------------------------------------------------------------
// Scene
// ----------------------------------------------------------------------
bool gCopyright(true);
int gSceneId(0);
bool gAutoFocus(false);
bool gAnimate(false);
vec4f gBkColor = make_vec4f();
bool gDraft(false);
int gSphereMaterial = 0;
int gGroundMaterial = 0;
bool gSuspended(false);
bool gSavedToDisk(false);
std::string gHint;

#ifdef WIN32
ULONGLONG gRenderingTime;
#endif

// ----------------------------------------------------------------------
// Benchmark
// ----------------------------------------------------------------------
#ifdef WIN32
int gTickCount = 0;
#endif // WIN32
bool gBenchmarking(false);

// Oculus
float gDistortion = 0.1f;

// GPU
int gDrawing(false);

// Command line arguments
std::string gFilename;

// Rendering window vars
#ifdef USE_OCULUS
unsigned int gWindowWidth = 1280 / 2;
unsigned int gWindowHeight = 800 / 2;
#else
unsigned int gWindowWidth = 960;
unsigned int gWindowHeight = 540;
#endif // USE_OCULUS
unsigned int gWindowDepth = 4;

// Scene
Scene *gScene = 0;

// Menu
struct MenuItem
{
    std::string description;
    char key;
};
const int NB_MENU_ITEMS = 22;
MenuItem menuItems[NB_MENU_ITEMS] = {
    {"A: Black background and no image noise", 'A'},
    {"b: Randomly set background color and image noise", 'b'},
    {"f: Auto-focus", 'f'},
    {"h: Help", 'h'},
    {"i: Show/hide Bounding boxes", 'i'},
    {"m: Animate scene", 'm'},
    {"n: Load next 3D model (in folder medias/obj)", 'n'},
    {"o: Switch to VR mode", 'o'},
    {"p: Post processing (Depth of field, ambient occlusion, bloom, etc)", 'p'},
    {"r: Reset current scene", 'r'},
    {"t: Next scene", 't'},
    {"s: Change graphics level", 's'},
    {"v: Random materials", 'i'},
    {"x: Next environment (CornellBox, SkyBox, etc.)", 'x'},
    {"*: View modes (Standard, Anaglyth 3D, Oculus Rift", '*'},
    {"1: Camera mouse control", '1'},
    {"2: Light mouse control", '2'},
    {"3: 3D model mouse control", '3'},
    {"4: Depth of field mouse control", '3'},
    {"5: Field of view mouse control", '3'},
    {"9: Shadow strength", '9'},
    {"Esc: Exit application", '\033'}};

// controlType
enum ControlType
{
    ctCamera = 0,
    ctLightSource = 1,
    ctObject = 2,
    ctFocus = 3,
    ctFOV = 4,
    ct3DVision = 5,
    ctGroundHeight = 6
};
ControlType gControlType = ctCamera;
int gSelectedPrimitive = -1;

// Materials
int currentMaterial = 0;
int gMaterials[2];
int gWallTexture = 3;

// Scene description and behavior
int gCornellBoxType = 0;
int gNbBoxes = 0;
int m_nbPrimitives = 0;
int gLampId = 0;
int gNbMaterials = 0;
int gNbTextureTiles = 1;

vec4f gRotationAngles = make_vec4f();
vec4f gViewAngles = make_vec4f(0.f, 0.f, 0.f, 6400.f);
vec3f gViewPos = make_vec3f(0.f, 0.f, -15000.f);
vec3f gViewDir = make_vec3f();
PostProcessingInfo gPostProcessingInfo = {ppe_none, 0.f, 10.f, 1000};

float groundHeight = -3000.f;

// materials
float gReflection = 0.f;
float gRefraction = 1.33f; // Water
float gTransparency = 0.9f;
float gSpecValue = 1.f;
float gSpecPower = 100.f;
float gSpecCoef = 1.f;
float gInnerIllumination = 0.f;

// OpenGL
int gTimebase(0);
int gFrame(0);
int gFPS(0);

// GL functionality
void initgl(int argc, char **argv);
void display();
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void timerEvent(int value);
void reshape(int x, int y);
void createScene();

#ifdef WIN32
void RenderString(float x, float y, float z, void *font,
                  const std::string &string, const vec4f &rgb);
#endif

// Helpers
bool gHelp(false);
void Cleanup(int iExitCode);
void (*pCleanup)(int) = &Cleanup;

// Sim and Auto-Verification parameters
float anim = 0.f;
bool bNoPrompt = false;

// mouse controls
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float rotate_x = 0.0, rotate_y = 0.0;

unsigned int *uiOutput = 0;

void readParameters(const std::vector<std::string> &parameters)
{
    std::vector<std::string>::const_iterator it = parameters.begin();
    while (it != parameters.end())
    {
        std::string argument(*it);
        size_t eqPos = argument.find("=");
        if (eqPos != std::string::npos)
        {
            std::string key(argument.substr(0, eqPos));
            std::string value(argument.substr(eqPos + 1));
            LOG_INFO(1, "Argument: " << key << " = " << value);
#ifdef USE_OPENCL
            if (key.find("-platform") != std::string::npos)
                gKernel->setPlatformId(atoi(value.c_str()));
            if (key.find("-device") != std::string::npos)
                gKernel->setDeviceId(atoi(value.c_str()));
            if (key.find("-opencl-kernel") != std::string::npos)
                gKernel->setKernelFilename(value);
#endif // USE_OPENCL
            if (key.find("-objFile") != std::string::npos)
                gFilename = value.c_str();
            if (key.find("-width") != std::string::npos)
                gWindowWidth = atoi(value.c_str());
            if (key.find("-height") != std::string::npos)
                gWindowHeight = atoi(value.c_str());
            if (key.find("-benchmark") != std::string::npos)
                gBenchmarking = (atoi(value.c_str()) == 1);
            if (key.find("-scene") != std::string::npos)
                gSceneId = atoi(value.c_str());
            if (key.find("-cornellBox") != std::string::npos)
                gCornellBoxType = atoi(value.c_str());
        }
        ++it;
    }
}

void initgl(int argc, char **argv)
{
    glutInit(&argc, (char **)argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowPosition(glutGet(GLUT_SCREEN_WIDTH) / 2 - gWindowWidth / 2,
                           glutGet(GLUT_SCREEN_HEIGHT) / 2 - gWindowHeight / 2);
    std::string caption("SoL-R tech demo v00.02.00");
#ifdef USE_CUDA
    caption += " (Powered by CUDA)";
    glutInitWindowSize(gWindowWidth, gWindowHeight);
#endif
#ifdef USE_OPENCL
    caption += " (Powered by OpenCL)";
    glutInitWindowSize(gWindowWidth, gWindowHeight);
#endif
    glutCreateWindow(caption.c_str());
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutReshapeFunc(reshape);
    glutTimerFunc(REFRESH_DELAY, timerEvent, 1);
}

#ifdef WIN32
void RenderString(float x, float y, float z, void *font,
                  const std::string &string, const vec4f &rgb)
{
    // glColor3f(rgb.x, rgb.y, rgb.z);
    glRasterPos3f(x, y, z);
    glutBitmapString(font,
                     reinterpret_cast<const unsigned char *>(string.c_str()));
}
#endif

void display()
{
    if (gSuspended)
        return;
    // clear graphics
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    gFrame++;
    int time = glutGet(GLUT_ELAPSED_TIME);
    int processingTime = time - gTimebase;
    if (processingTime > 1000)
    {
        gFPS = gFrame * 1000 / processingTime;
        gTimebase = time;
        gFrame = 0;
    }

    if (!gDrawing)
    {
        gDrawing = true;

        SceneInfo &si = gScene->getSceneInfo();
        if (si.pathTracingIteration == 0)
        {
#ifdef WIN32
            gRenderingTime = GetTickCount64();
#endif
            gSavedToDisk = false;
        };
#ifdef WIN32
        if (si.pathTracingIteration == si.maxPathTracingIterations - 1)
        {
            gRenderingTime = GetTickCount64() - gRenderingTime;
        }
#endif

        if (si.pathTracingIteration < si.maxPathTracingIterations)
        {
            gScene->getKernel()->setCamera(gViewPos, gViewDir, gViewAngles);
            gScene->getKernel()->setPostProcessingInfo(gPostProcessingInfo);
            gScene->render(gAnimate);
            if (gAnimate)
            {
                si.pathTracingIteration = 0;
            }
            else
            {
                si.pathTracingIteration++;
            }
#ifdef WIN32
            if (gHelp)
            {
                char tmp[1024];
                std::string tmpMenuItems;
                for (int i(0); i < NB_MENU_ITEMS; ++i)
                {
                    tmpMenuItems += menuItems[i].description;
                    tmpMenuItems += "\n";
                }
                vec4f textColor = {1.f, 1.f, 1.f};
                sprintf(
                    tmp,
                    "%sSelected primitive: %d\nFPS: %d on %s (%dx%d)\nScene "
                    "%d: %s\nMouse control: %s\nPrimitives/Boxes: "
                    "%d/%d\nhttp://cudaopencl.blogspot.com [%d]",
                    tmpMenuItems.c_str(), gSelectedPrimitive, gFPS,
                    gKernel->getGPUDescription().c_str(),
                    gScene->getSceneInfo().size.x,
                    gScene->getSceneInfo().size.y, gSceneId,
                    gScene->getName().c_str(), gHint.c_str(),
                    gKernel->getNbActivePrimitives(),
                    gKernel->getNbActiveBoxes(), si.pathTracingIteration);
                RenderString(-0.9f, 0.9f, 0.f, GLUT_BITMAP_HELVETICA_10, tmp,
                             textColor);
            }
#endif // WIN32
        }
        else
        {
            gScene->getKernel()->render_end();

            if (gBenchmarking)
                Cleanup(EXIT_SUCCESS);
        }

        // Screenshot
        if (si.pathTracingIteration == si.maxPathTracingIterations)
        {
#ifdef WIN32
            // Rendering time
            std::stringstream text;
            text << "Rendered in ";
            text << gRenderingTime << " ms\n";
            text << gKernel->getGPUDescription();
            text << "\n"
                 << gKernel->getNbActivePrimitives() << " triangles, "
                 << si.size.x << "x" << si.size.y << ", "
                 << si.maxPathTracingIterations << " samples/pixel";
            RenderString(-0.95f, -0.8f, 0.f, GLUT_BITMAP_HELVETICA_10,
                         text.str().c_str(), make_vec4f(1.f, 1.f, 1.f));
            gScene->renderText();
#endif

            if (!gSavedToDisk)
            {
                int margin = 32;
                size_t size =
                    (si.size.x - margin) * (si.size.y - margin) * gColorDepth;
                GLubyte *buffer = new GLubyte[size];
                glReadPixels(0, 0, si.size.x - margin, si.size.y - margin,
                             GL_RGB, GL_UNSIGNED_BYTE, buffer);
                GLubyte *dst = new GLubyte[size];
                int X = (si.size.x - margin) * gColorDepth;

                // Vertical flip
                for (int y(0); y < (si.size.y - margin); ++y)
                {
                    int idxSrc = y * X;
                    memcpy(dst + idxSrc, buffer + (size - idxSrc), X);
                }

                // Save to disc
                std::string filename("./SolR");
                filename += ".jpg";
                jpge::compress_image_to_jpeg_file(filename.c_str(),
                                                  si.size.x - margin,
                                                  si.size.y - margin,
                                                  gColorDepth, dst);
                delete[] buffer;
                delete[] dst;
                gSavedToDisk = true;
            }
        }

// TODO: Copyright
#ifdef WIN32
        if (gCopyright)
        {
            // Copyright
            const char *copyright = "http://cudaopencl.blogspot.com";
            float p = strlen(copyright) * 20.f / si.size.x;
            RenderString(-p / 2.f, -0.9f, 0.f, GLUT_BITMAP_HELVETICA_18,
                         copyright, make_vec4f(1.f, 1.f, 1.f));
            gScene->renderText();
        }

#endif
        glutSwapBuffers();
        gDrawing = false;
    }
}

void timerEvent(int value)
{
#ifdef _USE_KINECT
    animateSkeleton();
#endif // USE_KINECT
#ifdef _USE_KINECT
case 10: // Kinect
{
    int p = 0;
    int b = 0;
    char *depthBitmap = kernel.getDepthBitmap();
    if (depthBitmap != 0)
    {
        for (int y(0); y < KINECT_DEPTH_HEIGHT; y += gKinectStep)
        {
            for (int x(0); x < KINECT_DEPTH_WIDTH; x += gKinectStep)
            {
                int index = KINECT_DEPTH_SIZE -
                            (y * KINECT_DEPTH_WIDTH + x) * KINECT_DEPTH_DEPTH;
                char a = depthBitmap[index];
                char b = depthBitmap[index + 1];
                int s = b * 256 + a;

                USHORT player = s & 7;
                if (s < 1 || s > 15000)
                    s = 50000;

                kernel.setPrimitive(p, 5 + p / gKinectNbSpherePerBox,
                                    -(x - 160) * gKinectSpace,
                                    (y - 120) * gKinectSpace, s / 3.f - 5000.f,
                                    gKinectSize, 0.f, 0.f, 7, 1, 1);

                ++b;
                ++p;
            }
        }
    }
    kernel.compactBoxes();
    break;
}
#endif // USE_KINECT
    glutPostRedisplay();
    glutTimerFunc(REFRESH_DELAY, timerEvent, 0);
}

void reshape(int x, int y)
{
    if (x % 32 != 0)
        x += 32 - (x % 32);
    if (y % 32 != 0)
        y += 32 - (y % 32);
    // y = x*9/16;
    SceneInfo &sceneInfo = gScene->getSceneInfo();
    sceneInfo.size.x = x;
    sceneInfo.size.y = y;
    sceneInfo.pathTracingIteration = 0;
    gScene->getKernel()->reshape();
    glViewport(0, 0, x, y);
    gWindowWidth = x;
    gWindowHeight = y;
}

void mainMenu(int i)
{
    keyboard((unsigned char)i, 0, 0);
}

void initMenus()
{
    glutCreateMenu(mainMenu);
    for (int i(0); i < NB_MENU_ITEMS; ++i)
        glutAddMenuEntry(menuItems[i].description.c_str(), menuItems[i].key);
    glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

void keyboard(unsigned char key, int x, int y)
{
    solr::GPUKernel *kernel = gScene->getKernel();
    if (!kernel)
        return;

    switch (key)
    {
    case 'A':
    {
        gScene->getSceneInfo().backgroundColor = gBkColor;
        break;
    }
    case 'a':
    {
        // Translate camera position
        gViewPos.x -= 100.f;
        gViewDir.x -= 100.f;
        break;
    }
    case 'B':
    {
        gScene->getSceneInfo().backgroundColor.x = 0.1f;
        gScene->getSceneInfo().backgroundColor.y = 0.1f;
        gScene->getSceneInfo().backgroundColor.z = 0.1f;
        gScene->getSceneInfo().backgroundColor.w = 0.5f;
        gScene->getSceneInfo().skyboxMaterialId =
            (gScene->getSceneInfo().skyboxMaterialId == MATERIAL_NONE)
                ? SKYBOX_SPHERE_MATERIAL
                : MATERIAL_NONE;
        break;
    }
    case 'b':
    {
        gScene->getSceneInfo().backgroundColor =
            make_vec4f(rand() % 255 / 255.f, rand() % 255 / 255.f,
                       rand() % 255 / 255.f, 0.5f);
        break;
    }
    case 'D':
    {
        // Draft mode
        gDraft = !gDraft;
        break;
    }
    case 'd':
    {
        // Translate camera position
        gViewPos.x += 100.f;
        gViewDir.x += 100.f;
        break;
    }
    case 'E':
    {
        // Extended geometry
        gScene->getSceneInfo().extendedGeometry =
            !gScene->getSceneInfo().extendedGeometry;
        break;
    }
    case 'e':
    {
        int nbTextures = kernel->getNbActiveTextures();
        int nbHDRI = gScene->getNbHDRI();
        if (nbTextures >= nbHDRI)
        {
            Material *m = kernel->getMaterial(SKYBOX_SPHERE_MATERIAL);
            gSphereMaterial = (gSphereMaterial + 1) % nbHDRI;
            kernel->setMaterial(SKYBOX_SPHERE_MATERIAL, m->color.x, m->color.y,
                                m->color.z, m->color.w, 0.f, 0.f,
                                (m->attributes.y == 1), (m->attributes.z == 1),
                                m->attributes.w, 0.f, m->opacity,
                                gSphereMaterial, TEXTURE_NONE, TEXTURE_NONE,
                                TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
                                TEXTURE_NONE, m->specular.x, m->specular.y,
                                m->specular.z, m->innerIllumination.x,
                                m->innerIllumination.y, m->innerIllumination.z,
                                (m->attributes.x == 1));
        }
        break;
    }
    case 'F':
    {
        // Toggle to full screen mode
        glutFullScreen();
        break;
    }
    case 'f':
    {
        gAutoFocus = !gAutoFocus;
        break;
    }
    case 'G':
    {
        gScene->getSceneInfo().nbRayIterations = NB_MAX_ITERATIONS;
        break;
    }
    case 'g':
    {
        int nbTextures =
            (kernel->getNbActiveTextures() - gScene->getNbHDRI()) / 5;
        if (nbTextures > 0)
        {
            gGroundMaterial = (gGroundMaterial + 1) % nbTextures;
            int index = gScene->getNbHDRI() + 5 * gGroundMaterial;

            Material *m = kernel->getMaterial(SKYBOX_GROUND_MATERIAL);
            int bumpTextureId = MATERIAL_NONE;
            int normalTextureId = MATERIAL_NONE;
            int diffuseTextureId = MATERIAL_NONE;
            int specularTextureId = MATERIAL_NONE;
            int reflectionTextureId = MATERIAL_NONE;
            int transparencyTextureId = MATERIAL_NONE;
            for (int t(0); t < 5; ++t)
            {
                int idx = index + t;
                TextureInfo ti = kernel->getTextureInformation(idx);
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
                case tex_transparent:
                    transparencyTextureId = idx;
                    break;
                case tex_specular:
                    specularTextureId = idx;
                    break;
                case tex_reflective:
                    reflectionTextureId = idx;
                    break;
                default:
                    break;
                }
            }
            if (transparencyTextureId == TEXTURE_NONE)
            {
                m->transparency = 0.f;
                m->refraction = 0.f;
            }
            else
            {
                m->transparency = 1.f;
                m->refraction = 1.1f;
            }
            kernel->setMaterial(SKYBOX_GROUND_MATERIAL, m->color.x, m->color.y,
                                m->color.z, m->color.w, 1.f, m->refraction,
                                (m->attributes.y == 1), (m->attributes.z == 1),
                                m->attributes.w, m->transparency, m->opacity,
                                diffuseTextureId, normalTextureId,
                                bumpTextureId, specularTextureId,
                                reflectionTextureId, transparencyTextureId,
                                TEXTURE_NONE, m->specular.x, m->specular.y,
                                m->specular.z, m->innerIllumination.x,
                                m->innerIllumination.y, m->innerIllumination.z,
                                (m->attributes.x == 1));
        }
        break;
    }
    case 'h':
    {
        gHelp = !gHelp;
        break;
    }
    case 'I':
    {
        gScene->getSceneInfo().advancedIllumination =
            static_cast<AdvancedIllumination>(
                (gScene->getSceneInfo().advancedIllumination + 1) % 3);
        break;
    }
    case 'i':
    {
        gScene->getSceneInfo().renderBoxes++;
        if (gScene->getSceneInfo().renderBoxes == 2)
            gScene->getSceneInfo().renderBoxes = 0;
        LOG_INFO(1, "Render Boxes: " << gScene->getSceneInfo().renderBoxes);
        break;
    }
    case 'k':
    {
        gScene->getSceneInfo().gradientBackground =
            !gScene->getSceneInfo().gradientBackground;
        break;
    }
    case 'K':
    {
        // Reset scene
        gSuspended = !gSuspended;
        break;
    }
    case 'L':
    {
        kernel->resetAll();
        kernel->loadFromFile("test.irt");
        kernel->compactBoxes(true);
        break;
    }
    case 'm':
    {
        gAnimate = !gAnimate;
        break;
    }
    case 'n':
    {
        // Reset scene
        delete gScene;
        gScene = 0;
        m_counter++;
        createScene();
        break;
    }
    case 'O':
    {
        gScene->getKernel()->switchOculusVR();
        break;
    }
    case 'o':
    {
        gScene->getSceneInfo().cameraType =
            (gScene->getSceneInfo().cameraType == ctVolumeRendering)
                ? ctPerspective
                : ctVolumeRendering;
        gScene->getSceneInfo().graphicsLevel =
            (gScene->getSceneInfo().cameraType == ctVolumeRendering)
                ? glNoShading
                : glReflectionsAndRefractions;
        gScene->getSceneInfo().backgroundColor.w =
            (gScene->getSceneInfo().cameraType == ctVolumeRendering) ? 0.01f
                                                                     : 0.5f;
        break;
    }
    case 'p':
    {
        gPostProcessingInfo.type++;
        gPostProcessingInfo.type %= 6;
        break;
    }
    case 'q':
    case 'Q':
    {
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[255];
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer, 255, "%Y-%m-%d_%H-%M-%S.jpg", timeinfo);
        if (key == 'q')
        {
#ifdef WIN32
            std::string filename(
                "E:/Cloud/Dropbox/Samsung Link/Photos/1K/CudaSolR_");
#else
            std::string filename("~/Pictures/1K/CudaSolR_");
#endif
            filename += buffer;

            if (kernel)
                kernel->generateScreenshot(
                    filename, 2100, 2970,
                    gScene->getSceneInfo().maxPathTracingIterations);
        }
        else
        {
#ifdef WIN32
            std::string filename(
                "E:/Cloud/Dropbox/Samsung Link/Photos/4K/CudaSolR_");
#else
            std::string filename("~/Pictures/4K/CudaSolR_");
#endif
            filename += buffer;
            if (kernel)
                kernel->generateScreenshot(
                    filename, 2 * 2970, 2 * 2100,
                    gScene->getSceneInfo().maxPathTracingIterations);
        }
        break;
    }
    case 'R':
    {
        // Recompile OpenCL Kernel
        gScene->getKernel()->recompileKernels();
        break;
    }
    case 'r':
    {
        // Reset scene
        delete gScene;
        gScene = 0;
        createScene();
        break;
    }
    case 'S':
    {
        gScene->getSceneInfo().graphicsLevel = static_cast<GraphicsLevel>(
            gScene->getSceneInfo().graphicsLevel + 1);
        if (gScene->getSceneInfo().graphicsLevel > glFull)
            gScene->getSceneInfo().graphicsLevel = glNoShading;
        break;
    }
    case 's':
    {
        // Translate camera position
        gViewPos.y -= 100.f;
        gViewDir.y -= 100.f;
        break;
    }
    case 'T':
    {
        // Double sided triangles
        gScene->getSceneInfo().doubleSidedTriangles =
            !gScene->getSceneInfo().doubleSidedTriangles;
        break;
    }
    case 't':
    {
        // Reset scene
        delete gScene;
        gScene = 0;
        gAnimate = false;
        gSceneId = ((gSceneId + 1) % 17);
        createScene();
        break;
    }
    case 'v':
    {
        gScene->createRandomMaterials(true, false);
        break;
    }
    case 'w':
    {
        // Translate camera position
        gViewPos.y += 100.f;
        gViewDir.y += 100.f;
        break;
    }
    case 'W':
    {
        gScene->getSceneInfo().backgroundColor.w += 0.1f;
        gScene->getSceneInfo().backgroundColor.w =
            (gScene->getSceneInfo().backgroundColor.w > 1.f)
                ? 0.f
                : gScene->getSceneInfo().backgroundColor.w;
        break;
    }
    case 'x':
    {
        // Reset scene
        delete gScene;
        gScene = 0;
        gCornellBoxType = (gCornellBoxType + 1) % 10;
        createScene();
        break;
    }
    case 'y':
    case 'Y':
    {
        int light = gScene->getKernel()->getLight(gLampId);
        solr::CPUPrimitive *p = gScene->getKernel()->getPrimitive(light);
        Material *m = gScene->getKernel()->getMaterial(p->materialId);
        if (m)
        {
            if (key == 'Y')
            {
                m->color.x = 1.f;
                m->color.y = 1.f;
                m->color.z = 1.f;
            }
            else
            {
                m->color.x = rand() % 256 / 256.f;
                m->color.y = rand() % 256 / 256.f;
                m->color.z = rand() % 256 / 256.f;
            }
            gScene->getKernel()->compactBoxes(false);
        }
        break;
    }
    case '*':
    {
        gScene->getSceneInfo().cameraType = static_cast<CameraType>(
            (gScene->getSceneInfo().cameraType + 1) % 7);
        LOG_INFO(1, "Camera type: " << gScene->getSceneInfo().cameraType);
        break;
    }
    case 'l':
    {
        kernel->saveToFile("test.irt");
        break;
    }
    case '1':
    {
        gControlType = ctCamera;
        gHint = "Camera";
        break;
    }
    case '2':
    {
        gControlType = ctLightSource;
        unsigned int activeLamps = gKernel->getNbActiveLamps();

        std::stringstream s;
        s << "Light source (" << gLampId + 1 << "/" << activeLamps << ")";
        LOG_INFO(3, s.str());
        gHint = s.str();
        if (activeLamps > 1)
        {
            gLampId++;
            gLampId %= activeLamps;
        }
        break;
    }
    case '3':
    {
        gControlType = ctObject;
        gHint = "Object";
        break;
    }
    case '4':
    {
        gControlType = ctFocus;
        gHint = "Focus";
        break;
    }
    case '5':
    {
        gControlType = ctFOV;
        gHint = "Field of view";
        break;
    }
    case '6':
    {
        gControlType = ctGroundHeight;
        gHint = "Ground Height";
        break;
    }
    case '7':
    {
        ++gPostProcessingInfo.param3;
        gPostProcessingInfo.param3 %= 6;
        break;
    }
    case '9':
    {
        gScene->getSceneInfo().shadowIntensity += 0.05f;
        gScene->getSceneInfo().shadowIntensity =
            (gScene->getSceneInfo().shadowIntensity > 1.f)
                ? 0.f
                : gScene->getSceneInfo().shadowIntensity;
        break;
    }
    case '+':
    {
        if (gControlType == ctLightSource)
        {
            int light = gScene->getKernel()->getLight(gLampId);
            solr::CPUPrimitive *p = gScene->getKernel()->getPrimitive(light);
            Material *m = gScene->getKernel()->getMaterial(p->materialId);
            if (m)
            {
                if (m->innerIllumination.x < 1.f)
                {
                    m->innerIllumination.x += 0.01f;
                    gScene->getKernel()->compactBoxes(false);
                }
            }
        }
        else if (gScene->getSceneInfo().cameraType == ctVR)
        {
            gDistortion += 0.01f;
            gKernel->setDistortion(gDistortion);
            LOG_INFO(1, "Distortion = " << gDistortion);
        }
        else
        {
            gScene->getSceneInfo().rayEpsilon *= 2.f;
            LOG_INFO(1, "Epsilon = " << gScene->getSceneInfo().rayEpsilon);
        }
        break;
    }
    case '-':
    {
        if (gControlType == ctLightSource)
        {
            int light = gScene->getKernel()->getLight(gLampId);
            solr::CPUPrimitive *p = gScene->getKernel()->getPrimitive(light);
            Material *m = gScene->getKernel()->getMaterial(p->materialId);
            if (m)
            {
                if (m->innerIllumination.x > 0.f)
                {
                    m->innerIllumination.x -= 0.01f;
                    gScene->getKernel()->compactBoxes(false);
                }
            }
        }
        else if (gScene->getSceneInfo().cameraType == ctVR)
        {
            gDistortion -= 0.01f;
            gKernel->setDistortion(gDistortion);
            LOG_INFO(1, "Distortion = " << gDistortion);
        }
        else
        {
            gScene->getSceneInfo().rayEpsilon /= 2.f;
            LOG_INFO(1, "Epsilon = " << gScene->getSceneInfo().rayEpsilon);
        }
        break;
    }
    case '\033':
    case '\015':
    {
        // Cleanup up and quit
        bNoPrompt = true;
        Cleanup(EXIT_SUCCESS);
        break;
    }
    }
    gScene->getSceneInfo().pathTracingIteration = 0;
}

void mouse(int button, int state, int x, int y)
{
    SceneInfo &si = gScene->getSceneInfo();
    if (state == GLUT_DOWN)
    {
        mouse_buttons |= 1 << button;
        if (gDraft)
            // si.draftMode = 1;
            si.renderBoxes = 2;
    }
    else
    {
        if (state == GLUT_UP)
        {
            mouse_buttons = 0;
            si.renderBoxes = 0;
            si.pathTracingIteration = 0;
            // si.draftMode = 0;
        }
    }
    mouse_old_x = x;
    mouse_old_y = y;
}

void motion(int x, int y)
{
    gSelectedPrimitive = gKernel->getPrimitiveAt(x, y);

    SceneInfo &si = gScene->getSceneInfo();
    switch (mouse_buttons)
    {
    case 1: // Right Button
    {
        si.pathTracingIteration = 0;

        switch (gControlType)
        {
        case ctCamera:
        {
            // Camera Rotation
            float a = (mouse_old_x - x) / 100.f;
            if (fabs(a) <= 1.f)
                gViewAngles.y = gViewAngles.y - asin(a);
            a = -(mouse_old_y - y) / 100.f;
            if (fabs(a) <= 1.f)
                gViewAngles.x = gViewAngles.x + asin(a);
            break;
        }
        case ctFocus:
        {
            // Depth of field focus
            gPostProcessingInfo.param2 += 50 * (mouse_old_y - y);
            LOG_INFO(1, "Param1=" << gPostProcessingInfo.param1);
            break;
        }
        case ctLightSource:
        {
            int light = gKernel->getLight(gLampId);
            LOG_INFO(3, "Lamp " << gLampId << "[" << light << "] selected");
            if (light != -1)
            {
                solr::CPUPrimitive *lamp = gKernel->getPrimitive(light);
                lamp->p0.x -= 20 * (mouse_old_x - x);
                lamp->p0.z += 20 * (mouse_old_y - y);
                gKernel->setPrimitiveCenter(light, lamp->p0);
                gKernel->compactBoxes(false);
            }
            else
                LOG_ERROR("No light defined in the current scene");
            break;
        }
        case ctObject:
        {
            // Rotate primitives
            vec3f rotationCenter = make_vec3f();
            float a = (mouse_old_x - x) / 100.f;
            if (fabs(a) <= 1.f)
                gRotationAngles.y = asin(a);
            a = (mouse_old_y - y) / 100.f;
            if (fabs(a) <= 1.f)
                gRotationAngles.x = -asin(a);
            gRotationAngles.z += 0.f;
            gScene->rotatePrimitives(rotationCenter, gRotationAngles);
            gKernel->compactBoxes(false);
            break;
        }
        case ct3DVision: // Eye width
        {
            gScene->getSceneInfo().eyeSeparation += (mouse_old_y - y);
            break;
        }
        default:
            break;
        }
        break;
    }
    case 4: // Left Button
    {
        si.pathTracingIteration = 0;
        switch (gControlType)
        {
        case ctCamera:
        {
            // Translate camera distance
            gViewPos.z += 20.f * (mouse_old_y - y);
            gViewDir.z += 20.f * (mouse_old_y - y);
            break;
        }
        case ctLightSource:
        {
            // Translate Light source
            int light = gKernel->getLight(gLampId);
            LOG_INFO(3, "Lamp " << gLampId << "[" << light << "] selected");
            if (light != -1)
            {
                solr::CPUPrimitive *lamp = gKernel->getPrimitive(light);
                lamp->p0.x -= 20 * (mouse_old_x - x);
                lamp->p0.y += 20 * (mouse_old_y - y);
                gKernel->setPrimitiveCenter(light, lamp->p0);
                gKernel->compactBoxes(false);
            }
            else
                LOG_ERROR("No light defined in the current scene");
            break;
        }
        case ctFocus:
        {
            // Depth of field focus
            gPostProcessingInfo.param1 += 20.f * (mouse_old_y - y);
            // gPostProcessingInfo.param3 = gPostProcessingInfo.param2/5;
            LOG_INFO(1, "Param2=" << gPostProcessingInfo.param2);
            break;
        }
        case ctFOV:
        {
            // Changing camera angle
            gViewPos.z -= 50 * (mouse_old_y - y);
            gViewAngles.w -= 50 * (mouse_old_x - x);
            LOG_INFO(1, "ViewPos        = " << gViewPos.x << "," << gViewPos.y
                                            << "," << gViewPos.z);
            LOG_INFO(1, "ViewDir        = " << gViewDir.x << "," << gViewDir.y
                                            << "," << gViewDir.z);
            LOG_INFO(1, "Eye Separation = "
                            << gScene->getSceneInfo().eyeSeparation);
            break;
        }
        case ctGroundHeight:
        {
            float h = gScene->getGroundHeight();
            h += 50 * (mouse_old_y - y);
            gScene->setGroundHeight(h);
            break;
        }
        default:
            break;
        }
        break;
    default:
        break;
    }
    break;
    }

    mouse_old_x = x;
    mouse_old_y = y;

    if (gAutoFocus)
    {
        // Autofocus
        int index = gKernel->getPrimitiveAt(gKernel->getSceneInfo().size.x / 2,
                                            gKernel->getSceneInfo().size.y / 2);
        if (index != -1)
        {
            solr::CPUPrimitive *p = gKernel->getPrimitive(index);
            gViewDir.z = (p->p0.z + p->p1.z + p->p2.z) / 3.f;
        }
        else
            gViewDir.z = gViewPos.z + 8000.f;
    }
}

void Cleanup(int iExitCode)
{
#ifdef WIN32
    gTickCount = GetTickCount() - gTickCount;
    LOG_INFO(1, "Benchmark: " << gTickCount << " tick counts");
#endif // WIN32
    delete gScene;
    LOG_INFO(1, "Looking forward to seeing you again (^_^)/");
    exit(iExitCode);
}

#ifdef _USE_KINECT
void createKinectScene(int platform, int device)
{
    std::cout << "createKinectScene" << std::endl;

    kernel = new kernel(false, gPlatform, gDevice);
    gScene->getSceneInfo().pathTracingIteration = 0;
    kernel.setSceneInfo(gScene->getSceneInfo());
    kernel.initBuffers();
    kernel.setCamera(gViewPos, gViewDir, gViewAngles);
    kernel.setPostProcessingInfo(gPostProcessingInfo);

    createRandomMaterials();

    kernel.render_begin(0.f);
    kernel.render_end((char *)gUbImage);

    char *depthBitmap = kernel.getDepthBitmap();
    if (depthBitmap != 0)
    {
        for (int x(0); x < KINECT_DEPTH_WIDTH; x += gKinectStep)
        {
            for (int y(0); y < KINECT_DEPTH_HEIGHT; y += gKinectStep)
            {
                int index = (y * KINECT_DEPTH_WIDTH + x) * KINECT_DEPTH_DEPTH;
                char a = depthBitmap[index];
                char b = depthBitmap[index + 1];
                int s = a * 256 + b;

                USHORT RealDepth = (s & 0xfff8) >> 3;
                // Masque pour retrouver si un player a �t� detect�
                USHORT Player = s & 7;
                // Transforme les informations de profondeur sur 13-Bit en une
                // intensit� cod� sur 8 Bits

                // afin d'afficher une couleur en fonction de la profondeur
                BYTE l = 255 - (BYTE)(256 * RealDepth / 0x0fff);

                int p = kernel.addPrimitive(ptSphere);
                kernel.setPrimitive(p, 5 + p / gKinectNbSpherePerBox,
                                    (x - 160) * 100.f, (y - 120) * 100.f, 0.f,
                                    gKinectStep * 10.f, 0.f, 0.f, 20 + x % 20,
                                    1, 1);
            }
        }
    }

    // Sol
    int s0 = kernel.addPrimitive(ptXZPlane);
    kernel.setPrimitive(s0, 0, 0.f, -5000.f, 0.f, 10000.f, 0.f, 10000.f, 100, 1,
                        1);

    // Wall
    int m_nbPrimitives = kernel.addPrimitive(ptXYPlane);
    kernel.setPrimitive(m_nbPrimitives, 0, 0.f, 0.f, 5000.f, 5000.f, 5000.f,
                        0.f, 110, 1, 1);
    m_nbPrimitives = kernel.addPrimitive(ptYZPlane);
    kernel.setPrimitive(m_nbPrimitives, 1, -5000.f, 0.f, 0.f, 0.f, 5000.f,
                        5000.f, 107, 1, 1);
    m_nbPrimitives = kernel.addPrimitive(ptYZPlane);
    kernel.setPrimitive(m_nbPrimitives, 2, 5000.f, 0.f, 0.f, 0.f, 5000.f,
                        5000.f, 108, 1, 1);
    m_nbPrimitives = kernel.addPrimitive(ptXZPlane);
    kernel.setPrimitive(m_nbPrimitives, 3, 0.f, 5000.f, 0.f, 5000.f, 0.f,
                        5000.f, 110, 1, 1);

    // Camera
    m_nbPrimitives = kernel.addPrimitive(ptCamera);
    kernel.setPrimitive(m_nbPrimitives, 0, -2500.f, -4000.f, -5000.f, 1500.f,
                        1500.f, 0.f, DEFAULT_LIGHT_MATERIAL, 1, 1);

    // Lights
    gLampIndex = kernel.addPrimitive(ptSphere);
    kernel.setPrimitive(gLampIndex, 4, -5000.f, 2000.f, -20000.f, 500, 0, 50,
                        128, 1, 1);

    gNbBoxes = kernel.compactBoxes();
}
#endif // USE_KINECT

void createScene()
{
    gAnimate = false;
    gHint = "Camera";
    switch (gSceneId)
    {
    case 1:
        gScene = new TrianglesScene("IRT models");
        break;
    case 2:
        gScene = new ObjScene("OBJ models", gFilename);
        break;
    case 3:
        gScene = new MoleculeScene("Molecules");
        break;
    case 4:
        gScene = new HypercubeScene("Hypercubes");
        break;
    case 5:
        gScene = new CylinderScene("Cylinders");
        break;
    case 6:
        gScene = new XmasScene("Christmas");
        break;
    case 7:
        gScene = new Year2013("Happy new year 2013");
        break;
    case 8:
        gScene = new DoggyStyleScene("Doggy style");
        break;
    case 9:
        gScene = new MetaballsScene("Metaballs");
        gAnimate = true;
        break;
    case 10:
        gScene = new GraphScene("Charts");
        break;
    case 11:
        gScene = new CornellBoxScene("Cornell Box");
        break;
    case 12:
        gScene = new WaterScene("Water");
        break;
    case 13:
        gScene = new CubesScene("Cubes");
        break;
    case 14:
        gScene = new TransparentScene("Transparent");
        break;
    case 15:
        gScene = new SwcScene("Neuroscience");
        break;
    case 16:
        gScene = new FractalScene("Fractals");
        break;
    default:
#ifdef _USE_KINECT
        gScene = new KinectFaceTrackingScene("Kinect Face Tracking");
        break;
#else
        gScene = new TesseractScene("Tesseract");
        //        gScene = new PerpetualMotionScene("Perpetual Motion");
        break;
#endif
    }

    gScene->setCornellBoxType(gCornellBoxType);
    gScene->setCurrentModel(m_counter);
    gScene->initialize(gKernel, gWindowWidth, gWindowHeight);
    gKernel->setCamera(gViewPos, gViewDir, gViewAngles);
}

int main(int argc, char *argv[])
{
#if USE_RANDOM_DEVICE
    LOG_INFO(1, "Using random device generator");
#endif

    std::vector<std::string> arguments;
    for (int i(0); i < argc; ++i)
    {
        arguments.push_back(argv[i]);
    }
    readParameters(arguments);

    srand(static_cast<int>(time(0)));

    initgl(argc, argv);

    initMenus();

    createScene();

    LOG_INFO(1, "Sol-R is happily running... (^_^)y");
    for (const auto &menuItem : menuItems)
        LOG_INFO(1, menuItem.description);

    glutMainLoop();

    Cleanup(EXIT_SUCCESS);

    return 0;
}
