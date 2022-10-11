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

#ifdef WIN32
#include <windows.h>
#else
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#endif

#include <algorithm>
#include <random>

// JPeg
#include <images/ImageLoader.h>
#include <images/jpge.h>

// Raytracing
#include "GPUKernel.h"
#include "Logging.h"
#include "consts.h"
#include "io/FileMarshaller.h"
#include "opengl/rtgl.h"

#ifdef USE_CUDA
#include <engines/cuda/CudaKernel.h>
#else
#ifdef USE_OPENCL
#include <engines/opencl/OpenCLKernel.h>
#else
#ifdef USE_CPU
#include "cpu/CPUKernel.h"
#endif // USE_CPU
#endif // USE_OPENCL
#endif // USE_CUDA

// Oculus
#ifdef USE_OCULUS
#if _MSC_VER >= 1900
// This is needed to compile with VS 2015
FILE _iob[] = {*stdin, *stdout, *stderr};

extern "C" FILE *__cdecl __iob_func(void)
{
    return _iob;
}
#pragma comment(lib, "legacy_stdio_definitions.lib")
#endif
#endif // USE_OCULUS

using namespace solr;

const unsigned int AABB_MAGIC_NUMBER = 6400;

vec3f min2(const vec3f a, const vec3f b)
{
    vec3f r;
    r.x = std::min(a.x, b.x);
    r.y = std::min(a.y, b.y);
    r.z = std::min(a.z, b.z);
    return r;
}

vec3f max2(const vec3f a, const vec3f b)
{
    vec3f r;
    r.x = std::max(a.x, b.x);
    r.y = std::max(a.y, b.y);
    r.z = std::max(a.z, b.z);
    return r;
}

vec3f min3(const vec3f a, const vec3f b, const vec3f c)
{
    vec3f r;
    r.x = std::min(std::min(a.x, b.x), c.x);
    r.y = std::min(std::min(a.y, b.y), c.y);
    r.z = std::min(std::min(a.z, b.z), c.z);
    return r;
}

vec3f max3(const vec3f a, const vec3f b, const vec3f c)
{
    vec3f r;
    r.x = std::max(std::max(a.x, b.x), c.x);
    r.y = std::max(std::max(a.y, b.y), c.y);
    r.z = std::max(std::max(a.z, b.z), c.z);
    return r;
}

namespace solr
{
GPUKernel *SingletonKernel::m_kernel = 0;

SingletonKernel::SingletonKernel() {}

GPUKernel *SingletonKernel::kernel()
{
    if (!m_kernel)
#ifdef USE_CUDA
        m_kernel = new CudaKernel();
#else
#ifdef USE_OPENCL
        m_kernel = new OpenCLKernel();
#else
#ifdef USE_OPENCL
        m_kernel = new CPUKernel();
#endif // USE_CPU
#endif // USE_OPENCL
#endif // USE_CUDA
    return m_kernel;
}

float GPUKernel::dotProduct(const vec3f &a, const vec3f &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// ________________________________________________________________________________
float GPUKernel::vectorLength(const vec3f &vector)
{
    return sqrt(vector.x * vector.x + vector.y * vector.y +
                vector.z * vector.z);
}

// ________________________________________________________________________________
void GPUKernel::normalizeVector(vec3f &v)
{
    float l = vectorLength(v);
    if (l != 0.f)
    {
        v.x /= l;
        v.y /= l;
        v.z /= l;
    }
}

vec3f GPUKernel::crossProduct(const vec3f &b, const vec3f &c)
{
    vec3f a;
    a.x = b.y * c.z - b.z * c.y;
    a.y = b.z * c.x - b.x * c.z;
    a.z = b.x * c.y - b.y * c.x;
    return a;
}

#ifndef WIN32
typedef struct
{
    short bfType;
    int bfSize;
    short Reserved1;
    short Reserved2;
    int bfOffBits;
} BITMAPFILEHEADER;

typedef struct
{
    int biSizeImage;
    int biWidth;
    int biHeight;
} BITMAPINFOHEADER;
#endif

GPUKernel::GPUKernel()
    : m_oculus(false)
    , m_hBoundingBoxes(0)
    , m_hPrimitives(0)
    , m_hLamps(0)
    , m_hMaterials(0)
    , m_hRandoms(0)
    , m_hPrimitivesXYIds(0)
    , m_nbActiveMaterials(-1)
    , m_nbActiveTextures(0)
    , m_lightInformationSize(0)
    , m_maxPrimitivesPerBox(0)
    , m_doneWithAdding(false)
    , m_addingIndex(0)
    , m_distortion(0.1f)
    , m_frame(0)
    , m_nbFrames(0)
    , m_morph(0.f)
    , m_treeDepth(2)
    , m_bitmap(0)
    , m_primitivesTransfered(false)
    , m_materialsTransfered(false)
    , m_texturesTransfered(false)
    , m_randomsTransfered(false)
    , m_refresh(true)
    , m_activeLogging(false)
    , m_lightInformation(0)
    , m_optimalNbOfBoxes(NB_MAX_BOXES)
    , m_GLMode(-1)
    , m_currentMaterial(0)
    , m_pointSize(1.f)
#if USE_KINECT
    , m_hVideo(0)
    , m_hDepth(0)
    , m_skeletons(0)
    , m_hNextDepthFrameEvent(0)
    , m_hNextVideoFrameEvent(0)
    , m_hNextSkeletonEvent(0)
    , m_pVideoStreamHandle(0)
    , m_pDepthStreamHandle(0)
    , m_skeletonsBody(-1)
    , m_skeletonsLamp(-1)
    , m_skeletonIndex(-1)
#endif // USE_KINECT
#ifdef USE_OCULUS
    , m_sensorFusion(0)
#endif // USE_OCULUS
{
    LOG_INFO(1, "");
    LOG_INFO(1,
             "                     _|_|_|            _|              _|_|_|  ");
    LOG_INFO(1,
             "                   _|          _|_|    _|              _|    _|");
    LOG_INFO(1,
             "                     _|_|    _|    _|  _|  _|_|_|_|_|  _|_|_|  ");
    LOG_INFO(1,
             "                         _|  _|    _|  _|              _|    _|");
    LOG_INFO(1,
             "                   _|_|_|      _|_|    _|              _|    _|");
    LOG_INFO(1, "");
    LOG_INFO(1, "                            Speed Of Light Ray-tracer");
    LOG_INFO(1, "                       Created by cyrille.favreau@gmail.com");
    LOG_INFO(1, "");

    for (int i(0); i < NB_MAX_FRAMES; ++i)
    {
        m_nbActiveBoxes[i] = 0;
        m_nbActivePrimitives[i] = 0;
        m_nbActiveLamps[i] = 0;
    }

    LOG_INFO(3,
             "----------++++++++++  GPU Kernel created  ++++++++++----------");
    LOG_INFO(3, "CPU: SceneInfo         : " << sizeof(SceneInfo));
    LOG_INFO(3, "CPU: Ray               : " << sizeof(Ray));
    LOG_INFO(3, "CPU: PrimitiveType     : " << sizeof(PrimitiveType));
    LOG_INFO(3, "CPU: Material          : " << sizeof(Material));
    LOG_INFO(3, "CPU: BoundingBox       : " << sizeof(BoundingBox));
    LOG_INFO(3, "CPU: Primitive         : " << sizeof(Primitive));
    LOG_INFO(3, "CPU: PostProcessingType: " << sizeof(PostProcessingType));
    LOG_INFO(3, "CPU: PostProcessingInfo: " << sizeof(PostProcessingInfo));
    LOG_INFO(3, "Textures " << NB_MAX_TEXTURES);

    m_viewPos.x = 0.f;
    m_viewPos.y = 0.f;
    m_viewPos.z = 0.f;
    m_viewDir.x = 0.f;
    m_viewDir.y = 0.f;
    m_viewDir.z = 0.f;

#if USE_KINECT
    // Initialize Kinect
    LOG_INFO(1, "----------------------------");
    LOG_INFO(1, "                         O  ");
    LOG_INFO(1, "                       --+--");
    LOG_INFO(1, "                         |  ");
    LOG_INFO(1, "Kinect initialization   / \\");

    HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |
                               NUI_INITIALIZE_FLAG_USES_SKELETON |
                               NUI_INITIALIZE_FLAG_USES_COLOR);
    m_kinectEnabled = (hr == S_OK);

    if (m_kinectEnabled)
    {
        m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        m_hNextVideoFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        m_hNextSkeletonEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

        m_skeletons = CreateEvent(NULL, TRUE, FALSE, NULL);
        NuiSkeletonTrackingEnable(
            m_skeletons, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);

        NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
                           0, 2, m_hNextVideoFrameEvent, &m_pVideoStreamHandle);
        NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
                           NUI_IMAGE_RESOLUTION_320x240, 0, 2,
                           m_hNextDepthFrameEvent, &m_pDepthStreamHandle);

        NuiCameraElevationSetAngle(0);
    }
    else
    {
        LOG_ERROR("    FAILED");
    }
    LOG_INFO(3, "----------------------------");
#endif // USE_KINECT
}

GPUKernel::~GPUKernel()
{
    LOG_INFO(3, "GPUKernel::~GPUKernel");

#if USE_KINECT
    if (m_kinectEnabled)
    {
        CloseHandle(m_skeletons);
        CloseHandle(m_hNextDepthFrameEvent);
        CloseHandle(m_hNextVideoFrameEvent);
        CloseHandle(m_hNextSkeletonEvent);
        NuiShutdown();
    }
#endif // USE_KINECT

    cleanup();
    LOG_INFO(3,
             "----------++++++++++ GPU Kernel Destroyed ++++++++++----------");
}

void GPUKernel::initBuffers()
{
    LOG_INFO(3, "GPUKernel::initBuffers");

    // Setup CPU resources
    m_lightInformation = new LightInformation[NB_MAX_LIGHTINFORMATIONS];

    m_hMaterials = new Material[NB_MAX_MATERIALS + 1];
    memset(m_hMaterials, 0, NB_MAX_MATERIALS * sizeof(Material));

#ifndef USE_MANAGED_MEMORY
    m_hBoundingBoxes = new BoundingBox[NB_MAX_BOXES];
    memset(m_hBoundingBoxes, 0, NB_MAX_BOXES * sizeof(BoundingBox));

    m_hPrimitives = new Primitive[NB_MAX_PRIMITIVES];
    memset(m_hPrimitives, 0, NB_MAX_PRIMITIVES * sizeof(Primitive));
#endif

    m_hLamps = new Lamp[NB_MAX_LAMPS];
    memset(m_hLamps, 0, NB_MAX_LAMPS * sizeof(Lamp));

    // Textures
    memset(m_hTextures, 0, NB_MAX_TEXTURES * sizeof(TextureInfo));

    reshape();

#ifdef USE_OCULUS
    LOG_INFO(1, "Initializing Oculus DK1");
    LOG_INFO(1, "-----------------------");
    LOG_INFO(1, "        [ O O ]");
    LOG_INFO(1, "-----------------------");
    initializeOVR();
#endif // USE_OCULUS
}

void GPUKernel::cleanup()
{
#ifdef USE_OCULUS
// finializeOVR();
#endif // USE_OCULUS

    LOG_INFO(3, "Cleaning up resources");

    for (int i(0); i < NB_MAX_FRAMES; ++i)
    {
        for (int j(0); j < BOUNDING_BOXES_TREE_DEPTH; ++j)
        {
            m_boundingBoxes[i][j].clear();
        }
        m_nbActiveBoxes[i] = 0;

        m_primitives[i].clear();
        m_nbActivePrimitives[i] = 0;

        m_lamps[i].clear();
        m_nbActiveLamps[i] = 0;

        m_minPos[i].x = -m_sceneInfo.viewDistance;
        m_minPos[i].y = -m_sceneInfo.viewDistance;
        m_minPos[i].z = -m_sceneInfo.viewDistance;
        m_maxPos[i].x = m_sceneInfo.viewDistance;
        m_maxPos[i].y = m_sceneInfo.viewDistance;
        m_maxPos[i].z = m_sceneInfo.viewDistance;
    }

    for (int i(0); i < NB_MAX_TEXTURES; ++i)
    {
#ifdef USE_KINECT
        if (i > 1 && m_hTextures[i].buffer)
            delete[] m_hTextures[i].buffer;
#else
        if (m_hTextures[i].buffer != 0)
        {
            LOG_INFO(3, "[cleanup] Buffer " << i << " needs to be released");
            delete[] m_hTextures[i].buffer;
            m_hTextures[i].buffer = 0;
        }
#endif // USE_KINECT
    }
    memset(m_hTextures, 0, NB_MAX_TEXTURES * sizeof(TextureInfo));

    m_vertices.clear();
    m_normals.clear();
    m_textCoords.clear();

    if (m_hRandoms)
        delete m_hRandoms;
    m_hRandoms = 0;
    if (m_bitmap)
        delete[] m_bitmap;
    m_bitmap = 0;
#ifndef USE_MANAGED_MEMORY
    if (m_hBoundingBoxes)
        delete m_hBoundingBoxes;
    m_hBoundingBoxes = 0;
    if (m_hPrimitives)
        delete m_hPrimitives;
    m_hPrimitives = nullptr;
#endif
    if (m_hLamps)
        delete m_hLamps;
    m_hLamps = 0;
    if (m_hMaterials)
        delete m_hMaterials;
    m_hMaterials = 0;
    if (m_hPrimitivesXYIds)
        delete m_hPrimitivesXYIds;
    m_hPrimitivesXYIds = 0;
    if (m_lightInformation)
        delete m_lightInformation;
    m_lightInformation = 0;

    m_nbActiveMaterials = -1;
    m_nbActiveTextures = 0;
    m_materialsTransfered = false;
    m_primitivesTransfered = false;
    m_texturesTransfered = false;
    m_randomsTransfered = false;

    // Morphing
    m_morph = 0.f;
#if USE_KINECT
    m_hVideo = 0;
    m_hDepth = 0;
    m_skeletons = 0;
    m_hNextDepthFrameEvent = 0;
    m_hNextVideoFrameEvent = 0;
    m_hNextSkeletonEvent = 0;
    m_pVideoStreamHandle = 0;
    m_pDepthStreamHandle = 0;
    m_skeletonsBody = -1;
    m_skeletonsLamp = -1;
    m_skeletonIndex = -1;
#endif // USE_KINECT
}

void GPUKernel::reshape()
{
    size_t size = m_sceneInfo.size.x * m_sceneInfo.size.y;

    // Randoms
    if (m_hRandoms)
        delete m_hRandoms;
    m_hRandoms = new RandomBuffer[size];

    // Primitive IDs
    if (m_hPrimitivesXYIds)
        delete m_hPrimitivesXYIds;
    m_hPrimitivesXYIds = new PrimitiveXYIdBuffer[size];
    memset(m_hPrimitivesXYIds, 0, size * sizeof(PrimitiveXYIdBuffer));

    // Bitmap
    if (m_bitmap)
        delete m_bitmap;
    size *= gColorDepth;
    m_bitmap = new BitmapBuffer[size];
    memset(m_bitmap, 0, size * sizeof(BitmapBuffer));
    LOG_INFO(3, m_bitmap << " - Bitmap Size=" << size);
}

/*
________________________________________________________________________________

Sets camera
________________________________________________________________________________
*/
void GPUKernel::setCamera(const vec3f &eye, const vec3f &dir,
                          const vec4f &angles)
{
    LOG_INFO(3, "GPUKernel::setCamera("
                    << eye.x << "," << eye.y << "," << eye.z << " -> " << dir.x
                    << "," << dir.y << "," << dir.z << " : " << angles.x << ","
                    << angles.y << "," << angles.z << ")");
    m_viewPos = eye;
    m_viewDir = dir;
    m_angles = angles;
    m_refresh = true;
}

int GPUKernel::addPrimitive(PrimitiveType type, bool belongsToModel)
{
    LOG_INFO(3, "GPUKernel::addPrimitive");
    int returnValue = -1;
    if (m_doneWithAdding)
    {
        returnValue = m_addingIndex;
        m_addingIndex++;
    }
    else
    {
        CPUPrimitive primitive;
        memset(&primitive, 0, sizeof(CPUPrimitive));
        primitive.belongsToModel = belongsToModel;
        primitive.type = type;
        int index = static_cast<int>(m_primitives[m_frame].size());
        m_primitives[m_frame][index] = primitive;
        LOG_INFO(3, "m_primitives.size() = " << m_primitives[m_frame].size());
        returnValue = index;
    }
    return returnValue;
}

CPUPrimitive *GPUKernel::getPrimitive(const unsigned int index)
{
    CPUPrimitive *returnValue(NULL);
    if (index <= m_primitives[m_frame].size())
    {
        returnValue = &(m_primitives[m_frame])[index];
    }
    return returnValue;
}

void GPUKernel::setPrimitive(const int &index, float x0, float y0, float z0,
                             float w, float h, float d, int materialId)
{
    setPrimitive(index, x0, y0, z0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, w, h, d,
                 materialId);
}

void GPUKernel::setPrimitive(const int &index, float x0, float y0, float z0,
                             float x1, float y1, float z1, float w, float h,
                             float d, int materialId)
{
    setPrimitive(index, x0, y0, z0, x1, y1, z1, 0.f, 0.f, 0.f, w, h, d,
                 materialId);
}

void GPUKernel::setPrimitive(const int &index, float x0, float y0, float z0,
                             float x1, float y1, float z1, float x2, float y2,
                             float z2, float w, float h, float d,
                             int materialId)
{
    float scale = 1.f;
    m_primitivesTransfered = false;
    if (index >= 0 && index <= m_primitives[m_frame].size())
    {
        (m_primitives[m_frame])[index].movable = true;
        (m_primitives[m_frame])[index].p0.x = x0 * scale;
        (m_primitives[m_frame])[index].p0.y = y0 * scale;
        (m_primitives[m_frame])[index].p0.z = z0 * scale;
        (m_primitives[m_frame])[index].p1.x = x1 * scale;
        (m_primitives[m_frame])[index].p1.y = y1 * scale;
        (m_primitives[m_frame])[index].p1.z = z1 * scale;
        (m_primitives[m_frame])[index].p2.x = x2 * scale;
        (m_primitives[m_frame])[index].p2.y = y2 * scale;
        (m_primitives[m_frame])[index].p2.z = z2 * scale;
        (m_primitives[m_frame])[index].size.x = w * scale;
        (m_primitives[m_frame])[index].size.y = h * scale;
        (m_primitives[m_frame])[index].size.z = d * scale;
        (m_primitives[m_frame])[index].n0 = make_vec3f();
        (m_primitives[m_frame])[index].n1 = make_vec3f();
        (m_primitives[m_frame])[index].n2 = make_vec3f();
        (m_primitives[m_frame])[index].vt0 = make_vec2f();
        (m_primitives[m_frame])[index].vt1 = make_vec2f();
        (m_primitives[m_frame])[index].vt2 = make_vec2f();
        (m_primitives[m_frame])[index].materialId = materialId;

        switch ((m_primitives[m_frame])[index].type)
        {
        case ptSphere:
        {
            (m_primitives[m_frame])[index].size.x = w * scale;
            (m_primitives[m_frame])[index].size.y = w * scale;
            (m_primitives[m_frame])[index].size.z = w * scale;
            break;
        }
        case ptEllipsoid:
        {
            (m_primitives[m_frame])[index].size.x = w * scale;
            (m_primitives[m_frame])[index].size.y = h * scale;
            (m_primitives[m_frame])[index].size.z = d * scale;
            break;
        }
        case ptCylinder:
        case ptCone:
        {
            // Axis
            vec4f axis;
            axis.x = x1 * scale - x0 * scale;
            axis.y = y1 * scale - y0 * scale;
            axis.z = z1 * scale - z0 * scale;
            float len =
                sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (len != 0.f)
            {
                axis.x /= len;
                axis.y /= len;
                axis.z /= len;
            }
            (m_primitives[m_frame])[index].n1.x = axis.x;
            (m_primitives[m_frame])[index].n1.y = axis.y;
            (m_primitives[m_frame])[index].n1.z = axis.z;

            // Center
            (m_primitives[m_frame])[index].p2.x =
                (x0 * scale + x1 * scale) / 2.f;
            (m_primitives[m_frame])[index].p2.y =
                (y0 * scale + y1 * scale) / 2.f;
            (m_primitives[m_frame])[index].p2.z =
                (z0 * scale + z1 * scale) / 2.f;

            // Length
            (m_primitives[m_frame])[index].size.x = w * scale;
            (m_primitives[m_frame])[index].size.y = w * scale;
            (m_primitives[m_frame])[index].size.z = w * scale;
            break;
        }
#ifdef USE_KINECT
        case ptCamera:
        {
            (m_primitives[m_frame])[index].n0.x = 0.f;
            (m_primitives[m_frame])[index].n0.y = 0.f;
            (m_primitives[m_frame])[index].n0.z = -1.f;
            break;
        }
#endif // USE_KINECT
        case ptXYPlane:
        {
            (m_primitives[m_frame])[index].n0.x = 0.f;
            (m_primitives[m_frame])[index].n0.y = 0.f;
            (m_primitives[m_frame])[index].n0.z = 1.f;
            (m_primitives[m_frame])[index].n1 =
                (m_primitives[m_frame])[index].n0;
            (m_primitives[m_frame])[index].n2 =
                (m_primitives[m_frame])[index].n0;
            break;
        }
        case ptYZPlane:
        {
            (m_primitives[m_frame])[index].n0.x = 1.f;
            (m_primitives[m_frame])[index].n0.y = 0.f;
            (m_primitives[m_frame])[index].n0.z = 0.f;
            (m_primitives[m_frame])[index].n1 =
                (m_primitives[m_frame])[index].n0;
            (m_primitives[m_frame])[index].n2 =
                (m_primitives[m_frame])[index].n0;
            break;
        }
        case ptXZPlane:
        case ptCheckboard:
        {
            (m_primitives[m_frame])[index].n0.x = 0.f;
            (m_primitives[m_frame])[index].n0.y = 1.f;
            (m_primitives[m_frame])[index].n0.z = 0.f;
            (m_primitives[m_frame])[index].n1 =
                (m_primitives[m_frame])[index].n0;
            (m_primitives[m_frame])[index].n2 =
                (m_primitives[m_frame])[index].n0;
            break;
        }
        case ptTriangle:
        {
            vec3f v0, v1;
            v0.x = (m_primitives[m_frame])[index].p1.x -
                   (m_primitives[m_frame])[index].p0.x;
            v0.y = (m_primitives[m_frame])[index].p1.y -
                   (m_primitives[m_frame])[index].p0.y;
            v0.z = (m_primitives[m_frame])[index].p1.z -
                   (m_primitives[m_frame])[index].p0.z;
            normalizeVector(v0);

            v1.x = (m_primitives[m_frame])[index].p2.x -
                   (m_primitives[m_frame])[index].p0.x;
            v1.y = (m_primitives[m_frame])[index].p2.y -
                   (m_primitives[m_frame])[index].p0.y;
            v1.z = (m_primitives[m_frame])[index].p2.z -
                   (m_primitives[m_frame])[index].p0.z;
            normalizeVector(v1);

            (m_primitives[m_frame])[index].n0 = crossProduct(v0, v1);
            normalizeVector((m_primitives[m_frame])[index].n0);
            (m_primitives[m_frame])[index].n1 =
                (m_primitives[m_frame])[index].n0;
            (m_primitives[m_frame])[index].n2 =
                (m_primitives[m_frame])[index].n0;
            break;
        }
        }
        // min
        m_minPos[m_frame].x = std::min(x0 * scale, m_minPos[m_frame].x);
        m_minPos[m_frame].y = std::min(y0 * scale, m_minPos[m_frame].y);
        m_minPos[m_frame].z = std::min(z0 * scale, m_minPos[m_frame].z);

        // max
        m_maxPos[m_frame].x = std::max(x0 * scale, m_maxPos[m_frame].x);
        m_maxPos[m_frame].y = std::max(y0 * scale, m_maxPos[m_frame].y);
        m_maxPos[m_frame].z = std::max(z0 * scale, m_maxPos[m_frame].z);
    }
    else
    {
        LOG_ERROR("GPUKernel::setPrimitive: Out of bounds ("
                  << index << "/" << NB_MAX_PRIMITIVES << ")");
    }
}

void GPUKernel::setPrimitiveIsMovable(const int &index, bool movable)
{
    if (index >= 0 && index < m_primitives[m_frame].size())
    {
        CPUPrimitive &primitive((m_primitives[m_frame])[index]);
        primitive.movable = movable;
    }
}

void GPUKernel::setPrimitiveBellongsToModel(const int &index,
                                            bool bellongsToModel)
{
    if (index >= 0 && index < m_primitives[m_frame].size())
    {
        CPUPrimitive &primitive((m_primitives[m_frame])[index]);
        primitive.belongsToModel = bellongsToModel;
    }
}

void GPUKernel::setPrimitiveTextureCoordinates(const unsigned int index,
                                               const vec2f &vt0,
                                               const vec2f &vt1,
                                               const vec2f &vt2)
{
    if (index < m_primitives[m_frame].size())
    {
        CPUPrimitive &primitive((m_primitives[m_frame])[index]);
        primitive.vt0 = vt0;
        primitive.vt1 = vt1;
        primitive.vt2 = vt2;
    }
}

void GPUKernel::setPrimitiveNormals(int unsigned index, vec3f n0, vec3f n1,
                                    vec3f n2)
{
    if (index < m_primitives[m_frame].size())
    {
        CPUPrimitive &primitive((m_primitives[m_frame])[index]);
        normalizeVector(n0);
        primitive.n0 = n0;
        normalizeVector(n1);
        primitive.n1 = n1;
        normalizeVector(n2);
        primitive.n2 = n2;
    }
}

unsigned int GPUKernel::getPrimitiveAt(int x, int y)
{
    LOG_INFO(3, "GPUKernel::getPrimitiveAt(" << x << "," << y << ")");
    unsigned int returnValue = -1;
    unsigned int index = y * m_sceneInfo.size.x + x;
    if (index <
        static_cast<unsigned int>(m_sceneInfo.size.x * m_sceneInfo.size.y))
    {
        returnValue = m_hPrimitivesXYIds[index].x;
    }
    return returnValue;
}

bool GPUKernel::updateBoundingBox(CPUBoundingBox &box)
{
    LOG_INFO(3, "GPUKernel::updateBoundingBox()");

    bool result(false);

    // Process box size
    vec3f corner0;
    vec3f corner1;

    box.parameters[0].x = 1000000;
    box.parameters[0].y = 1000000;
    box.parameters[0].z = 1000000;
    box.parameters[1].x = -1000000;
    box.parameters[1].y = -1000000;
    box.parameters[1].z = -1000000;

    for (const auto &p : box.primitives)
    {
        CPUPrimitive &primitive = (m_primitives[m_frame])[p];
        result =
            (m_hMaterials[primitive.materialId].innerIllumination.x != 0.f);
        switch (primitive.type)
        {
        case ptTriangle:
        {
            corner0 = min3(primitive.p0, primitive.p1, primitive.p2);
            corner1 = max3(primitive.p0, primitive.p1, primitive.p2);
            break;
        }
        case ptCylinder:
        {
            corner0 = min2(primitive.p0, primitive.p1);
            corner1 = max2(primitive.p0, primitive.p1);
            break;
        }
        default:
        {
            corner0 = primitive.p0;
            corner1 = primitive.p0;
            break;
        }
        }

        vec4f p0, p1;
        p0.x = (corner0.x <= corner1.x) ? corner0.x : corner1.x;
        p0.y = (corner0.y <= corner1.y) ? corner0.y : corner1.y;
        p0.z = (corner0.z <= corner1.z) ? corner0.z : corner1.z;
        p1.x = (corner0.x > corner1.x) ? corner0.x : corner1.x;
        p1.y = (corner0.y > corner1.y) ? corner0.y : corner1.y;
        p1.z = (corner0.z > corner1.z) ? corner0.z : corner1.z;

        switch (primitive.type)
        {
        case ptCylinder:
        case ptSphere:
        case ptCone:
        {
            p0.x -= primitive.size.x;
            p0.y -= primitive.size.x;
            p0.z -= primitive.size.x;

            p1.x += primitive.size.x;
            p1.y += primitive.size.x;
            p1.z += primitive.size.x;
            break;
        }
        default:
        {
            p0.x -= primitive.size.x;
            p0.y -= primitive.size.y;
            p0.z -= primitive.size.z;
            p1.x += primitive.size.x;
            p1.y += primitive.size.y;
            p1.z += primitive.size.z;
            break;
        }
        }

        if (p0.x < box.parameters[0].x)
            box.parameters[0].x = p0.x;
        if (p0.y < box.parameters[0].y)
            box.parameters[0].y = p0.y;
        if (p0.z < box.parameters[0].z)
            box.parameters[0].z = p0.z;
        if (p1.x > box.parameters[1].x)
            box.parameters[1].x = p1.x;
        if (p1.y > box.parameters[1].y)
            box.parameters[1].y = p1.y;
        if (p1.z > box.parameters[1].z)
            box.parameters[1].z = p1.z;
    }

    box.center.x = (box.parameters[0].x + box.parameters[1].x) / 2.f;
    box.center.y = (box.parameters[0].y + box.parameters[1].y) / 2.f;
    box.center.z = (box.parameters[0].z + box.parameters[1].z) / 2.f;
    // LOG_INFO(1,"Box
    // Center="<<box.center.x<<","<<box.center.y<<","<<box.center.z);
    return result;
}

bool GPUKernel::updateOutterBoundingBox(CPUBoundingBox &outterBox,
                                        const int depth)
{
    LOG_INFO(3, "GPUKernel::updateOutterBoundingBox()");

    bool result(false);

    // Reset box size
    outterBox.parameters[0].x = m_sceneInfo.viewDistance;
    outterBox.parameters[0].y = m_sceneInfo.viewDistance;
    outterBox.parameters[0].z = m_sceneInfo.viewDistance;
    outterBox.parameters[1].x = -m_sceneInfo.viewDistance;
    outterBox.parameters[1].y = -m_sceneInfo.viewDistance;
    outterBox.parameters[1].z = -m_sceneInfo.viewDistance;

    LOG_INFO(3, "OutterBox: ("
                    << outterBox.center.x << "," << outterBox.center.y << ","
                    << outterBox.center.z << "),(" << outterBox.indexForNextBox
                    << "," << outterBox.primitives.size() << "),("
                    << outterBox.parameters[0].x << ","
                    << outterBox.parameters[0].y << ","
                    << outterBox.parameters[0].z << "),("
                    << outterBox.parameters[1].x << ","
                    << outterBox.parameters[1].y << ","
                    << outterBox.parameters[1].z << ")");

    // Process box size
    for (const auto &p : outterBox.primitives)
    {
        try
        {
            CPUBoundingBox &box = (m_boundingBoxes[m_frame][depth])[p];
            if (outterBox.parameters[0].x > box.parameters[0].x)
                outterBox.parameters[0].x = box.parameters[0].x;
            if (outterBox.parameters[0].y > box.parameters[0].y)
                outterBox.parameters[0].y = box.parameters[0].y;
            if (outterBox.parameters[0].z > box.parameters[0].z)
                outterBox.parameters[0].z = box.parameters[0].z;
            if (outterBox.parameters[1].x < box.parameters[1].x)
                outterBox.parameters[1].x = box.parameters[1].x;
            if (outterBox.parameters[1].y < box.parameters[1].y)
                outterBox.parameters[1].y = box.parameters[1].y;
            if (outterBox.parameters[1].z < box.parameters[1].z)
                outterBox.parameters[1].z = box.parameters[1].z;
        }
        catch (...)
        {
            LOG_ERROR("Ca chie grave dans les boites!");
        }
    }
    outterBox.center.x =
        (outterBox.parameters[0].x + outterBox.parameters[1].x) / 2.f;
    outterBox.center.y =
        (outterBox.parameters[0].y + outterBox.parameters[1].y) / 2.f;
    outterBox.center.z =
        (outterBox.parameters[0].z + outterBox.parameters[1].z) / 2.f;

    return result;
}

void GPUKernel::resetBoxes(bool resetPrimitives)
{
    if (resetPrimitives)
        for (int i(0); i < m_boundingBoxes[m_frame][0].size(); ++i)
            resetBox(m_boundingBoxes[m_frame][0][i], resetPrimitives);
    else
        m_boundingBoxes[m_frame][0].clear();
}

void GPUKernel::resetBox(CPUBoundingBox &box, bool resetPrimitives)
{
    LOG_INFO(3, "GPUKernel::resetBox(" << resetPrimitives << ")");
    if (resetPrimitives)
    {
        box.primitives.clear();
        box.indexForNextBox = 1;
    }
    box.parameters[0].x = m_sceneInfo.viewDistance;
    box.parameters[0].y = m_sceneInfo.viewDistance;
    box.parameters[0].z = m_sceneInfo.viewDistance;
    box.parameters[1].x = -m_sceneInfo.viewDistance;
    box.parameters[1].y = -m_sceneInfo.viewDistance;
    box.parameters[1].z = -m_sceneInfo.viewDistance;
}

int GPUKernel::processBoxes(const int boxSize, bool simulate)
{
    vec4f boxSteps;
    boxSteps.x = (m_maxPos[m_frame].x - m_minPos[m_frame].x) / boxSize;
    boxSteps.y = (m_maxPos[m_frame].y - m_minPos[m_frame].y) / boxSize;
    boxSteps.z = (m_maxPos[m_frame].z - m_minPos[m_frame].z) / boxSize;

    boxSteps.x = (boxSteps.x == 0.f) ? 1 : boxSteps.x;
    boxSteps.y = (boxSteps.y == 0.f) ? 1 : boxSteps.y;
    boxSteps.z = (boxSteps.z == 0.f) ? 1 : boxSteps.z;

    std::map<unsigned int, unsigned int> primitivesPerBox;

    // Add primitives to boxes
    unsigned int p = 0;
    size_t maxPrimitivesPerBox = 0;
    for (const auto &prim : m_primitives[m_frame])
    {
        auto &primitive = prim.second;

        const auto &center = primitive.p0;
        unsigned int X =
            static_cast<int>((center.x - m_minPos[m_frame].x) / boxSteps.x);
        unsigned int Y =
            static_cast<int>((center.y - m_minPos[m_frame].y) / boxSteps.y);
        unsigned int Z =
            static_cast<int>((center.z - m_minPos[m_frame].z) / boxSteps.z);
        unsigned int B = 1 + 1000 * (X * boxSize * boxSize + Y * boxSize + Z);

        if (simulate)
            if (primitivesPerBox.find(B) == primitivesPerBox.end())
                primitivesPerBox[B] = 0;
            else
                primitivesPerBox[B]++;
        else
        {
            if (primitivesPerBox.find(B) == primitivesPerBox.end())
            {
                // Create Box B since it does not exist yet
                CPUBoundingBox box;
                memset(&box, 0, sizeof(CPUBoundingBox));
                box.parameters[0].x = m_sceneInfo.viewDistance;
                box.parameters[0].y = m_sceneInfo.viewDistance;
                box.parameters[0].z = m_sceneInfo.viewDistance;
                box.parameters[1].x = -m_sceneInfo.viewDistance;
                box.parameters[1].y = -m_sceneInfo.viewDistance;
                box.parameters[1].z = -m_sceneInfo.viewDistance;
                box.indexForNextBox = 1;
                m_boundingBoxes[m_frame][0].insert(std::make_pair(B, box));
            }

            // Lights
            if (m_hMaterials[primitive.materialId].innerIllumination.x != 0.f)
            {
                // Lights are added to first box of higher level
                m_boundingBoxes[m_frame][m_treeDepth][0].primitives.push_back(
                    p);
                LOG_INFO(3, "[" << m_treeDepth << "] Lamp " << p << " added ("
                                << primitive.p0.x << "," << primitive.p0.y
                                << "," << primitive.p0.z << " "
                                << m_nbActiveLamps[m_frame] << "/"
                                << NB_MAX_LAMPS
                                << "), Material ID=" << primitive.materialId);
            }
            else
            {
                // LOG_INFO(3, "Adding primitive to box " << B);
                m_boundingBoxes[m_frame][0][B].primitives.push_back(p);
                if (m_boundingBoxes[m_frame][0][B].primitives.size() >
                    maxPrimitivesPerBox)
                    maxPrimitivesPerBox =
                        m_boundingBoxes[m_frame][0][B].primitives.size();
            }
        }
        ++p;
    }

    // Now update box sizes
    if (!simulate)
        for (auto &box : m_boundingBoxes[m_frame][0])
            updateBoundingBox(box.second);

    LOG_INFO(3, "Maximum number of primitives per box=" << maxPrimitivesPerBox
                                                        << " for level 0");
    return static_cast<int>(maxPrimitivesPerBox);
}

int GPUKernel::processOutterBoxes(const int boxSize,
                                  const int boundingBoxesDepth)
{
    LOG_INFO(3, "processOutterBoxes(" << boxSize << "," << boundingBoxesDepth
                                      << ")");
    vec4f boxSteps;
    boxSteps.x = (m_maxPos[m_frame].x - m_minPos[m_frame].x) / boxSize;
    boxSteps.y = (m_maxPos[m_frame].y - m_minPos[m_frame].y) / boxSize;
    boxSteps.z = (m_maxPos[m_frame].z - m_minPos[m_frame].z) / boxSize;

    boxSteps.x = (boxSteps.x == 0.f) ? 1 : boxSteps.x;
    boxSteps.y = (boxSteps.y == 0.f) ? 1 : boxSteps.y;
    boxSteps.z = (boxSteps.z == 0.f) ? 1 : boxSteps.z;

    // Create boxes in Rubik's cube mode :-)
    size_t maxPrimitivesPerBox(0);
    for (const auto &box : m_boundingBoxes[m_frame][boundingBoxesDepth - 1])
    {
        const auto &center = box.second.center;
        int X = static_cast<int>((center.x - m_minPos[m_frame].x) / boxSteps.x);
        int Y = static_cast<int>((center.y - m_minPos[m_frame].y) / boxSteps.y);
        int Z = static_cast<int>((center.z - m_minPos[m_frame].z) / boxSteps.z);
        int B = (X * boxSize * boxSize + Y * boxSize + Z);

        B++; // Index 0 is used to store lights, so we start storing primitives
             // from index 1

        // Box B
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].parameters[0].x =
            m_sceneInfo.viewDistance;
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].parameters[0].y =
            m_sceneInfo.viewDistance;
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].parameters[0].z =
            m_sceneInfo.viewDistance;
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].parameters[1].x =
            -m_sceneInfo.viewDistance;
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].parameters[1].y =
            -m_sceneInfo.viewDistance;
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].parameters[1].z =
            -m_sceneInfo.viewDistance;
        m_boundingBoxes[m_frame][boundingBoxesDepth][B].primitives.push_back(
            box.first);
        if (m_boundingBoxes[m_frame][boundingBoxesDepth][B].primitives.size() >
            maxPrimitivesPerBox)
            maxPrimitivesPerBox =
                m_boundingBoxes[m_frame][boundingBoxesDepth][B]
                    .primitives.size();
    }
    LOG_INFO(3, "Depth " << boundingBoxesDepth << ": "
                         << m_boundingBoxes[m_frame][boundingBoxesDepth].size()
                         << " created");

    // Now update box sizes
    for (auto &box : m_boundingBoxes[m_frame][boundingBoxesDepth])
        updateOutterBoundingBox(box.second, boundingBoxesDepth - 1);

    LOG_INFO(3, "Maximum number of sub-boxes per box=" << maxPrimitivesPerBox
                                                       << " for level "
                                                       << boundingBoxesDepth);
    return static_cast<int>(maxPrimitivesPerBox);
}

int GPUKernel::compactBoxes(bool reconstructBoxes)
{
    LOG_INFO(3, "GPUKernel::compactBoxes ("
                    << (reconstructBoxes ? "true" : "false") << ")");

    // First box of highest level is dedicated to light sources
    m_primitivesTransfered = false;
    if (reconstructBoxes)
    {
        resetBox(m_boundingBoxes[m_frame][m_treeDepth][0], true);
        int gridGranularity(2);
        int gridDivider(4);

        // This is needed to determine the scene depth before calling
        // the processBoxes method. It's because lights are stored in
        // the higher level layer of the tree
        m_treeDepth = 0;
        int nbBoxes = static_cast<int>(m_primitives[m_frame].size());
        while (nbBoxes > gridGranularity)
        {
            ++m_treeDepth;
            nbBoxes /= gridDivider;
        }

        // Dispatch primitives into level 0 boxes
        processBoxes(AABB_MAGIC_NUMBER, false);

        // Construct sub-boxes (level 1 and +)
        m_treeDepth = 0;
        nbBoxes = static_cast<int>(m_primitives[m_frame].size());
        do
        {
            ++m_treeDepth;
            processOutterBoxes(nbBoxes, m_treeDepth);
            nbBoxes /= gridDivider;
        } while (nbBoxes > gridGranularity);
        LOG_INFO(2, "Primitives.........: " << m_primitives[m_frame].size());
        LOG_INFO(2, "Scene depth........: " << m_treeDepth);
    }

    LOG_INFO(3, "Streaming data to GPU");
    streamDataToGPU();
    return static_cast<int>(m_nbActiveBoxes[m_frame]);
}

void GPUKernel::recursiveDataStreamToGPU(const int depth,
                                         std::vector<long> &elements)
{
    LOG_INFO(3, "RecursiveDataStreamToGPU(" << depth << ")");
    LOG_INFO(3,
             "Depth " << depth << " contains " << elements.size() << " boxes");

    size_t c = 0;
    for (const auto &element : elements)
    {
        // Create Box
        CPUBoundingBox &box = m_boundingBoxes[m_frame][depth][element];

        if (box.primitives.size() != 0 &&
            m_nbActiveBoxes[m_frame] < NB_MAX_BOXES)
        {
            int boxIndex = m_nbActiveBoxes[m_frame];
            m_hBoundingBoxes[boxIndex].parameters[0] = box.parameters[0];
            m_hBoundingBoxes[boxIndex].parameters[1] = box.parameters[1];
            m_hBoundingBoxes[boxIndex].nbPrimitives =
                (depth == 0) ? static_cast<int>(box.primitives.size()) : 0;
            m_hBoundingBoxes[boxIndex].startIndex =
                (depth == 0) ? m_nbActivePrimitives[m_frame] : depth;
            LOG_INFO(3,
                     "==> Box " << boxIndex << " Depth [" << depth << "] ++");
            ++m_nbActiveBoxes[m_frame];
            ++c;

            if (depth == 0)
            {
                LOG_INFO(3, "=== Box " << boxIndex << " Depth [" << depth
                                       << "] ... adding "
                                       << box.primitives.size()
                                       << " primitives");
                m_maxPrimitivesPerBox =
                    (box.primitives.size() > m_maxPrimitivesPerBox)
                        ? box.primitives.size()
                        : m_maxPrimitivesPerBox;
                // Add primitive for Nodes
                std::vector<long>::const_iterator itp = box.primitives.begin();
                while (itp != box.primitives.end())
                {
                    // Prepare primitives for GPU
                    if ((*itp) < NB_MAX_PRIMITIVES)
                    {
                        CPUPrimitive &primitive = (m_primitives[m_frame])[*itp];
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].index =
                            (*itp);
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].type =
                            primitive.type;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].p0 =
                            primitive.p0;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].p1 =
                            primitive.p1;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].p2 =
                            primitive.p2;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].n0 =
                            primitive.n0;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].n1 =
                            primitive.n1;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].n2 =
                            primitive.n2;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].size =
                            primitive.size;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]]
                            .materialId = primitive.materialId;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].vt0 =
                            primitive.vt0;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].vt1 =
                            primitive.vt1;
                        m_hPrimitives[m_nbActivePrimitives[m_frame]].vt2 =
                            primitive.vt2;
                        ++m_nbActivePrimitives[m_frame];
                    }
                    ++itp;
                }
            }
            else
                // Resursively continue to build the tree
                recursiveDataStreamToGPU(depth - 1, box.primitives);

            m_hBoundingBoxes[boxIndex].indexForNextBox.x =
                (depth == 0) ? 1 : m_nbActiveBoxes[m_frame] - boxIndex;
            LOG_INFO(3,
                     "<== Box "
                         << boxIndex << " Depth [" << depth << "] --> "
                         << m_hBoundingBoxes[boxIndex].indexForNextBox.x
                         << " = box "
                         << boxIndex +
                                m_hBoundingBoxes[boxIndex].indexForNextBox.x);
        }
    }
}

void GPUKernel::streamDataToGPU()
{
    LOG_INFO(3, "GPUKernel::streamDataToGPU");
    // --------------------------------------------------------------------------------
    // Transform data for ray-tracer
    // CPU -> GPU
    // --------------------------------------------------------------------------------
    m_primitivesTransfered = false;
    m_nbActiveBoxes[m_frame] = 0;
    m_nbActivePrimitives[m_frame] = 0;
    m_nbActiveLamps[m_frame] = 0;
    m_maxPrimitivesPerBox = 0;

    // Build boxes tree recursively
    int maxDepth(m_treeDepth);
    LOG_INFO(3, "Processing " << m_boundingBoxes[m_frame][maxDepth].size()
                              << " master boxes");
    BoxContainer::iterator itob = m_boundingBoxes[m_frame][maxDepth].begin();
    while (itob != m_boundingBoxes[m_frame][maxDepth].end())
    {
        // Create Box
        CPUBoundingBox &box = (*itob).second;
        int boxIndex = m_nbActiveBoxes[m_frame];
        LOG_INFO(3, "==> Box " << boxIndex << " Depth [" << maxDepth << "] ++");
        m_hBoundingBoxes[boxIndex].parameters[0] = box.parameters[0];
        m_hBoundingBoxes[boxIndex].parameters[1] = box.parameters[1];
        m_hBoundingBoxes[boxIndex].nbPrimitives = 0;
        m_hBoundingBoxes[boxIndex].startIndex = maxDepth;

        if (itob == m_boundingBoxes[m_frame][maxDepth].begin())
        {
            LOG_INFO(3, "Box 0 of higher level (" << 0 << ") contains ligths");
            m_lightInformationSize = 0;
            m_hBoundingBoxes[boxIndex].parameters[0].x =
                -m_sceneInfo.viewDistance;
            m_hBoundingBoxes[boxIndex].parameters[0].y =
                -m_sceneInfo.viewDistance;
            m_hBoundingBoxes[boxIndex].parameters[0].z =
                -m_sceneInfo.viewDistance;
            m_hBoundingBoxes[boxIndex].parameters[1].x =
                m_sceneInfo.viewDistance;
            m_hBoundingBoxes[boxIndex].parameters[1].y =
                m_sceneInfo.viewDistance;
            m_hBoundingBoxes[boxIndex].parameters[1].z =
                m_sceneInfo.viewDistance;
            m_hBoundingBoxes[boxIndex].nbPrimitives =
                static_cast<int>(box.primitives.size());
            m_hBoundingBoxes[boxIndex].startIndex = 0;
            std::vector<long>::const_iterator itp = box.primitives.begin();
            while (itp != box.primitives.end())
            {
                // Add the primitive
                CPUPrimitive &primitive = (m_primitives[m_frame])[*itp];
                m_hPrimitives[m_nbActivePrimitives[m_frame]].index = (*itp);
                m_hPrimitives[m_nbActivePrimitives[m_frame]].type =
                    primitive.type;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].p0 = primitive.p0;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].p1 = primitive.p1;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].p2 = primitive.p2;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].n0 = primitive.n0;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].n1 = primitive.n1;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].n2 = primitive.n2;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].size =
                    primitive.size;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].materialId =
                    primitive.materialId;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].vt0 =
                    primitive.vt0;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].vt1 =
                    primitive.vt1;
                m_hPrimitives[m_nbActivePrimitives[m_frame]].vt2 =
                    primitive.vt2;
                ++m_nbActivePrimitives[m_frame];

                // Add light information related to primitive
                Material &material = m_hMaterials[primitive.materialId];
                LightInformation lightInformation;
                LOG_INFO(3, "LightInformation " << (*itp) << ", MaterialId="
                                                << primitive.materialId);
                lightInformation.primitiveId = (*itp);
                lightInformation.materialId = primitive.materialId;

                lightInformation.location.x = primitive.p0.x;
                lightInformation.location.y = primitive.p0.y;
                lightInformation.location.z = primitive.p0.z;

                lightInformation.color.x = material.color.x;
                lightInformation.color.y = material.color.y;
                lightInformation.color.z = material.color.z;
                lightInformation.color.w = material.innerIllumination.x;

                m_lightInformation[m_lightInformationSize] = lightInformation;

                LOG_INFO(
                    3,
                    "Adding Light Information: "
                        << m_lightInformation[m_lightInformationSize]
                               .primitiveId
                        << ","
                        << m_lightInformation[m_lightInformationSize].materialId
                        << ":"
                        << m_lightInformation[m_lightInformationSize].location.x
                        << ","
                        << m_lightInformation[m_lightInformationSize].location.y
                        << ","
                        << m_lightInformation[m_lightInformationSize].location.z
                        << " "
                        << m_lightInformation[m_lightInformationSize].color.x
                        << ","
                        << m_lightInformation[m_lightInformationSize].color.y
                        << ","
                        << m_lightInformation[m_lightInformationSize].color.z
                        << " "
                        << m_lightInformation[m_lightInformationSize].color.w);

                m_hLamps[m_nbActiveLamps[m_frame]] = *itp;
                ++m_nbActiveLamps[m_frame];
                ++m_lightInformationSize;
                ++itp;
            }
        }

        ++m_nbActiveBoxes[m_frame];

        // Recursively populate flattened tree representation
        if (maxDepth > 0)
            recursiveDataStreamToGPU(maxDepth - 1, box.primitives);

        m_hBoundingBoxes[boxIndex].indexForNextBox.x =
            m_nbActiveBoxes[m_frame] - boxIndex;
        LOG_INFO(3, "Master Primitive ("
                        << box.parameters[0].x << "," << box.parameters[0].y
                        << "," << box.parameters[0].z << "),("
                        << box.parameters[1].x << "," << box.parameters[1].y
                        << "," << box.parameters[1].z << "),"
                        << m_hBoundingBoxes[boxIndex].indexForNextBox.x);
        ++itob;
    }

    LOG_INFO(3, "Max primitives per box: " << m_maxPrimitivesPerBox);

    // Build global illumination structures
    // buildLightInformationFromTexture(4);

    // Done
    LOG_INFO(3, "Compacted " << m_nbActiveBoxes[m_frame] << " boxes, "
                             << m_nbActivePrimitives[m_frame]
                             << " primitives and " << m_nbActiveLamps[m_frame]
                             << " lamps");
    if (m_nbActivePrimitives[m_frame] != m_primitives[m_frame].size())
    {
        LOG_ERROR("Lost primitives on the way for frame "
                  << m_frame << "... " << m_nbActivePrimitives[m_frame]
                  << "!=" << m_primitives[m_frame].size());
    }

    for (int i(0); i < m_nbActiveBoxes[m_frame]; ++i)
    {
        if (i + m_hBoundingBoxes[i].indexForNextBox.x >
            m_nbActiveBoxes[m_frame])
        {
            LOG_ERROR("Box " << i << " --> "
                             << i + m_hBoundingBoxes[i].indexForNextBox.x);
        }
    }
}

void GPUKernel::resetFrame()
{
    LOG_INFO(3, "Resetting frame " << m_frame);
    memset(&m_translation, 0, sizeof(vec4f));
    memset(&m_rotation, 0, sizeof(vec4f));

    m_currentMaterial = 0;

    for (int i(0); i < BOUNDING_BOXES_TREE_DEPTH; ++i)
        m_boundingBoxes[m_frame][i].clear();

    m_boundingBoxes[m_frame][0].clear();
    m_nbActiveBoxes[m_frame] = 0;
    LOG_INFO(3, "Nb Boxes: " << m_boundingBoxes[m_frame][0].size());

    m_primitives[m_frame].clear();
    m_nbActivePrimitives[m_frame] = 0;
    LOG_INFO(3, "Nb Primitives: " << m_primitives[m_frame].size());

    m_lamps[m_frame].clear();
    m_nbActiveLamps[m_frame] = 0;
    LOG_INFO(3, "Nb Lamps: " << m_lamps[m_frame].size());
}

void GPUKernel::resetAll()
{
    LOG_INFO(3, "Resetting frames");

#ifdef USE_OCULUS
    if (m_sensorFusion->IsAttachedToSensor())
        m_sensorFusion->Reset();
#endif // USE_OCULUS

    int oldFrame(m_frame);
    for (int frame(0); frame < NB_MAX_FRAMES; ++frame)
    {
        m_frame = frame;
        resetFrame();
    }
    m_frame = oldFrame;
    m_primitivesTransfered = false;

    LOG_INFO(3, "Resetting textures and materials");
    m_nbActiveMaterials = -1;
    m_materialsTransfered = false;

    for (int i(0); i < NB_MAX_TEXTURES; ++i)
    {
#ifdef USE_KINECT
        if (i > 1 && m_hTextures[i].buffer != 0)
            delete[] m_hTextures[i].buffer;
#else
        if (m_hTextures[i].buffer)
        {
            LOG_INFO(3, "[resetAll] Buffer " << i << " needs to be released");
            delete[] m_hTextures[i].buffer;
            m_hTextures[i].buffer = 0;
        }
#endif // USE_KINECT
    }
    memset(&m_hTextures[0], 0, NB_MAX_TEXTURES * sizeof(TextureInfo));
    m_nbActiveTextures = 0;
    m_texturesTransfered = false;
#ifdef USE_KINECT
    initializeKinectTextures();
#endif // USE_KINECT
}

void GPUKernel::displayBoxesInfo()
{
    for (unsigned int i(0); i <= m_boundingBoxes[m_frame][0].size(); ++i)
    {
        CPUBoundingBox &box = m_boundingBoxes[m_frame][0][i];
        LOG_INFO(3, "Box " << i);
        LOG_INFO(3, "- # of primitives: " << box.primitives.size());
        LOG_INFO(3, "- Corners 1      : " << box.parameters[0].x << ","
                                          << box.parameters[0].y << ","
                                          << box.parameters[0].z);
        LOG_INFO(3, "- Corners 2      : " << box.parameters[1].x << ","
                                          << box.parameters[1].y << ","
                                          << box.parameters[1].z);
        unsigned int p(0);
        std::vector<long>::const_iterator it = box.primitives.begin();
        while (it != box.primitives.end())
        {
            CPUPrimitive &primitive((m_primitives[m_frame])[*it]);
            LOG_INFO(3, "- - "
                            << p << ":"
                            << "type = " << primitive.type << ", "
                            << "center = (" << primitive.p0.x << ","
                            << primitive.p0.y << "," << primitive.p0.z << "), "
                            << "p1 = (" << primitive.p1.x << ","
                            << primitive.p1.y << "," << primitive.p1.z << ")");
            ++p;
            ++it;
        }
    }
}

void GPUKernel::rotatePrimitives(const vec3f &rotationCenter,
                                 const vec4f &angles)
{
    LOG_INFO(3, "GPUKernel::rotatePrimitives");

    m_primitivesTransfered = false;
    vec3f cosAngles, sinAngles;

    cosAngles.x = cos(angles.x);
    cosAngles.y = cos(angles.y);
    cosAngles.z = cos(angles.z);

    sinAngles.x = sin(angles.x);
    sinAngles.y = sin(angles.y);
    sinAngles.z = sin(angles.z);

#pragma omp parallel
    for (BoxContainer::iterator itb = m_boundingBoxes[m_frame][0].begin();
         itb != m_boundingBoxes[m_frame][0].end(); ++itb)
    {
#pragma omp single nowait
        {
            CPUBoundingBox &box = (*itb).second;
            resetBox(box, false);

            for (std::vector<long>::iterator it = box.primitives.begin();
                 it != box.primitives.end(); ++it)
            {
                //#pragma single nowait
                CPUPrimitive &primitive((m_primitives[m_frame])[*it]);
                if (primitive.movable && primitive.type != ptCamera)
                {
#if 0
                    float limit = -3000.f;
                    if( primitive.speed0.y != 0.f && (primitive.p0.y > limit || primitive.p1.y > limit || primitive.p2.y > limit) )
                    {
                        // Fall
                        primitive.p0.y += primitive.speed0.y;
                        primitive.p1.y += primitive.speed0.y;
                        primitive.p2.y += primitive.speed0.y;

                        if( primitive.p0.y < limit-primitive.size.x )
                        {
                            primitive.speed0.y = -primitive.speed0.y/1.6f;
                            // Rotate
                            vec4f center = (primitive.p0.y < primitive.p1.y) ? primitive.p0 : primitive.p1;
                            center = (primitive.p2.y < center.y) ? primitive.p2 : center;


                            //center.x = (primitive.p0.x + primitive.p1.x + primitive.p2.x) / 3.f;
                            // center.y = (primitive.p0.y + primitive.p1.y + primitive.p2.y) / 3.f;
                            //center.z = (primitive.p0.z + primitive.p1.z + primitive.p2.z) / 3.f;

                            rotatePrimitive( primitive, center, cosAngles, sinAngles );
                        }
                        primitive.speed0.y -= 4+(rand()%400/100.f);
                    }
                    else
                    {
                        primitive.speed0.y = 0.f;
                    }
#else
                    rotatePrimitive(primitive, rotationCenter, cosAngles,
                                    sinAngles);
#endif // 0
                }
                updateBoundingBox(box);
            }
        }
    }

    // Update bounding boxes
    for (int b(1); b < BOUNDING_BOXES_TREE_DEPTH; ++b)
    {
#pragma omp parallel
        for (BoxContainer::iterator itb = m_boundingBoxes[m_frame][b].begin();
             itb != m_boundingBoxes[m_frame][b].end(); ++itb)
        {
#pragma omp single nowait
            {
                CPUBoundingBox &box = (*itb).second;
                updateOutterBoundingBox(box, b - 1);
            }
        }
    }
}

void GPUKernel::translatePrimitives(const vec3f &translation)
{
    LOG_INFO(3, "GPUKernel::translatePrimitives ("
                    << m_boundingBoxes[m_frame][0].size() << ")");
    m_primitivesTransfered = false;
    for (BoxContainer::iterator itb = m_boundingBoxes[m_frame][0].begin();
         itb != m_boundingBoxes[m_frame][0].end(); ++itb)
    {
#pragma omp single nowait
        {
            CPUBoundingBox &box = (*itb).second;
            resetBox(box, false);

            for (std::vector<long>::iterator it = box.primitives.begin();
                 it != box.primitives.end(); ++it)
            {
                //#pragma single nowait
                CPUPrimitive &primitive((m_primitives[m_frame])[*it]);
                if (primitive.movable && primitive.type != ptCamera)
                {
                    primitive.p0.x += translation.x;
                    primitive.p0.y += translation.y;
                    primitive.p0.z += translation.z;

                    primitive.p1.x += translation.x;
                    primitive.p1.y += translation.y;
                    primitive.p1.z += translation.z;

                    primitive.p2.x += translation.x;
                    primitive.p2.y += translation.y;
                    primitive.p2.z += translation.z;
                }
                updateBoundingBox(box);
            }
        }
    }

    // Update bounding boxes
    for (int b(1); b < BOUNDING_BOXES_TREE_DEPTH; ++b)
    {
#pragma omp parallel
        for (BoxContainer::iterator itb = m_boundingBoxes[m_frame][b].begin();
             itb != m_boundingBoxes[m_frame][b].end(); ++itb)
        {
#pragma omp single nowait
            {
                CPUBoundingBox &box = (*itb).second;
                updateOutterBoundingBox(box, b - 1);
            }
        }
    }
}

void GPUKernel::morphPrimitives()
{
    LOG_INFO(3, "Morphing frames " << m_nbFrames);
    for (unsigned int frame(1); frame < (m_nbFrames - 1); ++frame)
    {
        LOG_INFO(3, "Morphing frame " << frame << ", " << m_primitives[0].size()
                                      << " primitives");
        setFrame(frame);
        resetFrame();
        PrimitiveContainer::iterator it2 = m_primitives[m_nbFrames - 1].begin();
        for (PrimitiveContainer::iterator it1 = m_primitives[0].begin();
             it1 != m_primitives[0].end() &&
             it2 != m_primitives[m_nbFrames - 1].end();
             ++it1)
        {
            CPUPrimitive &primitive1((*it1).second);
            CPUPrimitive &primitive2((*it2).second);
            vec3f p0, p1, p2;
            vec3f n0, n1, n2;
            vec3f size;
            float r =
                static_cast<float>(m_frame) / static_cast<float>(m_nbFrames);
            p0.x = primitive1.p0.x + r * (primitive2.p0.x - primitive1.p0.x);
            p0.y = primitive1.p0.y + r * (primitive2.p0.y - primitive1.p0.y);
            p0.z = primitive1.p0.z + r * (primitive2.p0.z - primitive1.p0.z);

            p1.x = primitive1.p1.x + r * (primitive2.p1.x - primitive1.p1.x);
            p1.y = primitive1.p1.y + r * (primitive2.p1.y - primitive1.p1.y);
            p1.z = primitive1.p1.z + r * (primitive2.p1.z - primitive1.p1.z);

            p2.x = primitive1.p2.x + r * (primitive2.p2.x - primitive1.p2.x);
            p2.y = primitive1.p2.y + r * (primitive2.p2.y - primitive1.p2.y);
            p2.z = primitive1.p2.z + r * (primitive2.p2.z - primitive1.p2.z);

            n0.x = primitive1.n0.x + r * (primitive2.n0.x - primitive1.n0.x);
            n0.y = primitive1.n0.y + r * (primitive2.n0.y - primitive1.n0.y);
            n0.z = primitive1.n0.z + r * (primitive2.n0.z - primitive1.n0.z);

            n1.x = primitive1.n1.x + r * (primitive2.n1.x - primitive1.n1.x);
            n1.y = primitive1.n1.y + r * (primitive2.n1.y - primitive1.n1.y);
            n1.z = primitive1.n1.z + r * (primitive2.n1.z - primitive1.n1.z);

            n2.x = primitive1.n2.x + r * (primitive2.n2.x - primitive1.n2.x);
            n2.y = primitive1.n2.y + r * (primitive2.n2.y - primitive1.n2.y);
            n2.z = primitive1.n2.z + r * (primitive2.n2.z - primitive1.n2.z);

            size.x =
                primitive1.size.x + r * (primitive2.size.x - primitive1.size.x);
            size.y =
                primitive1.size.y + r * (primitive2.size.y - primitive1.size.y);
            size.z =
                primitive1.size.z + r * (primitive2.size.z - primitive1.size.z);

            int i = addPrimitive(PrimitiveType(primitive1.type));
            setPrimitive(i, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y,
                         p2.z, size.x, size.y, size.z, primitive1.materialId);

            setPrimitiveNormals(i, n0, n1, n2);
            setPrimitiveTextureCoordinates(i, primitive1.vt0, primitive1.vt1,
                                           primitive1.vt2);

            setPrimitiveIsMovable(i, primitive1.movable);

            ++it2;
        }
        compactBoxes(true);
    }
}

void GPUKernel::scalePrimitives(float scale, unsigned int from, unsigned int to)
{
    LOG_INFO(3, "GPUKernel::scalePrimitives(" << from << "->" << to << ")");
    m_primitivesTransfered = false;

    PrimitiveContainer::iterator it = (m_primitives[m_frame]).begin();
    while (it != (m_primitives[m_frame]).end())
    {
        CPUPrimitive &primitive((*it).second);
        primitive.p0.x *= scale;
        primitive.p0.y *= scale;
        primitive.p0.z *= scale;

        primitive.p1.x *= scale;
        primitive.p1.y *= scale;
        primitive.p1.z *= scale;

        primitive.p2.x *= scale;
        primitive.p2.y *= scale;
        primitive.p2.z *= scale;

        primitive.size.x *= scale;
        primitive.size.y *= scale;
        primitive.size.z *= scale;
        ++it;
    }
}

void GPUKernel::rotateVector(vec3f &v, const vec3f &rotationCenter,
                             const vec3f &cosAngles, const vec3f &sinAngles)
{
    // Rotate Center
    vec3f vector;
    vector.x = v.x - rotationCenter.x;
    vector.y = v.y - rotationCenter.y;
    vector.z = v.z - rotationCenter.z;
    vec3f result = vector;

    /* X axis */
    result.y = vector.y * cosAngles.x - vector.z * sinAngles.x;
    result.z = vector.y * sinAngles.x + vector.z * cosAngles.x;
    vector = result;
    result = vector;

    /* Y axis */
    result.z = vector.z * cosAngles.y - vector.x * sinAngles.y;
    result.x = vector.z * sinAngles.y + vector.x * cosAngles.y;
    vector = result;
    result = vector;

    /* Z axis */
    result.x = vector.x * cosAngles.z - vector.y * sinAngles.z;
    result.y = vector.x * sinAngles.z + vector.y * cosAngles.z;

    v.x = result.x + rotationCenter.x;
    v.y = result.y + rotationCenter.y;
    v.z = result.z + rotationCenter.z;
}

void GPUKernel::rotateBox(CPUBoundingBox &box, vec3f rotationCenter,
                          vec3f cosAngles, vec3f sinAngles)
{
    LOG_INFO(3, "GPUKernel::rotatePrimitive");
    rotateVector(box.parameters[0], rotationCenter, cosAngles, sinAngles);
    rotateVector(box.parameters[1], rotationCenter, cosAngles, sinAngles);
}

void GPUKernel::rotatePrimitive(CPUPrimitive &primitive,
                                const vec3f &rotationCenter,
                                const vec3f &cosAngles, const vec3f &sinAngles)
{
    LOG_INFO(3, "GPUKernel::rotatePrimitive");
    rotateVector(primitive.p0, rotationCenter, cosAngles, sinAngles);
    if (primitive.type == ptCylinder || primitive.type == ptTriangle)
    {
        rotateVector(primitive.p1, rotationCenter, cosAngles, sinAngles);
        rotateVector(primitive.p2, rotationCenter, cosAngles, sinAngles);
        // Rotate Normals
        vec3f zeroCenter = make_vec3f();
        rotateVector(primitive.n0, zeroCenter, cosAngles, sinAngles);
        rotateVector(primitive.n1, zeroCenter, cosAngles, sinAngles);
        rotateVector(primitive.n2, zeroCenter, cosAngles, sinAngles);
        if (primitive.type == ptCylinder)
        {
            // Axis
            vec4f axis;
            axis.x = primitive.p1.x - primitive.p0.x;
            axis.y = primitive.p1.y - primitive.p0.y;
            axis.z = primitive.p1.z - primitive.p0.z;
            float len =
                sqrtf(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (len != 0)
            {
                axis.x /= len;
                axis.y /= len;
                axis.z /= len;
            }
            primitive.n1.x = axis.x;
            primitive.n1.y = axis.y;
            primitive.n1.z = axis.z;
        }
    }
}

vec4f GPUKernel::getPrimitiveCenter(unsigned int index)
{
    vec4f center = make_vec4f();
    if (index <= m_primitives[m_frame].size())
    {
        center.x = (m_primitives[m_frame])[index].p0.x;
        center.y = (m_primitives[m_frame])[index].p0.y;
        center.z = (m_primitives[m_frame])[index].p0.z;
    }
    return center;
}

void GPUKernel::getPrimitiveOtherCenter(unsigned int index, vec3f &center)
{
    if (index <= m_primitives[m_frame].size())
        center = (m_primitives[m_frame])[index].p1;
}

void GPUKernel::setPrimitiveCenter(unsigned int index, const vec3f &center)
{
    m_primitivesTransfered = false;

    // TODO, Box needs to be updated
    if (index <= m_primitives[m_frame].size())
    {
        (m_primitives[m_frame])[index].p0.x = center.x;
        (m_primitives[m_frame])[index].p0.y = center.y;
        (m_primitives[m_frame])[index].p0.z = center.z;
    }
}

int GPUKernel::addCube(float x, float y, float z, float radius, int materialId)
{
    LOG_INFO(3, "GPUKernel::addCube(" << m_frame << ")");
    return addRectangle(x, y, z, radius, radius, radius, materialId);
}

int GPUKernel::addRectangle(float x, float y, float z, float w, float h,
                            float d, int materialId)
{
    LOG_INFO(3, "GPUKernel::addRectangle(" << m_frame << ")");
    int returnValue;
    // Back
    returnValue = addPrimitive(ptXYPlane);
    setPrimitive(returnValue, x, y, z + d, w, h, d, materialId);

    // Front
    returnValue = addPrimitive(ptXYPlane);
    setPrimitive(returnValue, x, y, z - d, w, h, d, materialId);

    // Left
    returnValue = addPrimitive(ptYZPlane);
    setPrimitive(returnValue, x - w, y, z, w, h, d, materialId);

    // Right
    returnValue = addPrimitive(ptYZPlane);
    setPrimitive(returnValue, x + w, y, z, w, h, d, materialId);

    // Top
    returnValue = addPrimitive(ptXZPlane);
    setPrimitive(returnValue, x, y + h, z, w, h, d, materialId);

    // Bottom
    returnValue = addPrimitive(ptXZPlane);
    setPrimitive(returnValue, x, y - h, z, w, h, d, materialId);
    return returnValue;
}

void GPUKernel::setPrimitiveMaterial(unsigned int index, int materialId)
{
    LOG_INFO(3, "GPUKernel::setPrimitiveMaterial(" << index << "," << materialId
                                                   << ")");
    if (index <= m_primitives[m_frame].size())
    {
        (m_primitives[m_frame])[index].materialId = materialId;
        // TODO: updateLight( index );
    }
}

int GPUKernel::getPrimitiveMaterial(unsigned int index)
{
    LOG_INFO(3, "GPUKernel::getPrimitiveMaterial(" << index << ")");
    unsigned int returnValue(-1);
    if (index <= m_primitives[m_frame].size())
    {
        returnValue = (m_primitives[m_frame])[index].materialId;
    }
    return returnValue;
}

// ---------- Materials ----------
int GPUKernel::addMaterial()
{
    LOG_INFO(3, "GPUKernel::addMaterial");
    m_nbActiveMaterials++;
    LOG_INFO(3, "m_nbActiveMaterials = " << m_nbActiveMaterials);
    return m_nbActiveMaterials;
}

void GPUKernel::setMaterial(unsigned int index, const Material &material)
{
    if (index < NB_MAX_MATERIALS)
    {
        m_hMaterials[index] = material;
        m_materialsTransfered = false;
    }
}

void GPUKernel::setMaterial(
    unsigned int index, float r, float g, float b, float gloss,
    float reflection, float refraction, bool procedural, bool wireframe,
    int wireframeWidth, float transparency, float opacity, int diffuseTextureId,
    int normalTextureId, int bumpTextureId, int specularTextureId,
    int reflectionTextureId, int transparentTextureId,
    int ambientOcclusionTextureId, float specValue, float specPower,
    float specCoef, float innerIllumination, float illuminationDiffusion,
    float illuminationPropagation, bool fastTransparency)
{
    LOG_INFO(3, "GPUKernel::setMaterial ("
                    << index << "," << static_cast<float>(r) << ","
                    << static_cast<float>(g) << "," << static_cast<float>(b)
                    << "," << static_cast<float>(gloss) << ","
                    << static_cast<float>(reflection) << ","
                    << static_cast<float>(refraction) << "," << procedural
                    << "," << wireframe << ","
                    << static_cast<int>(wireframeWidth) << ","
                    << static_cast<float>(transparency) << ","
                    << static_cast<float>(opacity) << ","
                    << static_cast<int>(diffuseTextureId) << ","
                    << static_cast<int>(normalTextureId) << ","
                    << static_cast<int>(bumpTextureId) << ","
                    << static_cast<int>(specularTextureId) << ","
                    << static_cast<int>(reflectionTextureId) << ","
                    << static_cast<int>(transparentTextureId) << ","
                    << static_cast<float>(specValue) << ","
                    << static_cast<float>(specPower) << ","
                    << static_cast<float>(specCoef) << ","
                    << static_cast<float>(innerIllumination) << ","
                    << static_cast<float>(illuminationDiffusion) << ","
                    << static_cast<float>(illuminationPropagation) << ","
                    << fastTransparency << ")");

    if (index < NB_MAX_MATERIALS)
    {
        m_hMaterials[index].color.x = r;
        m_hMaterials[index].color.y = g;
        m_hMaterials[index].color.z = b;
        m_hMaterials[index].color.w = gloss / 100.f;
        m_hMaterials[index].specular.x = specValue;
        m_hMaterials[index].specular.y = specPower;
        m_hMaterials[index].specular.z = 0.f; // Not used
        m_hMaterials[index].specular.w = specCoef;
        m_hMaterials[index].innerIllumination.x = innerIllumination;
        m_hMaterials[index].innerIllumination.y = illuminationDiffusion;
        m_hMaterials[index].innerIllumination.z = illuminationPropagation;
        m_hMaterials[index].innerIllumination.w = 0.f; // noise;
        m_hMaterials[index].reflection = reflection;
        m_hMaterials[index].refraction = refraction;
        m_hMaterials[index].transparency = transparency;
        m_hMaterials[index].opacity = opacity;
        m_hMaterials[index].attributes.x = fastTransparency ? 1 : 0;
        m_hMaterials[index].attributes.y = procedural ? 1 : 0;
        m_hMaterials[index].attributes.z =
            wireframe ? ((wireframeWidth == 0) ? 1 : 2) : 0;
        m_hMaterials[index].attributes.w = wireframeWidth;
        m_hMaterials[index].textureMapping.x = 1;
        m_hMaterials[index].textureMapping.y = 1;
        m_hMaterials[index].textureMapping.z = TEXTURE_NONE; // Deprecated
        m_hMaterials[index].textureMapping.w = 0;
        m_hMaterials[index].textureIds.x = diffuseTextureId;
        m_hMaterials[index].textureIds.y = normalTextureId;
        m_hMaterials[index].textureIds.z = bumpTextureId;
        m_hMaterials[index].textureIds.w = specularTextureId;
        m_hMaterials[index].advancedTextureIds.x = reflectionTextureId;
        m_hMaterials[index].advancedTextureIds.y = transparentTextureId;
        m_hMaterials[index].advancedTextureIds.z = ambientOcclusionTextureId;
        m_hMaterials[index].advancedTextureIds.w = TEXTURE_NONE;
        m_hMaterials[index].advancedTextureOffset.x = 0;
        m_hMaterials[index].advancedTextureOffset.y = 0;
        m_hMaterials[index].advancedTextureOffset.z = 0;
        m_hMaterials[index].advancedTextureOffset.w = 0;
        m_hMaterials[index].mappingOffset.x = 1.f;
        m_hMaterials[index].mappingOffset.y = 1.f;
#ifdef USE_KINECT
        switch (diffuseTextureId)
        {
        case KINECT_COLOR_TEXTURE:
            m_hMaterials[index].textureMapping.x = KINECT_COLOR_WIDTH; // Width
            m_hMaterials[index].textureMapping.y =
                KINECT_COLOR_HEIGHT;                                   // Height
            m_hMaterials[index].textureMapping.w = KINECT_COLOR_DEPTH; // Depth
            break;
        case KINECT_DEPTH_TEXTURE:
            m_hMaterials[index].textureMapping.x = KINECT_DEPTH_WIDTH; // Width
            m_hMaterials[index].textureMapping.y =
                KINECT_DEPTH_HEIGHT;                                   // Height
            m_hMaterials[index].textureMapping.w = KINECT_DEPTH_DEPTH; // Depth
            break;
        }
#endif // USE_KINECT

        if (diffuseTextureId >= 0 && diffuseTextureId < m_nbActiveTextures)
        {
            m_hMaterials[index].textureMapping.x =
                m_hTextures[diffuseTextureId].size.x; // Width
            m_hMaterials[index].textureMapping.y =
                m_hTextures[diffuseTextureId].size.y; // Height
            m_hMaterials[index].textureMapping.w =
                m_hTextures[diffuseTextureId].size.z;            // Depth
            m_hMaterials[index].textureMapping.z = TEXTURE_NONE; // Deprecated
            m_hMaterials[index].textureIds.x = diffuseTextureId;
            m_hMaterials[index].textureIds.y = normalTextureId;
            m_hMaterials[index].textureIds.z = bumpTextureId;
            m_hMaterials[index].textureIds.w = specularTextureId;
            m_hMaterials[index].textureOffset.x =
                m_hTextures[diffuseTextureId].offset; // Offset
            m_hMaterials[index].textureOffset.y =
                (normalTextureId == TEXTURE_NONE)
                    ? 0
                    : m_hTextures[normalTextureId].offset;
            m_hMaterials[index].textureOffset.z =
                (bumpTextureId == TEXTURE_NONE)
                    ? 0
                    : m_hTextures[bumpTextureId].offset;
            m_hMaterials[index].textureOffset.w =
                (specularTextureId == TEXTURE_NONE)
                    ? 0
                    : m_hTextures[specularTextureId].offset;
            // Advanced textures
            m_hMaterials[index].advancedTextureIds.x = reflectionTextureId;
            m_hMaterials[index].advancedTextureIds.y = transparentTextureId;
            m_hMaterials[index].advancedTextureIds.z =
                ambientOcclusionTextureId;
            m_hMaterials[index].advancedTextureOffset.x =
                (reflectionTextureId == TEXTURE_NONE)
                    ? 0
                    : m_hTextures[reflectionTextureId].offset;
            m_hMaterials[index].advancedTextureOffset.y =
                (transparentTextureId == TEXTURE_NONE)
                    ? 0
                    : m_hTextures[transparentTextureId].offset;
            m_hMaterials[index].advancedTextureOffset.z =
                (ambientOcclusionTextureId == TEXTURE_NONE)
                    ? 0
                    : m_hTextures[ambientOcclusionTextureId].offset;
        }
        else
        {
            // Computed textures (Mandelbrot, Julia, etc)
            m_hMaterials[index].textureMapping.x = 40000;
            m_hMaterials[index].textureMapping.y = 40000;
            m_hMaterials[index].textureMapping.z = TEXTURE_NONE; // Deprecated
            m_hMaterials[index].textureMapping.w = 3;
            m_hMaterials[index].textureIds.x = diffuseTextureId;
            m_hMaterials[index].textureIds.y = TEXTURE_NONE;
            m_hMaterials[index].textureIds.z = TEXTURE_NONE;
            m_hMaterials[index].textureIds.w = TEXTURE_NONE;
            m_hMaterials[index].textureOffset.x = 0;
            m_hMaterials[index].textureOffset.y = 0;
            m_hMaterials[index].textureOffset.z = 0;
            m_hMaterials[index].textureOffset.w = 0;
        }

        m_materialsTransfered = false;
    }
    else
    {
        LOG_ERROR("GPUKernel::setMaterial: Out of bounds("
                  << index << "/" << NB_MAX_MATERIALS << ")");
    }
}

void GPUKernel::setMaterialColor(unsigned int index, float r, float g, float b)
{
    LOG_INFO(3, "GPUKernel::setMaterialColor( " << index << ","
                                                << "color=(" << r << "," << g
                                                << "," << b << ")");

    if (index < NB_MAX_MATERIALS)
    {
        if (m_hMaterials[index].color.x != r ||
            m_hMaterials[index].color.y != g ||
            m_hMaterials[index].color.z != b)
        {
            m_hMaterials[index].color.x = r;
            m_hMaterials[index].color.y = g;
            m_hMaterials[index].color.z = b;
            m_materialsTransfered = false;
        }
    }
    else
    {
        LOG_ERROR("GPUKernel::setMaterial: Out of bounds("
                  << index << "/" << NB_MAX_MATERIALS << ")");
    }
}

void GPUKernel::setMaterialTextureId(unsigned int textureId)
{
    LOG_INFO(3, "GPUKernel::setMaterialTextureId( "
                    << m_currentMaterial << ","
                    << "Texture Id=" << textureId << ")");

    if (textureId != m_hMaterials[m_currentMaterial].textureIds.x)
    {
        m_hMaterials[m_currentMaterial].reflection = 0.3f;
        m_hMaterials[m_currentMaterial].refraction = 0.f;
        m_hMaterials[m_currentMaterial].transparency = 0.f;
        m_hMaterials[m_currentMaterial].opacity = 0.f;
        m_hMaterials[m_currentMaterial].textureMapping.x =
            m_hTextures[textureId].size.x;
        m_hMaterials[m_currentMaterial].textureMapping.y =
            m_hTextures[textureId].size.y;
        m_hMaterials[m_currentMaterial].textureMapping.z =
            TEXTURE_NONE; // Deprecated
        m_hMaterials[m_currentMaterial].textureMapping.w =
            m_hTextures[textureId].size.z;
        m_hMaterials[m_currentMaterial].textureIds.x = textureId;
        m_hMaterials[m_currentMaterial].textureIds.y = TEXTURE_NONE;
        m_hMaterials[m_currentMaterial].textureIds.z = TEXTURE_NONE;
        m_hMaterials[m_currentMaterial].textureIds.w = TEXTURE_NONE;
        m_materialsTransfered = false;
    }
}

int GPUKernel::getMaterialAttributes(
    int index, float &r, float &g, float &b, float &gloss, float &reflection,
    float &refraction, bool &procedural, bool &wireframe, int &wireframeDepth,
    float &transparency, float &opacity, int &diffuseTextureId,
    int &normalTextureId, int &bumpTextureId, int &specularTextureId,
    int &reflectionTextureId, int &transparencyTextureId,
    int &ambientOcclusionTextureId, float &specValue, float &specPower,
    float &specCoef, float &innerIllumination, float &illuminationDiffusion,
    float &illuminationPropagation, bool &fastTransparency)
{
    int returnValue = -1;

    if (index >= 0 && index <= m_nbActiveMaterials)
    {
        r = m_hMaterials[index].color.x;
        g = m_hMaterials[index].color.y;
        b = m_hMaterials[index].color.z;
        gloss = m_hMaterials[index].color.w;

        reflection = m_hMaterials[index].reflection;
        refraction = m_hMaterials[index].refraction;
        transparency = m_hMaterials[index].transparency;
        opacity = m_hMaterials[index].opacity;

        diffuseTextureId = m_hMaterials[index].textureIds.x;
        normalTextureId = m_hMaterials[index].textureIds.y;
        bumpTextureId = m_hMaterials[index].textureIds.z;
        specularTextureId = m_hMaterials[index].textureIds.w;
        reflectionTextureId = m_hMaterials[index].advancedTextureIds.x;
        transparencyTextureId = m_hMaterials[index].advancedTextureIds.y;
        ambientOcclusionTextureId = m_hMaterials[index].advancedTextureIds.z;

        specValue = m_hMaterials[index].specular.x;
        specPower = m_hMaterials[index].specular.y;
        specCoef = m_hMaterials[index].specular.w;
        innerIllumination = m_hMaterials[index].innerIllumination.x;
        illuminationDiffusion = m_hMaterials[index].innerIllumination.y;
        illuminationPropagation = m_hMaterials[index].innerIllumination.z;
        fastTransparency = (m_hMaterials[index].attributes.x == 1);
        procedural = (m_hMaterials[index].attributes.y == 1);
        wireframe = (m_hMaterials[index].attributes.z == 1);
        wireframeDepth = m_hMaterials[index].attributes.w;
        returnValue = 0;
    }
    else
    {
        LOG_ERROR("GPUKernel::setMaterial: Out of bounds("
                  << index << "/" << NB_MAX_MATERIALS << ")");
    }
    return returnValue;
}

Material *GPUKernel::getMaterial(const int index)
{
    Material *returnValue = NULL;

    if (index >= 0 && index <= static_cast<int>(m_nbActiveMaterials))
    {
        returnValue = &m_hMaterials[index];
    }
    return returnValue;
}

// ---------- Textures ----------
void GPUKernel::setTexture(const int index, const TextureInfo &textureInfo)
{
    LOG_INFO(1, "GPUKernel::setTexture(" << index << "/" << m_nbActiveTextures
                                         << ")");
    if (index >= m_nbActiveTextures)
        ++m_nbActiveTextures;
    if (m_hTextures[index].buffer != 0)
        delete[] m_hTextures[index].buffer;
    int size = textureInfo.size.x * textureInfo.size.y * textureInfo.size.z;
    m_hTextures[index].buffer = new BitmapBuffer[size];
    m_hTextures[index].offset = 0;
    m_hTextures[index].size.x = textureInfo.size.x;
    m_hTextures[index].size.y = textureInfo.size.y;
    m_hTextures[index].size.z = textureInfo.size.z;
    memcpy(m_hTextures[index].buffer, textureInfo.buffer, size);
    realignTexturesAndMaterials();
    m_texturesTransfered = false;
}

void GPUKernel::getTexture(const int index, TextureInfo &textureInfo)
{
    LOG_INFO(3, "GPUKernel::getTexture(" << index << ")");
    if (index <= m_nbActiveTextures)
    {
        textureInfo = m_hTextures[index];
    }
}

void GPUKernel::setSceneInfo(
    int width, int height, float transparentColor, int graphicsLevel,
    float viewDistance, float shadowIntensity, int nbRayIterations,
    vec4f backgroundColor, int cameraType, float eyeSeparation,
    bool renderBoxes, int pathTracingIteration, int maxPathTracingIterations,
    FrameBufferType frameBufferType, int timestamp, int atmosphericEffect,
    int skyboxSize, int skyboxMaterialId)
{
    LOG_INFO(3, "GPUKernel::setSceneInfo");
    memset(&m_sceneInfo, 0, sizeof(SceneInfo));
    m_sceneInfo.size.x = width;
    m_sceneInfo.size.y = height;
    m_sceneInfo.transparentColor = transparentColor;
    m_sceneInfo.graphicsLevel = static_cast<GraphicsLevel>(graphicsLevel);
    m_sceneInfo.viewDistance = viewDistance;
    m_sceneInfo.shadowIntensity = shadowIntensity;
    m_sceneInfo.nbRayIterations = nbRayIterations;
    m_sceneInfo.backgroundColor = backgroundColor;
    m_sceneInfo.eyeSeparation = eyeSeparation;
    m_sceneInfo.renderBoxes = renderBoxes;
    m_sceneInfo.pathTracingIteration = pathTracingIteration;
    m_sceneInfo.maxPathTracingIterations = maxPathTracingIterations;
    m_sceneInfo.frameBufferType = frameBufferType;
    m_sceneInfo.timestamp = timestamp;
    m_sceneInfo.atmosphericEffect =
        static_cast<AtmosphericEffect>(atmosphericEffect);
    m_sceneInfo.skyboxRadius = skyboxSize;
    m_sceneInfo.skyboxMaterialId = skyboxMaterialId;
}

void GPUKernel::setSceneInfo(const SceneInfo &sceneInfo)
{
    m_sceneInfo = sceneInfo;
}

SceneInfo &GPUKernel::getSceneInfo()
{
    return m_sceneInfo;
}

void GPUKernel::setPostProcessingInfo(PostProcessingType type, float param1,
                                      float param2, int param3)
{
    LOG_INFO(3, "GPUKernel::setPostProcessingInfo");
    m_postProcessingInfo.type = type;
    m_postProcessingInfo.param1 = param1;
    m_postProcessingInfo.param2 = param2;
    m_postProcessingInfo.param3 = param3;
}

void GPUKernel::setPostProcessingInfo(
    const PostProcessingInfo &postProcessingInfo)
{
    m_postProcessingInfo = postProcessingInfo;
}

void GPUKernel::loadFromFile(const std::string &filename)
{
    const vec4f center = make_vec4f();
    const float scale = 5000.f;
    FileMarshaller fm;
    fm.loadFromFile(*this, filename, center, scale);
}

void GPUKernel::saveToFile(const std::string &filename)
{
    FileMarshaller fm;
    fm.saveToFile(*this, filename);
}

// ---------- Kinect ----------
bool GPUKernel::loadTextureFromFile(const int index,
                                    const std::string &filename)
{
    LOG_INFO(3, "Loading texture from file " << filename << " into slot "
                                             << index << "/"
                                             << m_nbActiveTextures);
    bool result(false);

    if (filename.length() != 0)
    {
        m_texturesTransfered = false;
        ImageLoader imageLoader;

        if (filename.find(".bmp") != std::string::npos)
        {
            result = imageLoader.loadBMP24(index, filename, m_hTextures);
        }
        else if (filename.find(".jpg") != std::string::npos)
        {
            result = imageLoader.loadJPEG(index, filename, m_hTextures);
        }
        else if (filename.find(".tga") != std::string::npos)
        {
            result = imageLoader.loadTGA(index, filename, m_hTextures);
        }

        if (result)
        {
            m_textureFilenames[index] = filename;
            m_hTextures[index].type =
                tex_diffuse; // Default texture type is 'diffused'
            if (filename.find("b.") != std::string::npos)
                m_hTextures[index].type = tex_bump;
            if (filename.find("n.") != std::string::npos)
                m_hTextures[index].type = tex_normal;
            if (filename.find("a.") != std::string::npos)
                m_hTextures[index].type = tex_ambient_occlusion;
            if (filename.find("r.") != std::string::npos)
                m_hTextures[index].type = tex_reflective;
            if (filename.find("s.") != std::string::npos)
                m_hTextures[index].type = tex_specular;
            if (filename.find("t.") != std::string::npos)
                m_hTextures[index].type = tex_transparent;

            LOG_INFO(3, "Texture "
                            << index << "(" << filename
                            << ") loaded. Type=" << m_hTextures[index].type
                            << " size=" << m_hTextures[index].size.x << "x"
                            << m_hTextures[index].size.y << "x"
                            << m_hTextures[index].size.z);
            ++m_nbActiveTextures;
        }
        else
        {
            LOG_ERROR("Failed to load " << filename);
        }
    }
    return result;
}

void GPUKernel::reorganizeLights()
{
    LOG_INFO(1, "GPUKernel::reorganizeLights()");
    LOG_INFO(1, "Nb Primitives: " << m_boundingBoxes[m_frame][0].size())
    BoxContainer::iterator it = m_boundingBoxes[m_frame][0].begin();
    while (it != m_boundingBoxes[m_frame][0].end())
    {
        CPUBoundingBox &box = (*it).second;
        std::vector<long>::iterator itp = box.primitives.begin();
        while (itp != box.primitives.end())
        {
            Primitive &primitive = m_hPrimitives[*itp];
            if (primitive.materialId != MATERIAL_NONE)
            {
                Material &material = m_hMaterials[primitive.materialId];
                if (material.innerIllumination.x != 0.f)
                {
                    // Lights
                    bool found(false);
                    int i(0);
                    while (!found && i < m_nbActiveLamps[m_frame])
                    {
                        LOG_INFO(1, "[Box " << (*it).first << "] Lamp " << i
                                            << "/" << m_nbActiveLamps[m_frame]
                                            << " = " << m_hLamps[i]
                                            << ", Primitive index="
                                            << primitive.index);
                        if (m_hLamps[i] == primitive.index)
                        {
                            LOG_INFO(1, "Lamp " << i << " FOUND");
                            found = true;
                        }
                        ++i;
                    }

                    if (found)
                    {
                        LOG_INFO(3, "Add light information");
                        LightInformation lightInformation;
                        lightInformation.location.x = primitive.p0.x;
                        lightInformation.location.y = primitive.p0.y;
                        lightInformation.location.z = primitive.p0.z;
                        lightInformation.primitiveId = (*itp);
                        lightInformation.materialId = primitive.materialId;
                        lightInformation.color.x = material.color.x;
                        lightInformation.color.y = material.color.y;
                        lightInformation.color.z = material.color.z;
                        lightInformation.color.w = 0.f; // not used

                        LOG_INFO(
                            3, "Lamp "
                                   << m_lightInformation[m_lightInformationSize]
                                          .primitiveId
                                   << ","
                                   << m_lightInformation[m_lightInformationSize]
                                          .materialId
                                   << ":"
                                   << m_lightInformation[m_lightInformationSize]
                                          .location.x
                                   << ","
                                   << m_lightInformation[m_lightInformationSize]
                                          .location.y
                                   << ","
                                   << m_lightInformation[m_lightInformationSize]
                                          .location.z
                                   << " "
                                   << m_lightInformation[m_lightInformationSize]
                                          .color.x
                                   << ","
                                   << m_lightInformation[m_lightInformationSize]
                                          .color.y
                                   << ","
                                   << m_lightInformation[m_lightInformationSize]
                                          .color.z
                                   << " "
                                   << m_lightInformation[m_lightInformationSize]
                                          .color.w);

                        m_lightInformation[m_nbActiveLamps[m_frame] +
                                           m_lightInformationSize] =
                            lightInformation;
                        m_lightInformationSize++;
                    }
                }
            }
            ++itp;
        }
        ++it;
    }
    LOG_INFO(3, "Reorganized " << m_lightInformationSize << " Lights");
}

TextureInfo &GPUKernel::getTextureInformation(const int index)
{
    LOG_INFO(3, "Getting texture " << index << ": " << m_hTextures[index].size.x
                                   << "x" << m_hTextures[index].size.y << "x"
                                   << m_hTextures[index].size.z);
    return m_hTextures[index];
}

void GPUKernel::realignTexturesAndMaterials()
{
    LOG_INFO(3, "Realigning Textures And Materials");

    // Texture offsets
    processTextureOffsets();

    // Materials
    for (int i(0); i < m_nbActiveMaterials; ++i)
    {
        int diffuseTextureId = m_hMaterials[i].textureIds.x;
        int normalTextureId = m_hMaterials[i].textureIds.y;
        int bumpTextureId = m_hMaterials[i].textureIds.z;
        int specularTextureId = m_hMaterials[i].textureIds.w;
        int reflectionTextureId = m_hMaterials[i].advancedTextureIds.x;
        int transparencyTextureId = m_hMaterials[i].advancedTextureIds.y;

        if (diffuseTextureId != TEXTURE_NONE)
            LOG_INFO(3, "Material " << i << ": ids=" << diffuseTextureId);

        switch (diffuseTextureId)
        {
        case TEXTURE_MANDELBROT:
        case TEXTURE_JULIA:
            m_hMaterials[i].textureMapping.x = 40000;
            m_hMaterials[i].textureMapping.y = 40000;
            m_hMaterials[i].textureMapping.z = TEXTURE_NONE; // Deprecated
            m_hMaterials[i].textureMapping.w = 3;
            m_hMaterials[i].textureIds.x = diffuseTextureId;
            m_hMaterials[i].textureIds.y = TEXTURE_NONE;
            m_hMaterials[i].textureIds.z = TEXTURE_NONE;
            m_hMaterials[i].textureIds.w = TEXTURE_NONE;
            m_hMaterials[i].textureOffset.x = 0;
            m_hMaterials[i].textureOffset.y = 0;
            m_hMaterials[i].textureOffset.z = 0;
            m_hMaterials[i].textureOffset.w = 0;
            m_hMaterials[i].advancedTextureIds.x = TEXTURE_NONE;
            m_hMaterials[i].advancedTextureIds.y = TEXTURE_NONE;
            m_hMaterials[i].advancedTextureIds.z = TEXTURE_NONE;
            m_hMaterials[i].advancedTextureIds.w = TEXTURE_NONE;
            m_hMaterials[i].advancedTextureOffset.x = 0;
            m_hMaterials[i].advancedTextureOffset.y = 0;
            m_hMaterials[i].advancedTextureOffset.z = 0;
            m_hMaterials[i].advancedTextureOffset.w = 0;
            break;
        default:
            if (diffuseTextureId < m_nbActiveTextures)
            {
                m_hMaterials[i].textureMapping.x =
                    m_hTextures[diffuseTextureId].size.x;
                m_hMaterials[i].textureMapping.y =
                    m_hTextures[diffuseTextureId].size.y;
                m_hMaterials[i].textureMapping.z = TEXTURE_NONE; // Deprecated
                m_hMaterials[i].textureMapping.w =
                    m_hTextures[diffuseTextureId].size.z;
                m_hMaterials[i].textureIds.x = diffuseTextureId;
                m_hMaterials[i].textureIds.y = normalTextureId;
                m_hMaterials[i].textureIds.z = bumpTextureId;
                m_hMaterials[i].textureIds.w = specularTextureId;
                m_hMaterials[i].textureOffset.x =
                    (diffuseTextureId == TEXTURE_NONE)
                        ? 0
                        : m_hTextures[diffuseTextureId].offset;
                m_hMaterials[i].textureOffset.y =
                    (normalTextureId == TEXTURE_NONE)
                        ? 0
                        : m_hTextures[normalTextureId].offset;
                m_hMaterials[i].textureOffset.z =
                    (bumpTextureId == TEXTURE_NONE)
                        ? 0
                        : m_hTextures[bumpTextureId].offset;
                m_hMaterials[i].textureOffset.w =
                    (specularTextureId == TEXTURE_NONE)
                        ? 0
                        : m_hTextures[specularTextureId].offset;
                m_hMaterials[i].advancedTextureIds.x = reflectionTextureId;
                m_hMaterials[i].advancedTextureIds.y = transparencyTextureId;
                m_hMaterials[i].advancedTextureOffset.x =
                    (reflectionTextureId == TEXTURE_NONE)
                        ? 0
                        : m_hTextures[reflectionTextureId].offset;
                m_hMaterials[i].advancedTextureOffset.y =
                    (transparencyTextureId == TEXTURE_NONE)
                        ? 0
                        : m_hTextures[transparencyTextureId].offset;
                m_hMaterials[i].mappingOffset.x = 1.f;
                m_hMaterials[i].mappingOffset.y = 0.f;
            }
            else
            {
                m_hMaterials[i].textureMapping.x = 1;
                m_hMaterials[i].textureMapping.y = 1;
                m_hMaterials[i].textureMapping.z = TEXTURE_NONE; // Deprecated
                m_hMaterials[i].textureMapping.w = 1;
                m_hMaterials[i].textureIds.x = diffuseTextureId;
                m_hMaterials[i].textureIds.y = normalTextureId;
                m_hMaterials[i].textureIds.z = bumpTextureId;
                m_hMaterials[i].textureIds.w = specularTextureId;
                m_hMaterials[i].textureOffset.x = 0;
                m_hMaterials[i].textureOffset.y = 0;
                m_hMaterials[i].textureOffset.z = 0;
                m_hMaterials[i].textureOffset.w = 0;
                m_hMaterials[i].advancedTextureIds.x = reflectionTextureId;
                m_hMaterials[i].advancedTextureIds.y = transparencyTextureId;
                m_hMaterials[i].advancedTextureIds.z = TEXTURE_NONE;
                m_hMaterials[i].advancedTextureIds.w = TEXTURE_NONE;
                m_hMaterials[i].advancedTextureOffset.x = 0;
                m_hMaterials[i].advancedTextureOffset.y = 0;
                m_hMaterials[i].advancedTextureOffset.z = 0;
                m_hMaterials[i].advancedTextureOffset.w = 0;
                m_hMaterials[i].mappingOffset.x = 1.f;
                m_hMaterials[i].mappingOffset.y = 1.f;
            }
        }

        if (diffuseTextureId != TEXTURE_NONE)
        {
            LOG_INFO(3, "Material "
                            << i << ": " << m_hMaterials[i].textureMapping.x
                            << "x" << m_hMaterials[i].textureMapping.y << "x"
                            << m_hMaterials[i].textureMapping.w
                            << ", Wireframe: " << m_hMaterials[i].attributes.z
                            << ", diffuseTextureId [" << diffuseTextureId
                            << "] offset=" << m_hMaterials[i].textureOffset.x
                            << ", bumpTextureId [" << bumpTextureId
                            << "] offset=" << m_hMaterials[i].textureOffset.y);
        }
    }
}

void GPUKernel::buildLightInformationFromTexture(unsigned int index)
{
    LOG_INFO(3, "buildLightInformationFromTexture");
    m_lightInformationSize = 0;
    reorganizeLights();
    LOG_INFO(3, "Light Information Size = " << m_nbActiveLamps[m_frame] << "/"
                                            << m_lightInformationSize);
}

#ifdef USE_KINECT
void GPUKernel::initializeKinectTextures()
{
    LOG_INFO(3, "Initializing Kinect textures");
    m_hTextures[KINECT_COLOR_TEXTURE].offset = 0;
    m_hTextures[KINECT_COLOR_TEXTURE].size.x = KINECT_COLOR_WIDTH;
    m_hTextures[KINECT_COLOR_TEXTURE].size.y = KINECT_COLOR_HEIGHT;
    m_hTextures[KINECT_COLOR_TEXTURE].size.z = KINECT_COLOR_DEPTH;

    m_hTextures[KINECT_DEPTH_TEXTURE].offset = KINECT_COLOR_SIZE;
    m_hTextures[KINECT_DEPTH_TEXTURE].size.x = KINECT_DEPTH_WIDTH;
    m_hTextures[KINECT_DEPTH_TEXTURE].size.y = KINECT_DEPTH_HEIGHT;
    m_hTextures[KINECT_DEPTH_TEXTURE].size.z = KINECT_DEPTH_DEPTH;

    m_nbActiveTextures = 2;
}

int GPUKernel::updateSkeletons(unsigned int primitiveIndex,
                               vec3f skeletonPosition, float size, float radius,
                               int materialId, float head_radius,
                               int head_materialId, float hands_radius,
                               int hands_materialId, float feet_radius,
                               int feet_materialId)
{
    m_skeletonIndex = -1;
    HRESULT hr = NuiSkeletonGetNextFrame(0, &m_skeletonFrame);
    bool found = false;
    if (hr == S_OK)
    {
        int i = 0;
        while (i < NUI_SKELETON_COUNT && !found)
        {
            if (m_skeletonFrame.SkeletonData[i].eTrackingState ==
                NUI_SKELETON_TRACKED)
            {
                m_skeletonIndex = i;
                found = true; //(m_skeletonIndex==0);
                for (int j = 0; j < 20; j++)
                {
                    float r = radius;
                    int m = materialId;
                    switch (j)
                    {
                    case NUI_SKELETON_POSITION_FOOT_LEFT:
                    case NUI_SKELETON_POSITION_FOOT_RIGHT:
                        r = feet_radius;
                        m = feet_materialId;
                        break;
                    case NUI_SKELETON_POSITION_HAND_LEFT:
                    case NUI_SKELETON_POSITION_HAND_RIGHT:
                        r = hands_radius;
                        m = hands_materialId;
                        break;
                    case NUI_SKELETON_POSITION_HEAD:
                        r = head_radius;
                        m = head_materialId;
                        break;
                    }
                    setPrimitive(
                        primitiveIndex + j,
                        static_cast<float>(m_skeletonFrame.SkeletonData[i].SkeletonPositions[j].x * size +
                                           skeletonPosition.x),
                        static_cast<float>(m_skeletonFrame.SkeletonData[i].SkeletonPositions[j].y * size +
                                           skeletonPosition.y + size),
                        static_cast<float>(
                            0.f /*skeletonPosition.z - 2.f*m_skeletonFrame.SkeletonData[i].SkeletonPositions[j].z * size*/),
                        static_cast<float>(r), 0.f, 0.f, m);
                }
            }
            i++;
        }
    }
    return found ? S_OK : S_FALSE;
}

bool GPUKernel::getSkeletonPosition(int index, vec3f &position)
{
    bool returnValue(false);
    if (m_skeletonIndex != -1)
    {
        position.x = m_skeletonFrame.SkeletonData[m_skeletonIndex]
                         .SkeletonPositions[index]
                         .x;
        position.y =
            0.f; // m_skeletonFrame.SkeletonData[m_skeletonIndex].SkeletonPositions[index].y;
        position.z = m_skeletonFrame.SkeletonData[m_skeletonIndex]
                         .SkeletonPositions[index]
                         .z;
        returnValue = true;
    }
    return returnValue;
}

#endif // USE_KINECT

void GPUKernel::saveBitmapToFile(const std::string &filename,
                                 BitmapBuffer *bitmap, const int width,
                                 const int height, const int depth)
{
    FILE *f;

    unsigned char bmpfileheader[14] = {'B', 'M', 0, 0,  0, 0, 0,
                                       0,   0,   0, 54, 0, 0, 0};
    unsigned char bmpinfoheader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
                                       0,  0, 0, 0, 1, 0, 32, 0};

    int w = width;
    int h = height;
    int filesize = 54 + depth * w * h;

    bmpfileheader[2] = (unsigned char)(filesize);
    bmpfileheader[3] = (unsigned char)(filesize >> 8);
    bmpfileheader[4] = (unsigned char)(filesize >> 16);
    bmpfileheader[5] = (unsigned char)(filesize >> 24);

    bmpinfoheader[4] = (unsigned char)(w);
    bmpinfoheader[5] = (unsigned char)(w >> 8);
    bmpinfoheader[6] = (unsigned char)(w >> 16);
    bmpinfoheader[7] = (unsigned char)(w >> 24);

    bmpinfoheader[8] = (unsigned char)(h);
    bmpinfoheader[9] = (unsigned char)(h >> 8);
    bmpinfoheader[10] = (unsigned char)(h >> 16);
    bmpinfoheader[11] = (unsigned char)(h >> 24);

#ifdef WIN32
    fopen_s(&f, filename.c_str(), "wb");
#else
    f = fopen(filename.c_str(), "wb");
#endif
    fwrite(bmpfileheader, 1, 14, f);
    fwrite(bmpinfoheader, 1, 40, f);
    fwrite(bitmap, 1, width * height * depth, f);
    fclose(f);
};

int GPUKernel::getLight(int index)
{
    if (index <= m_nbActiveLamps[m_frame])
    {
        LOG_INFO(3, "getLight(" << index << ")=" << m_hLamps[index]);
        return m_hLamps[index];
    }
    else
    {
        LOG_ERROR("getLight(" << index << "/" << m_nbActiveLamps[m_frame]
                              << ") is out of bounds");
    }
    return -1;
}

// OpenGL
int GPUKernel::setGLMode(const int &glMode)
{
    int p = -1;
    if (glMode == -1)
    {
        for (int i(0); i < m_vertices.size(); ++i)
        {
            m_vertices[i].x += m_translation.x;
            m_vertices[i].y += m_translation.y;
            m_vertices[i].z += m_translation.z;
        }

        switch (m_GLMode)
        {
        case GL_POINTS:
        {
            for (int i(0); i < m_vertices.size(); ++i)
            {
                p = addPrimitive(ptSphere);
                setPrimitive(p, m_vertices[i].x, m_vertices[i].y,
                             m_vertices[i].z, m_pointSize, 0.f, 0.f,
                             m_currentMaterial);
            }
            LOG_INFO(3, "[OpenGL] Added " << m_vertices.size() << " Points");
        }
        break;
        case GL_LINES:
        {
            int nbLines = static_cast<int>(m_vertices.size() / 2);
            for (int i(0); i < nbLines; ++i)
            {
                int index = i * 2;
                if (index + 1 <= m_vertices.size())
                {
                    p = addPrimitive(ptCylinder);
                    setPrimitive(p, m_vertices[i].x, m_vertices[i].y,
                                 m_vertices[i].z, m_vertices[i + 1].x,
                                 m_vertices[i + 1].y, m_vertices[i + 1].z,
                                 m_pointSize, 0.f, 0.f, m_currentMaterial);
                }
            }
            LOG_INFO(3, "[OpenGL] Added " << nbLines << " Lines");
        }
        break;
        case GL_TRIANGLES:
        {
            // Vertices
            int nbTriangles = static_cast<int>(m_vertices.size() / 3);
            for (int i(0); i < nbTriangles; ++i)
            {
                int index = i * 3;
                if (index + 2 <= m_vertices.size())
                {
                    p = addPrimitive(ptTriangle);
                    setPrimitive(p, m_vertices[index].x, m_vertices[index].y,
                                 m_vertices[index].z, m_vertices[index + 1].x,
                                 m_vertices[index + 1].y,
                                 m_vertices[index + 1].z,
                                 m_vertices[index + 2].x,
                                 m_vertices[index + 2].y,
                                 m_vertices[index + 2].z, 0.f, 0.f, 0.f,
                                 m_currentMaterial);
                }

                if (index + 2 <= m_textCoords.size())
                    setPrimitiveTextureCoordinates(p, m_textCoords[index],
                                                   m_textCoords[index + 1],
                                                   m_textCoords[index + 2]);
                if (index + 2 <= m_normals.size())
                {
                    setPrimitiveNormals(p, m_normals[index],
                                        m_normals[index + 1],
                                        m_normals[index + 2]);
                }
            }
            LOG_INFO(3, "[OpenGL] Added " << nbTriangles
                                          << " triangles with material ID "
                                          << m_currentMaterial);
        }
        break;
        case GL_QUADS:
        {
            // Vertices
            int nbQuads = static_cast<int>(m_vertices.size() / 4);
            for (int i(0); i < nbQuads; ++i)
            {
                int p1, p2;
                int index = i * 4;
                if (index + 2 <= m_vertices.size())
                {
                    p1 = addPrimitive(ptTriangle);
                    setPrimitive(p1, m_vertices[index].x, m_vertices[index].y,
                                 m_vertices[index].z, m_vertices[index + 1].x,
                                 m_vertices[index + 1].y,
                                 m_vertices[index + 1].z,
                                 m_vertices[index + 2].x,
                                 m_vertices[index + 2].y,
                                 m_vertices[index + 2].z, 0.f, 0.f, 0.f,
                                 m_currentMaterial);

                    p2 = addPrimitive(ptTriangle);
                    setPrimitive(
                        p2, m_vertices[index + 1].x, m_vertices[index + 1].y,
                        m_vertices[index + 1].z, m_vertices[index + 3].x,
                        m_vertices[index + 3].y, m_vertices[index + 3].z,
                        m_vertices[index + 0].x, m_vertices[index + 0].y,
                        m_vertices[index + 0].z, 0.f, 0.f, 0.f,
                        m_currentMaterial);
                }

                if (index + 3 <= m_textCoords.size())
                {
                    setPrimitiveTextureCoordinates(p1, m_textCoords[index],
                                                   m_textCoords[index + 1],
                                                   m_textCoords[index + 2]);
                    setPrimitiveTextureCoordinates(p2, m_textCoords[index + 2],
                                                   m_textCoords[index + 3],
                                                   m_textCoords[index]);
                }
                if (index + 3 <= m_normals.size())
                {
                    setPrimitiveNormals(p1, m_normals[index],
                                        m_normals[index + 1],
                                        m_normals[index + 2]);
                    setPrimitiveNormals(p2, m_normals[index + 2],
                                        m_normals[index + 3], m_normals[index]);
                }
            }
            LOG_INFO(3, "[OpenGL] " << nbQuads
                                    << " quads created with material ID "
                                    << m_currentMaterial);
        }
        break;
        default:
        {
            LOG_INFO(3, "[OpenGL] Mode " << m_GLMode << " not supported");
        }
        break;
        }
        m_vertices.clear();
        m_normals.clear();
        m_textCoords.clear();

        memset(&m_translation, 0, sizeof(vec4f));
        // memset(&m_rotation,0,sizeof(vec4f));
        m_pointSize = 1.f;
    }
    m_GLMode = glMode;
    return p;
}

void GPUKernel::addVertex(float x, float y, float z)
{
    m_vertices.push_back(make_vec3f(x, y, z));
}

void GPUKernel::addNormal(float x, float y, float z)
{
    m_normals.push_back(make_vec3f(x, y, z));
}

void GPUKernel::addTextureCoordinates(float x, float y)
{
    m_textCoords.push_back(make_vec2f(x, y));
}

void GPUKernel::translate(float x, float y, float z)
{
    m_translation.x += x;
    m_translation.y += y;
    m_translation.z += z;
}

void GPUKernel::rotate(float x, float y, float z)
{
    m_rotation.x += x;
    m_rotation.y += y;
    m_rotation.z += z;
}

int GPUKernel::getCurrentMaterial()
{
    return m_currentMaterial;
}

void GPUKernel::setCurrentMaterial(const int currentMaterial)
{
    m_currentMaterial = currentMaterial;
}

unsigned int GPUKernel::getNbActiveBoxes()
{
    return static_cast<unsigned int>(m_boundingBoxes[m_frame][0].size());
}

unsigned int GPUKernel::getNbActivePrimitives()
{
    return static_cast<unsigned int>(m_primitives[m_frame].size());
}

unsigned int GPUKernel::getNbActiveLamps()
{
    return m_nbActiveLamps[m_frame];
}

unsigned int GPUKernel::getNbActiveMaterials()
{
    return m_nbActiveMaterials;
}

unsigned int GPUKernel::getNbActiveTextures()
{
    return m_nbActiveTextures;
}

std::string GPUKernel::getTextureFilename(const int index)
{
    return m_textureFilenames[index];
}

void GPUKernel::processTextureOffsets()
{
    // Reprocess offset
    int totalSize = 0;
    for (int i(0); i < NB_MAX_TEXTURES; ++i)
    {
        if (m_hTextures[i].buffer != 0)
        {
            m_hTextures[i].offset = totalSize;
            totalSize += m_hTextures[i].size.x * m_hTextures[i].size.y *
                         m_hTextures[i].size.z;
        }
        else
            m_hTextures[i].offset = 0;
    }
}

void GPUKernel::setPointSize(const float pointSize)
{
    m_pointSize = pointSize;
}

void GPUKernel::render_begin(const float timer)
{
    LOG_INFO(3, "GPUKernel::render_begin");
    LOG_INFO(3,
             "Scene size: " << m_sceneInfo.size.x << "x" << m_sceneInfo.size.y);

    // Randoms
    const size_t size = m_sceneInfo.size.x * m_sceneInfo.size.y;
    m_sceneInfo.timestamp = rand() % 10000;
    if (!m_randomsTransfered || m_sceneInfo.pathTracingIteration % 50 == 1)
    {
        //     m_randomsTransfered = false;
#if USE_RANDOM_DEVICE
        srand(static_cast<int>(time(0)));
#pragma omp parallel for
        for (int i = 0; i < size; ++i)
            m_hRandoms[i] = 0.0000005f * (rand() % 20000 - 10000);
#else
        std::random_device device;
        std::mt19937 generator(device());
        std::uniform_int_distribution<int> distribution(0, 20000);
        size_t i;
#pragma omp parallel for
        for (i = 0; i < size; ++i)
            m_hRandoms[i] = 0.0000001f * (distribution(generator) - 10000);
#endif
    }

#ifdef USE_OCULUS
    if (m_oculus && m_sensorFusion && m_sensorFusion->IsAttachedToSensor())
    {
        OVR::Quatf orientation = m_sensorFusion->GetOrientation();
        m_viewPos.y = 0.f;
        m_viewDir.y = m_viewPos.y;
        m_angles.x = -PI * orientation.x;
        m_angles.y = -PI * orientation.y;
        m_angles.z = PI * orientation.z;
        m_sceneInfo.pathTracingIteration = 0;
        m_sceneInfo.cameraType = ctVR;
    }
#endif // USE_OCULUS
}

void GPUKernel::setNbFrames(const int nbFrames)
{
    m_nbFrames = nbFrames;
}

void GPUKernel::setFrame(const int frame)
{
    m_frame = frame;
}

int GPUKernel::getNbFrames()
{
    return m_nbFrames;
}

int GPUKernel::getFrame()
{
    return m_frame;
}

void GPUKernel::nextFrame()
{
    m_frame++;
    if (m_frame >= m_nbFrames)
        m_frame = m_nbFrames - 1;
}

void GPUKernel::previousFrame()
{
    if (m_frame > 0)
        m_frame--;
}

void GPUKernel::switchOculusVR()
{
    m_oculus = !m_oculus;
    if (m_oculus)
    {
        m_viewPos.z = -2360;
        m_viewDir.z = m_viewPos.z + 2810.f;
    }
    else
    {
        m_viewPos.z = -10000.f;
        m_viewDir.z = 0.f;
    }
}

void GPUKernel::generateScreenshot(const std::string &filename,
                                   const unsigned int width,
                                   const unsigned int height,
                                   const unsigned int quality)
{
    LOG_INFO(1, "Generating screenshot " << filename << " (Quality=" << quality
                                         << ", Size=" << width << "x" << height
                                         << ")");
    SceneInfo sceneInfo = m_sceneInfo;
    SceneInfo bakSceneInfo = m_sceneInfo;
    sceneInfo.size.x = width;
    sceneInfo.size.y = height;
    sceneInfo.maxPathTracingIterations = quality;
    for (unsigned int i = 0; i < quality; ++i)
    {
#ifdef WIN32
        long t = GetTickCount();
#endif
        sceneInfo.pathTracingIteration = i;
        m_sceneInfo = sceneInfo;
        render_begin(0);
        render_end();
        LOG_INFO(1, "Frame " << i << " rendered!");
#ifdef WIN32
        int avg = GetTickCount() - t;
        int left = static_cast<int>(static_cast<float>(quality - i) *
                                    static_cast<float>(avg) / 1000.f);
        LOG_INFO(1, "Frame " << i << " generated in " << avg << "ms (" << left
                             << " seconds left...)");
#endif
        LOG_INFO(1, "Saving bitmap to disk");
        size_t size = sceneInfo.size.x * sceneInfo.size.y * gColorDepth;
        switch (sceneInfo.frameBufferType)
        {
        case ftRGB:
        {
            BitmapBuffer *dst = new BitmapBuffer[size];
            for (int i(0); i < size; i += gColorDepth)
            {
                dst[i] = m_bitmap[size - i];
                dst[i + 1] = m_bitmap[size - i + 1];
                dst[i + 2] = m_bitmap[size - i + 2];
            }
            jpge::compress_image_to_jpeg_file(filename.c_str(),
                                              sceneInfo.size.x,
                                              sceneInfo.size.y, gColorDepth,
                                              dst);
            delete[] dst;
            break;
        }
        default:
        {
            BitmapBuffer *dst = new BitmapBuffer[size];
            for (int i(0); i < size; i += gColorDepth)
            {
                dst[i] = m_bitmap[size - i + 2];
                dst[i + 1] = m_bitmap[size - i + 1];
                dst[i + 2] = m_bitmap[size - i];
            }
            jpge::compress_image_to_jpeg_file(filename.c_str(),
                                              sceneInfo.size.x,
                                              sceneInfo.size.y, gColorDepth,
                                              dst);
            delete[] dst;
            break;
        }
        break;
        }
    }
    m_sceneInfo = bakSceneInfo;
    LOG_INFO(1, "Screenshot successfully generated!");
}

#ifdef USE_OCULUS
void GPUKernel::initializeOVR()
{
    LOG_INFO(3, "----------------------------");
    LOG_INFO(3, "                       _____");
    LOG_INFO(3, "                       [O_O]");
    LOG_INFO(3, "                            ");
    LOG_INFO(3, "Oculus initialization       ");

    m_oculus = false;

    OVR::HMDInfo Info;
    bool InfoLoaded;

    OVR::System::Init();

    m_manager = *OVR::DeviceManager::Create();

    m_HMD = *m_manager->EnumerateDevices<OVR::HMDDevice>().CreateDevice();

    if (m_HMD)
    {
        InfoLoaded = m_HMD->GetDeviceInfo(&Info);
        m_sensor = *m_HMD->GetSensor();
    }
    else
    {
        m_sensor =
            *m_manager->EnumerateDevices<OVR::SensorDevice>().CreateDevice();
    }

    if (m_sensorFusion == 0)
    {
        m_sensorFusion = new OVR::SensorFusion();
    }

    if (m_sensor && m_sensorFusion != 0)
    {
        m_sensorFusion->AttachToSensor(m_sensor);
        m_sensorFusion->Reset();
    }
    else
    {
        LOG_ERROR("    FAILED");
    }
    LOG_INFO(3, "----------------------------");
}

void GPUKernel::finializeOVR()
{
    if (m_sensorFusion)
        delete m_sensorFusion;
    m_sensor.Clear();
    m_HMD.Clear();
    m_manager.Clear();
    OVR::System::Destroy();
}
#endif // USE_OCULUS
} // namespace solr
