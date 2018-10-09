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

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

#include <solr.h>
#include <Logging.h>
#include <engines/opencl/OpenCLKernel.h>

#ifdef WIN32
#include <windows.h>
#else
#include <math.h>
#include <sstream>
#endif
#include <fstream>

#define __CL_ENABLE_EXCEPTIONS

namespace solr
{
void getKernelCode(std::string &str)
{
    str = "";
}

const long MAX_SOURCE_SIZE = 3 * 65535;

// Platforms
cl_uint OpenCLKernel::m_numberOfPlatforms;
std::string OpenCLKernel::m_platformsDescription[MAX_PLATFORMS];

// Devices
cl_device_id OpenCLKernel::m_devices[MAX_PLATFORMS][MAX_DEVICES];
cl_uint OpenCLKernel::m_numberOfDevices[MAX_PLATFORMS];
std::string OpenCLKernel::m_devicesDescription[MAX_PLATFORMS][MAX_DEVICES];

/*
 * getErrorDesc
 */
std::string getErrorDesc(int err)
{
    switch (err)
    {
    case CL_SUCCESS:
        return "CL_SUCCESS";
    case CL_DEVICE_NOT_FOUND:
        return "CL_DEVICE_NOT_FOUND";
    case CL_COMPILER_NOT_AVAILABLE:
        return "CL_COMPILER_NOT_AVAILABLE";
    case CL_MEM_OBJECT_ALLOCATION_FAILURE:
        return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
    case CL_OUT_OF_RESOURCES:
        return "CL_OUT_OF_RESOURCES";
    case CL_OUT_OF_HOST_MEMORY:
        return "CL_OUT_OF_HOST_MEMORY";
    case CL_PROFILING_INFO_NOT_AVAILABLE:
        return "CL_PROFILING_INFO_NOT_AVAILABLE";
    case CL_MEM_COPY_OVERLAP:
        return "CL_MEM_COPY_OVERLAP";
    case CL_IMAGE_FORMAT_MISMATCH:
        return "CL_IMAGE_FORMAT_MISMATCH";
    case CL_IMAGE_FORMAT_NOT_SUPPORTED:
        return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
    case CL_BUILD_PROGRAM_FAILURE:
        return "CL_BUILD_PROGRAM_FAILURE";
    case CL_MAP_FAILURE:
        return "CL_MAP_FAILURE";

    case CL_INVALID_VALUE:
        return "CL_INVALID_VALUE";
    case CL_INVALID_DEVICE_TYPE:
        return "CL_INVALID_DEVICE_TYPE";
    case CL_INVALID_PLATFORM:
        return "CL_INVALID_PLATFORM";
    case CL_INVALID_DEVICE:
        return "CL_INVALID_DEVICE";
    case CL_INVALID_CONTEXT:
        return "CL_INVALID_CONTEXT";
    case CL_INVALID_QUEUE_PROPERTIES:
        return "CL_INVALID_QUEUE_PROPERTIES";
    case CL_INVALID_COMMAND_QUEUE:
        return "CL_INVALID_COMMAND_QUEUE";
    case CL_INVALID_HOST_PTR:
        return "CL_INVALID_HOST_PTR";
    case CL_INVALID_MEM_OBJECT:
        return "CL_INVALID_MEM_OBJECT";
    case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR:
        return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
    case CL_INVALID_IMAGE_SIZE:
        return "CL_INVALID_IMAGE_SIZE";
    case CL_INVALID_SAMPLER:
        return "CL_INVALID_SAMPLER";
    case CL_INVALID_BINARY:
        return "CL_INVALID_BINARY";
    case CL_INVALID_BUILD_OPTIONS:
        return "CL_INVALID_BUILD_OPTIONS";
    case CL_INVALID_PROGRAM:
        return "CL_INVALID_PROGRAM";
    case CL_INVALID_PROGRAM_EXECUTABLE:
        return "CL_INVALID_PROGRAM_EXECUTABLE";
    case CL_INVALID_KERNEL_NAME:
        return "CL_INVALID_KERNEL_NAME";
    case CL_INVALID_KERNEL_DEFINITION:
        return "CL_INVALID_KERNEL_DEFINITION";
    case CL_INVALID_KERNEL:
        return "CL_INVALID_KERNEL";
    case CL_INVALID_ARG_INDEX:
        return "CL_INVALID_ARG_INDEX";
    case CL_INVALID_ARG_VALUE:
        return "CL_INVALID_ARG_VALUE";
    case CL_INVALID_ARG_SIZE:
        return "CL_INVALID_ARG_SIZE";
    case CL_INVALID_KERNEL_ARGS:
        return "CL_INVALID_KERNEL_ARGS";
    case CL_INVALID_WORK_DIMENSION:
        return "CL_INVALID_WORK_DIMENSION";
    case CL_INVALID_WORK_GROUP_SIZE:
        return "CL_INVALID_WORK_GROUP_SIZE";
    case CL_INVALID_WORK_ITEM_SIZE:
        return "CL_INVALID_WORK_ITEM_SIZE";
    case CL_INVALID_GLOBAL_OFFSET:
        return "CL_INVALID_GLOBAL_OFFSET";
    case CL_INVALID_EVENT_WAIT_LIST:
        return "CL_INVALID_EVENT_WAIT_LIST";
    case CL_INVALID_OPERATION:
        return "CL_INVALID_OPERATION";
    case CL_INVALID_GL_OBJECT:
        return "CL_INVALID_GL_OBJECT";
    case CL_INVALID_BUFFER_SIZE:
        return "CL_INVALID_BUFFER_SIZE";
    case CL_INVALID_MIP_LEVEL:
        return "CL_INVALID_MIP_LEVEL";
    default:
        return "UNKNOWN";
    }
}

/*
 * Callback function for OpenCL notifications
 */
void buildNotify(cl_program, void *user_data)
{
    LOG_ERROR("OpenCL Error (via buildNotify): " << user_data);
}
void contextNotify(const char *errinfo, const void *private_info, size_t cb, void *user_data)
{
    LOG_ERROR("OpenCL Error (via contextNotify): " << errinfo);
}

/*
 * CHECKSTATUS
 */

int __status = CL_SUCCESS;

/*LOG_INFO(1,"[] " #stmt " []"); \*/
#define CHECKSTATUS(stmt)                                                                         \
    {                                                                                             \
        __status = stmt;                                                                          \
        if (__status != CL_SUCCESS)                                                               \
        {                                                                                         \
            LOG_ERROR("Status=" << __status << " (" << getErrorDesc(__status) << ") for " #stmt); \
            exit(1);                                                                              \
        }                                                                                         \
    }

/*
 * OpenCLKernel constructor
 */
OpenCLKernel::OpenCLKernel()
    : GPUKernel()
    , m_platform(0)
    , m_device(0)
    , m_kernelFilename(DEFAULT_KERNEL_FILENAME)
    , m_hContext(0)
    , m_hQueue(0)
    , m_hProgram(0)
    , m_kAlignment(0)
    , m_kStandardRenderer(0)
    , m_kAnaglyphRenderer(0)
    , m_k3DVisionRenderer(0)
    , m_kFishEyeRenderer(0)
    , m_kVolumeRenderer(0)
    , m_kDefault(0)
    , m_kDepthOfField(0)
    , m_kAmbientOcclusion(0)
    , m_kRadiosity(0)
    , m_kFilter(0)
    , _dPrimitives(0)
    , m_dLamps(0)
    , m_dLightInformation(0)
    , m_dTextures(0)
    , m_dRandoms(0)
    , m_dBitmap(0)
    , m_dPostProcessingBuffer(0)
    , m_dPrimitivesXYIds(0)
{
    // TODO: Occupancy parameters
    m_occupancyParameters.x = 1;
    m_occupancyParameters.y = 1;

#ifdef LOGGING
    // Initialize Log
    LOG_INITIALIZE_ETW(&GPU_OPENCLRAYTRACERMODULE, &GPU_OPENCLRAYTRACERMODULE_EVENT_DEBUG,
                       &GPU_OPENCLRAYTRACERMODULE_EVENT_VERBOSE, &GPU_OPENCLRAYTRACERMODULE_EVENT_INFO,
                       &GPU_OPENCLRAYTRACERMODULE_EVENT_WARNING, &GPU_OPENCLRAYTRACERMODULE_EVENT_ERROR);
#endif // NDEBUG

#if USE_KINECT
    // Initialize Kinect
    int status = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON |
                               NUI_INITIALIZE_FLAG_USES_COLOR);

    m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_hNextVideoFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_hNextSkeletonEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    m_skeletons = CreateEvent(NULL, TRUE, FALSE, NULL);
    status = NuiSkeletonTrackingEnable(m_skeletons, 0);

    status = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, m_hNextVideoFrameEvent,
                                &m_pVideoStreamHandle);
    status = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2,
                                m_hNextDepthFrameEvent, &m_pDepthStreamHandle);

    status = NuiCameraElevationSetAngle(0);
#endif // USE_KINECT

    // Eye position
    m_viewPos.x = 0.0f;
    m_viewPos.y = 0.0f;
    m_viewPos.z = -40.0f;

    // Rotation angles
    m_angles.x = 0.0f;
    m_angles.y = 0.0f;
    m_angles.z = 0.0f;
}

/*
 * compileKernels
 */
void OpenCLKernel::recompileKernels()
{
    releaseKernels();

    int status(0);
    try
    {
        std::string kernelCode;
        std::string line;
        std::ifstream inputFile(m_kernelFilename.c_str());
        if (inputFile.is_open())
        {
            LOG_INFO(1, "Recompiling kernel from " << m_kernelFilename);
            while (getline(inputFile, line))
                kernelCode += line;
            inputFile.close();
        }
        else
        {
            LOG_ERROR("Could not open file: " << m_kernelFilename);
            exit(1);
        }

        LOG_INFO(3, "Create Program from source");
        const char *kernel_code = kernelCode.c_str();
        const size_t len = kernelCode.length();
        m_hProgram =
            clCreateProgramWithSource(m_hContext, 1, (const char **)&kernel_code, (const size_t *)&len, &status);
        CHECKSTATUS(status);

        std::string compilationOptions =
            "-cl-no-signed-zeros -cl-fast-relaxed-math -cl-unsafe-math-optimizations "
            "-cl-mad-enable";
#ifdef WIN32
        compilationOptions += " -DMSVC";
#endif
#ifdef DEBUG
        compilationOptions += " -DDEBUG -g";
#endif
#ifdef USE_KINECT
        compilationOptions += " -DUSE_KINECT";
#endif
        LOG_INFO(1, "Building Program with " << compilationOptions);
        CHECKSTATUS(clBuildProgram(m_hProgram, 1, &m_devices[m_platform][m_device], compilationOptions.c_str(),
                                   &buildNotify, NULL));
        size_t logSize;
        CHECKSTATUS(clGetProgramBuildInfo(m_hProgram, m_hDeviceId, CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize));
        char *log = new char[logSize];
        CHECKSTATUS(clGetProgramBuildInfo(m_hProgram, m_hDeviceId, CL_PROGRAM_BUILD_LOG, logSize, log, NULL));
        if (logSize > 1)
            LOG_INFO(1, log);
        delete[] log;
#if DEBUG
        // Text kernels
        m_kAlignment = clCreateKernel(m_hProgram, "k_alignment", &status);
        CHECKSTATUS(status);
        size_t szLocalWorkSize[] = {1, 1};
        size_t szGlobalWorkSize[] = {1, 1};
        CHECKSTATUS(
            clEnqueueNDRangeKernel(m_hQueue, m_kAlignment, 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, 0, 0));
        LOG_INFO(3, "--== C++ Data structures ==--");
        LOG_INFO(3, "Ray               : " << sizeof(Ray));
        LOG_INFO(3, "SceneInfo         : " << sizeof(SceneInfo));
        LOG_INFO(3, "PostProcessingInfo: " << sizeof(PostProcessingInfo));
        LOG_INFO(3, "TextureInfo       : " << sizeof(TextureInfo));
        LOG_INFO(3, "Primitive         : " << sizeof(Primitive));
        LOG_INFO(3, "BoundingBox       : " << sizeof(BoundingBox));
        LOG_INFO(3, "Material          : " << sizeof(Material));
        LOG_INFO(3, "LightInformation  : " << sizeof(LightInformation));
        LOG_INFO(3, "------------------------------");
#endif

        // Rendering kernels
        m_kStandardRenderer = clCreateKernel(m_hProgram, "k_standardRenderer", &status);
        CHECKSTATUS(status);

        m_kAnaglyphRenderer = clCreateKernel(m_hProgram, "k_anaglyphRenderer", &status);
        CHECKSTATUS(status);

        m_k3DVisionRenderer = clCreateKernel(m_hProgram, "k_3DVisionRenderer", &status);
        CHECKSTATUS(status);

        m_kFishEyeRenderer = clCreateKernel(m_hProgram, "k_fishEyeRenderer", &status);
        CHECKSTATUS(status);

        m_kVolumeRenderer = clCreateKernel(m_hProgram, "k_volumeRenderer", &status);
        CHECKSTATUS(status);

        LOG_INFO(1, "Rendering kernels created");

        // Post-processing kernels
        m_kDefault = clCreateKernel(m_hProgram, "k_default", &status);
        CHECKSTATUS(status);

        m_kDepthOfField = clCreateKernel(m_hProgram, "k_depthOfField", &status);
        CHECKSTATUS(status);

        m_kAmbientOcclusion = clCreateKernel(m_hProgram, "k_ambientOcclusion", &status);
        CHECKSTATUS(status);

        m_kRadiosity = clCreateKernel(m_hProgram, "k_radiosity", &status);
        CHECKSTATUS(status);

        m_kFilter = clCreateKernel(m_hProgram, "k_filter", &status);
        CHECKSTATUS(status);
        LOG_INFO(1, "Post-processing kernels created");
    }
    catch (...)
    {
        LOG_ERROR("Unexpected exception");
    }
}

void OpenCLKernel::initializeDevice()
{
    int status(0);

    // Populate OpenCL Information
    queryDevice();

    // initialize OpenCL device
    LOG_INFO(1, "Initializing OpenCL context on platform: " << m_platform << ", device: " << m_hDeviceId);
    m_hContext =
        clCreateContext(NULL, m_numberOfDevices[m_platform], m_devices[m_platform], &contextNotify, NULL, &status);
    CHECKSTATUS(status);
    m_hDeviceId = m_devices[m_platform][m_device];
    if (m_hContext)
        LOG_INFO(1, "OpenCL context successfully initialized on platform: " << m_platform << ", device: " << m_device);
    m_hQueue = clCreateCommandQueue(m_hContext, m_hDeviceId, 0, &status);
    CHECKSTATUS(status);
    if (m_hQueue)
        LOG_INFO(3, "Queue successfully created");

    // Setup device memory
    LOG_INFO(3, "Setup device memory");
    vec1i errorCode = 0;
    reshape();
    m_dBoundingBoxes = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY, sizeof(BoundingBox) * NB_MAX_BOXES, 0, &errorCode);
    m_dLamps = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY, sizeof(Lamp) * NB_MAX_LAMPS, 0, &errorCode);
    m_dLightInformation = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY,
                                         sizeof(LightInformation) * NB_MAX_LIGHTINFORMATIONS, 0, &errorCode);
    m_dMaterials = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY, sizeof(Material) * NB_MAX_MATERIALS, 0, &errorCode);

#if USE_KINECT
    m_dVideo = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY,
                              KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * KINECT_COLOR_DEPTH, 0, &errorCode);
    m_dDepth = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY,
                              KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT * KINECT_DEPTH_DEPTH, 0, &errorCode);
#endif // USE_KINECT
}

void OpenCLKernel::releaseKernels()
{
    // Test kernels
    if (m_kAlignment)
    {
        CHECKSTATUS(clReleaseKernel(m_kAlignment));
        m_kAlignment = 0;
    }
    // Rendering kernels
    if (m_kStandardRenderer)
    {
        CHECKSTATUS(clReleaseKernel(m_kStandardRenderer));
        m_kStandardRenderer = 0;
    }
    if (m_kAnaglyphRenderer)
    {
        CHECKSTATUS(clReleaseKernel(m_kAnaglyphRenderer));
        m_kAnaglyphRenderer = 0;
    }
    if (m_k3DVisionRenderer)
    {
        CHECKSTATUS(clReleaseKernel(m_k3DVisionRenderer));
        m_k3DVisionRenderer = 0;
    }
    if (m_kFishEyeRenderer)
    {
        CHECKSTATUS(clReleaseKernel(m_kFishEyeRenderer));
        m_kFishEyeRenderer = 0;
    }
    if (m_kVolumeRenderer)
    {
        CHECKSTATUS(clReleaseKernel(m_kVolumeRenderer));
        m_kVolumeRenderer = 0;
    }

    // Post processing kernels
    if (m_kDefault)
    {
        CHECKSTATUS(clReleaseKernel(m_kDefault));
        m_kDefault = 0;
    }
    if (m_kDepthOfField)
    {
        CHECKSTATUS(clReleaseKernel(m_kDepthOfField));
        m_kDepthOfField = 0;
    }
    if (m_kAmbientOcclusion)
    {
        CHECKSTATUS(clReleaseKernel(m_kAmbientOcclusion));
        m_kAmbientOcclusion = 0;
    }
    if (m_kRadiosity)
    {
        CHECKSTATUS(clReleaseKernel(m_kRadiosity));
        m_kRadiosity = 0;
    }
    if (m_kFilter)
    {
        CHECKSTATUS(clReleaseKernel(m_kFilter));
        m_kFilter = 0;
    }

    m_primitivesTransfered = false;
    m_materialsTransfered = false;
    m_texturesTransfered = false;

    if (m_hProgram)
    {
        CHECKSTATUS(clReleaseProgram(m_hProgram));
        m_hProgram = 0;
    }
}

void OpenCLKernel::releaseDevice()
{
    releaseKernels();

    LOG_INFO(3, "Release device memory");
    if (_dPrimitives)
        CHECKSTATUS(clReleaseMemObject(_dPrimitives));
    _dPrimitives = 0;
    if (m_dBoundingBoxes)
        CHECKSTATUS(clReleaseMemObject(m_dBoundingBoxes));
    if (m_dMaterials)
        CHECKSTATUS(clReleaseMemObject(m_dMaterials));
    if (m_dTextures)
        CHECKSTATUS(clReleaseMemObject(m_dTextures));
    if (m_dRandoms)
        CHECKSTATUS(clReleaseMemObject(m_dRandoms));
    if (m_dPostProcessingBuffer)
        CHECKSTATUS(clReleaseMemObject(m_dPostProcessingBuffer));
    if (m_dPrimitivesXYIds)
        CHECKSTATUS(clReleaseMemObject(m_dPrimitivesXYIds));
    if (m_dBitmap)
        CHECKSTATUS(clReleaseMemObject(m_dBitmap));

    // Queue and context
    if (m_hQueue)
    {
        CHECKSTATUS(clReleaseCommandQueue(m_hQueue));
        m_hQueue = 0;
    }
    if (m_hContext)
    {
        CHECKSTATUS(clReleaseContext(m_hContext));
        m_hContext = 0;
    }
}

/*
 * runKernel
 */
void OpenCLKernel::render_begin(const float timer)
{
#ifdef WIN32
    if (m_sceneInfo.pathTracingIteration == 0)
        m_counter = GetTickCount();
#endif

    GPUKernel::render_begin(timer);
    if (m_refresh)
    {
        // CPU -> GPU Data transfers
        int nbBoxes = m_nbActiveBoxes[m_frame];
        int nbPrimitives = m_nbActivePrimitives[m_frame];
        int nbLamps = m_nbActiveLamps[m_frame];
        int nbMaterials = m_nbActiveMaterials + 1;
        LOG_INFO(3, "Data sizes [" << m_frame << "]: " << nbBoxes << ", " << nbPrimitives << ", "
                                   << m_lightInformationSize << ", " << nbLamps);

        if (!m_primitivesTransfered)
        {
            CHECKSTATUS(clEnqueueWriteBuffer(m_hQueue, m_dBoundingBoxes, CL_TRUE, 0, nbBoxes * sizeof(BoundingBox),
                                             m_hBoundingBoxes, 0, NULL, NULL));

            int errorCode;
            if (_dPrimitives)
                CHECKSTATUS(clReleaseMemObject(_dPrimitives));
            _dPrimitives =
                clCreateBuffer(m_hContext, CL_MEM_READ_ONLY, sizeof(Primitive) * nbPrimitives, 0, &errorCode);
            CHECKSTATUS(clEnqueueWriteBuffer(m_hQueue, _dPrimitives, CL_TRUE, 0, nbPrimitives * sizeof(Primitive),
                                             m_hPrimitives, 0, NULL, NULL));
            CHECKSTATUS(
                clEnqueueWriteBuffer(m_hQueue, m_dLamps, CL_TRUE, 0, nbLamps * sizeof(Lamp), m_hLamps, 0, NULL, NULL));
            CHECKSTATUS(clEnqueueWriteBuffer(m_hQueue, m_dLightInformation, CL_TRUE, 0,
                                             m_lightInformationSize * sizeof(LightInformation), m_lightInformation, 0,
                                             NULL, NULL));
            m_primitivesTransfered = true;
        }

        if (!m_randomsTransfered)
        {
            CHECKSTATUS(clEnqueueWriteBuffer(m_hQueue, m_dRandoms, CL_TRUE, 0,
                                             m_sceneInfo.size.x * m_sceneInfo.size.y * sizeof(RandomBuffer), m_hRandoms,
                                             0, NULL, NULL));
            m_randomsTransfered = true;
        }

        if (!m_materialsTransfered)
        {
            realignTexturesAndMaterials();
            CHECKSTATUS(clEnqueueWriteBuffer(m_hQueue, m_dMaterials, CL_TRUE, 0, nbMaterials * sizeof(Material),
                                             m_hMaterials, 0, NULL, NULL));
            m_materialsTransfered = true;
        }

#ifdef USE_KINECT
        if (m_kinectEnabled)
        {
            // Video
            const NUI_IMAGE_FRAME *pImageFrame = 0;
            WaitForSingleObject(m_hNextVideoFrameEvent, INFINITE);
            HRESULT status = NuiImageStreamGetNextFrame(m_pVideoStreamHandle, 0, &pImageFrame);
            if ((status == S_OK) && pImageFrame)
            {
                INuiFrameTexture *pTexture = pImageFrame->pFrameTexture;
                NUI_LOCKED_RECT LockedRect;
                pTexture->LockRect(0, &LockedRect, NULL, 0);
                if (LockedRect.Pitch != 0)
                {
                    m_hVideo = (unsigned char *)LockedRect.pBits;
                }
            }

            // Depth
            const NUI_IMAGE_FRAME *pDepthFrame = 0;
            WaitForSingleObject(m_hNextDepthFrameEvent, INFINITE);
            status = NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &pDepthFrame);
            if ((status == S_OK) && pDepthFrame)
            {
                INuiFrameTexture *pTexture = pDepthFrame->pFrameTexture;
                if (pTexture)
                {
                    NUI_LOCKED_RECT LockedRectDepth;
                    pTexture->LockRect(0, &LockedRectDepth, NULL, 0);
                    if (LockedRectDepth.Pitch != 0)
                    {
                        m_hDepth = (unsigned char *)LockedRectDepth.pBits;
                    }
                }
            }
            NuiImageStreamReleaseFrame(m_pVideoStreamHandle, pImageFrame);
            NuiImageStreamReleaseFrame(m_pDepthStreamHandle, pDepthFrame);

            m_hTextures[0].buffer = m_hVideo;
            m_hTextures[1].buffer = m_hDepth;
        }

#endif // USE_KINECT

        if (!m_texturesTransfered)
        {
            int totalSize(0);
            for (int i(0); i < m_nbActiveTextures; ++i)
            {
                totalSize += m_hTextures[i].size.x * m_hTextures[i].size.y * m_hTextures[i].size.z;
            }
            LOG_INFO(3, "Total texture size: " << totalSize << " bytes");

            if (m_dTextures)
            {
                LOG_INFO(3, "Releasing existing texture resources");
                CHECKSTATUS(clReleaseMemObject(m_dTextures));
                m_dTextures = 0;
            }

            if (totalSize > 0)
            {
                // Allocate host buffer
                BitmapBuffer *tmpTextures = new BitmapBuffer[totalSize];
                for (int i(0); i < m_nbActiveTextures; ++i)
                {
                    if (m_hTextures[i].buffer != 0)
                    {
                        int textureSize = m_hTextures[i].size.x * m_hTextures[i].size.y * m_hTextures[i].size.z;
                        memcpy(tmpTextures + m_hTextures[i].offset, m_hTextures[i].buffer, textureSize);
                    }
                }
                LOG_INFO(3, "Creating texture buffer");
                m_dTextures = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY, totalSize * sizeof(BitmapBuffer), 0, NULL);
                CHECKSTATUS(clEnqueueWriteBuffer(m_hQueue, m_dTextures, CL_TRUE, 0, totalSize * sizeof(BitmapBuffer),
                                                 tmpTextures, 0, NULL, NULL));
                LOG_INFO(3, "Total GPU texture memory allocated: " << totalSize << " bytes");
                delete[] tmpTextures;
            }
            m_texturesTransfered = true;
        }

        // Kernel execution
        LOG_INFO(3, "CPU PostProcessingBuffer: " << sizeof(PostProcessingBuffer));
        LOG_INFO(3, "CPU PrimitiveXYIdBuffer : " << sizeof(PrimitiveXYIdBuffer));
        LOG_INFO(3, "CPU BoundingBox         : " << sizeof(BoundingBox));
        LOG_INFO(3, "CPU Primitive           : " << sizeof(Primitive));
        LOG_INFO(3, "CPU Material            : " << sizeof(Material));

        SceneInfo sceneInfo = m_sceneInfo;
        if (m_sceneInfo.draftMode && m_sceneInfo.pathTracingIteration == 0)
            sceneInfo.graphicsLevel = glNoShading;

        size_t szLocalWorkSize[] = {1, 1};
        size_t szGlobalWorkSize[] = {m_sceneInfo.size.x / szLocalWorkSize[0], m_sceneInfo.size.y / szLocalWorkSize[1]};
        int zero(0);
        LOG_INFO(3, "Running default rendering kernel");
        switch (sceneInfo.cameraType)
        {
        case ctAnaglyph:
        {
            // Run the post processing kernel!!
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 1, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 2, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 3, sizeof(cl_mem), (void *)&m_dBoundingBoxes));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 4, sizeof(vec1i), (void *)&nbBoxes));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 5, sizeof(cl_mem), (void *)&_dPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 6, sizeof(vec1i), (void *)&nbPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 7, sizeof(cl_mem), (void *)&m_dLightInformation));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 8, sizeof(vec1i), (void *)&m_lightInformationSize));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 9, sizeof(vec1i), (void *)&nbLamps));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 10, sizeof(cl_mem), (void *)&m_dMaterials));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 11, sizeof(cl_mem), (void *)&m_dTextures));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 12, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 13, sizeof(vec4f), (void *)&m_viewPos));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 14, sizeof(vec4f), (void *)&m_viewDir));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 15, sizeof(vec4f), (void *)&m_angles));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 16, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(
                clSetKernelArg(m_kAnaglyphRenderer, 17, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 18, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kAnaglyphRenderer, 19, sizeof(cl_mem), (void *)&m_dPrimitivesXYIds));
            CHECKSTATUS(clEnqueueNDRangeKernel(m_hQueue, m_kAnaglyphRenderer, 2, NULL, szGlobalWorkSize,
                                               szLocalWorkSize, 0, 0, 0));
            break;
        }
        case ctVR:
        {
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 1, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 2, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 3, sizeof(cl_mem), (void *)&m_dBoundingBoxes));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 4, sizeof(vec1i), (void *)&nbBoxes));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 5, sizeof(cl_mem), (void *)&_dPrimitives));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 6, sizeof(vec1i), (void *)&nbPrimitives));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 7, sizeof(cl_mem), (void *)&m_dLightInformation));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 8, sizeof(vec1i), (void *)&m_lightInformationSize));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 9, sizeof(vec1i), (void *)&nbLamps));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 10, sizeof(cl_mem), (void *)&m_dMaterials));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 11, sizeof(cl_mem), (void *)&m_dTextures));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 12, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 13, sizeof(vec4f), (void *)&m_viewPos));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 14, sizeof(vec4f), (void *)&m_viewDir));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 15, sizeof(vec4f), (void *)&m_angles));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 16, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(
                clSetKernelArg(m_k3DVisionRenderer, 17, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 18, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_k3DVisionRenderer, 19, sizeof(cl_mem), (void *)&m_dPrimitivesXYIds));
            CHECKSTATUS(clEnqueueNDRangeKernel(m_hQueue, m_k3DVisionRenderer, 2, NULL, szGlobalWorkSize,
                                               szLocalWorkSize, 0, 0, 0));
            break;
        }
        case ctPanoramic:
        {
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 1, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 2, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 3, sizeof(cl_mem), (void *)&m_dBoundingBoxes));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 4, sizeof(vec1i), (void *)&nbBoxes));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 5, sizeof(cl_mem), (void *)&_dPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 6, sizeof(vec1i), (void *)&nbPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 7, sizeof(cl_mem), (void *)&m_dLightInformation));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 8, sizeof(vec1i), (void *)&m_lightInformationSize));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 9, sizeof(vec1i), (void *)&nbLamps));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 10, sizeof(cl_mem), (void *)&m_dMaterials));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 11, sizeof(cl_mem), (void *)&m_dTextures));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 12, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 13, sizeof(vec4f), (void *)&m_viewPos));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 14, sizeof(vec4f), (void *)&m_viewDir));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 15, sizeof(vec4f), (void *)&m_angles));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 16, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(
                clSetKernelArg(m_kFishEyeRenderer, 17, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 18, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kFishEyeRenderer, 19, sizeof(cl_mem), (void *)&m_dPrimitivesXYIds));
            CHECKSTATUS(clEnqueueNDRangeKernel(m_hQueue, m_kFishEyeRenderer, 2, NULL, szGlobalWorkSize, szLocalWorkSize,
                                               0, 0, 0));
            break;
        }
        case ctVolumeRendering:
        {
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 1, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 2, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 3, sizeof(cl_mem), (void *)&m_dBoundingBoxes));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 4, sizeof(vec1i), (void *)&nbBoxes));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 5, sizeof(cl_mem), (void *)&_dPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 6, sizeof(vec1i), (void *)&nbPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 7, sizeof(cl_mem), (void *)&m_dLightInformation));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 8, sizeof(vec1i), (void *)&m_lightInformationSize));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 9, sizeof(vec1i), (void *)&nbLamps));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 10, sizeof(cl_mem), (void *)&m_dMaterials));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 11, sizeof(cl_mem), (void *)&m_dTextures));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 12, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 13, sizeof(vec4f), (void *)&m_viewPos));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 14, sizeof(vec4f), (void *)&m_viewDir));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 15, sizeof(vec4f), (void *)&m_angles));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 16, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(
                clSetKernelArg(m_kVolumeRenderer, 17, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 18, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kVolumeRenderer, 19, sizeof(cl_mem), (void *)&m_dPrimitivesXYIds));
            CHECKSTATUS(clEnqueueNDRangeKernel(m_hQueue, m_kVolumeRenderer, 2, NULL, szGlobalWorkSize, szLocalWorkSize,
                                               0, 0, 0));
            break;
        }
        default:
        {
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 1, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 2, sizeof(vec1i), (void *)&zero));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 3, sizeof(cl_mem), (void *)&m_dBoundingBoxes));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 4, sizeof(vec1i), (void *)&nbBoxes));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 5, sizeof(cl_mem), (void *)&_dPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 6, sizeof(vec1i), (void *)&nbPrimitives));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 7, sizeof(cl_mem), (void *)&m_dLightInformation));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 8, sizeof(vec1i), (void *)&m_lightInformationSize));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 9, sizeof(vec1i), (void *)&nbLamps));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 10, sizeof(cl_mem), (void *)&m_dMaterials));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 11, sizeof(cl_mem), (void *)&m_dTextures));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 12, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 13, sizeof(vec4f), (void *)&m_viewPos));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 14, sizeof(vec4f), (void *)&m_viewDir));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 15, sizeof(vec4f), (void *)&m_angles));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 16, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(
                clSetKernelArg(m_kStandardRenderer, 17, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 18, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kStandardRenderer, 19, sizeof(cl_mem), (void *)&m_dPrimitivesXYIds));
            CHECKSTATUS(clEnqueueNDRangeKernel(m_hQueue, m_kStandardRenderer, 2, NULL, szGlobalWorkSize,
                                               szLocalWorkSize, 0, 0, 0));
            break;
        }
        }
        LOG_INFO(3, "Rendering kernel done");

        // --------------------------------------------------------------------------------
        // Post processing
        // --------------------------------------------------------------------------------
        LOG_INFO(3, "Running Post-Processing kernel");
        switch (m_postProcessingInfo.type)
        {
        case ppe_depthOfField:
            CHECKSTATUS(clSetKernelArg(m_kDepthOfField, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kDepthOfField, 1, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(clSetKernelArg(m_kDepthOfField, 2, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kDepthOfField, 3, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kDepthOfField, 4, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kDepthOfField, 5, sizeof(cl_mem), (void *)&m_dBitmap));
            CHECKSTATUS(
                clEnqueueNDRangeKernel(m_hQueue, m_kDepthOfField, 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, 0, 0));
            break;
        case ppe_ambientOcclusion:
            CHECKSTATUS(clSetKernelArg(m_kAmbientOcclusion, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kAmbientOcclusion, 1, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(
                clSetKernelArg(m_kAmbientOcclusion, 2, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kAmbientOcclusion, 3, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kAmbientOcclusion, 4, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kAmbientOcclusion, 5, sizeof(cl_mem), (void *)&m_dBitmap));
            CHECKSTATUS(clEnqueueNDRangeKernel(m_hQueue, m_kAmbientOcclusion, 2, NULL, szGlobalWorkSize,
                                               szLocalWorkSize, 0, 0, 0));
            break;
        case ppe_radiosity:
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 1, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 2, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 3, sizeof(cl_mem), (void *)&m_dPrimitivesXYIds));
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 4, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 5, sizeof(cl_mem), (void *)&m_dRandoms));
            CHECKSTATUS(clSetKernelArg(m_kRadiosity, 6, sizeof(cl_mem), (void *)&m_dBitmap));
            CHECKSTATUS(
                clEnqueueNDRangeKernel(m_hQueue, m_kRadiosity, 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, 0, 0));
            break;
        case ppe_filter:
            CHECKSTATUS(clSetKernelArg(m_kFilter, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kFilter, 1, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(clSetKernelArg(m_kFilter, 2, sizeof(PostProcessingInfo), (void *)&m_postProcessingInfo));
            CHECKSTATUS(clSetKernelArg(m_kFilter, 3, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kFilter, 4, sizeof(cl_mem), (void *)&m_dBitmap));
            CHECKSTATUS(
                clEnqueueNDRangeKernel(m_hQueue, m_kFilter, 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, 0, 0));
            break;
        default:
            CHECKSTATUS(clSetKernelArg(m_kDefault, 0, sizeof(vec2i), (void *)&m_occupancyParameters));
            CHECKSTATUS(clSetKernelArg(m_kDefault, 1, sizeof(SceneInfo), (void *)&sceneInfo));
            CHECKSTATUS(clSetKernelArg(m_kDefault, 2, sizeof(cl_mem), (void *)&m_dPostProcessingBuffer));
            CHECKSTATUS(clSetKernelArg(m_kDefault, 3, sizeof(cl_mem), (void *)&m_dBitmap));
            CHECKSTATUS(
                clEnqueueNDRangeKernel(m_hQueue, m_kDefault, 2, NULL, szGlobalWorkSize, szLocalWorkSize, 0, 0, 0));
            break;
        }
        LOG_INFO(3, "Post-Processing Kernel done");
    }
    m_refresh = (m_sceneInfo.pathTracingIteration < m_sceneInfo.maxPathTracingIterations);
}

void OpenCLKernel::render_end()
{
    // ------------------------------------------------------------
    // Read back the results
    // ------------------------------------------------------------
    size_t size = m_sceneInfo.size.x * m_sceneInfo.size.y * sizeof(BitmapBuffer) * gColorDepth;
    LOG_INFO(3, m_hQueue << ", " << m_dBitmap << ", " << m_bitmap << " - Bitmap Size=" << size);
    CHECKSTATUS(clEnqueueReadBuffer(m_hQueue, m_dBitmap, CL_TRUE, 0, size, m_bitmap, 0, NULL, NULL));
    size = m_sceneInfo.size.x * m_sceneInfo.size.y * sizeof(PrimitiveXYIdBuffer);
    LOG_INFO(3, "PrimitivesID Size=" << size);
    CHECKSTATUS(clEnqueueReadBuffer(m_hQueue, m_dPrimitivesXYIds, CL_TRUE, 0, size, m_hPrimitivesXYIds, 0, NULL, NULL));
    LOG_INFO(3, "Flushing queues");
    CHECKSTATUS(clFlush(m_hQueue));
    CHECKSTATUS(clFinish(m_hQueue));
#ifdef WIN32
    if (m_sceneInfo.pathTracingIteration == m_sceneInfo.maxPathTracingIterations - 1)
        LOG_INFO(1, "Rendering completed in " << GetTickCount() - m_counter << " ms");
#endif

    if (m_sceneInfo.frameBufferType == 0)
    {
        ::glEnable(GL_TEXTURE_2D);
        //::glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        //::glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        ::glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        //::glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
        ::glTexImage2D(GL_TEXTURE_2D, 0, gColorDepth, m_sceneInfo.size.x, m_sceneInfo.size.y, 0, GL_RGB,
                       GL_UNSIGNED_BYTE, m_bitmap);

        if (m_sceneInfo.cameraType == ctVR)
        {
            float step = 0.1f;
            float halfStep = 1.f;
            float scale = 2.f;

            for (int a(0); a < 2; ++a)
            {
                cl_float2 center = {{0.f, 0.f}};
                center.x = (a == 0) ? -0.5f : 0.5f;
                float b = (a == 0) ? 0.f : 0.5f;

                for (float x(0); x < 1; x += step)
                {
                    for (float y(0); y < 1; y += step)
                    {
                        cl_float2 s;
                        s.x = scale;
                        s.y = scale;

                        cl_float2 p0 = {{s.x * x - halfStep, s.y * y - halfStep}};
                        cl_float2 p1 = {{s.x * (x + step) - halfStep, s.y * y - halfStep}};
                        cl_float2 p2 = {{s.x * (x + step) - halfStep, s.y * (y + step) - halfStep}};
                        cl_float2 p3 = {{s.x * x - halfStep, s.y * (y + step) - halfStep}};

                        float d0 = sqrt(pow(p0.x, 2) + pow(p0.y, 2));
                        float d1 = sqrt(pow(p1.x, 2) + pow(p1.y, 2));
                        float d2 = sqrt(pow(p2.x, 2) + pow(p2.y, 2));
                        float d3 = sqrt(pow(p3.x, 2) + pow(p3.y, 2));

                        d0 = 1.f - pow(d0, 2.f) * m_distortion;
                        d1 = 1.f - pow(d1, 2.f) * m_distortion;
                        d2 = 1.f - pow(d2, 2.f) * m_distortion;
                        d3 = 1.f - pow(d3, 2.f) * m_distortion;

                        ::glBegin(GL_QUADS);
                        ::glTexCoord2f(1.f - (b + (x / 2.f)), y);
                        ::glVertex3f(center.x + 0.5f * p0.x * d0, center.y + p0.y * d0, 0.f);

                        ::glTexCoord2f(1.f - (b + (x + step) / 2.f), y);
                        ::glVertex3f(center.x + 0.5f * p1.x * d1, center.y + p1.y * d1, 0.f);

                        ::glTexCoord2f(1.f - (b + (x + step) / 2.f), y + step);
                        ::glVertex3f(center.x + 0.5f * p2.x * d2, center.y + p2.y * d2, 0.f);

                        ::glTexCoord2f(1.f - (b + (x / 2.f)), y + step);
                        ::glVertex3f(center.x + 0.5f * p3.x * d3, center.y + p3.y * d3, 0.f);
                        ::glEnd();
                    }
                }
            }
        }
        else
        {
            ::glBegin(GL_QUADS);
            ::glTexCoord2f(1.f, 0.f);
            ::glVertex3f(-1.f, -1.f, 0.f);

            ::glTexCoord2f(0.f, 0.f);
            ::glVertex3f(1.f, -1.f, 0.f);

            ::glTexCoord2f(0.f, 1.f);
            ::glVertex3f(1.f, 1.f, 0.f);

            ::glTexCoord2f(1.f, 1.f);
            ::glVertex3f(-1.f, 1.f, 0.f);
            ::glEnd();
        }
        ::glDisable(GL_TEXTURE_2D);
    }
}

void OpenCLKernel::initBuffers()
{
    LOG_INFO(3, "OpenCLKernel::initBuffers");
    initializeDevice();
    GPUKernel::initBuffers();
    recompileKernels();
}

OpenCLKernel::~OpenCLKernel()
{
    LOG_INFO(3, "OpenCLKernel::~OpenCLKernel");
    // Clean up
    releaseDevice();

#if USE_KINECT
    CloseHandle(m_skeletons);
    CloseHandle(m_hNextDepthFrameEvent);
    CloseHandle(m_hNextVideoFrameEvent);
    CloseHandle(m_hNextSkeletonEvent);
    NuiShutdown();
#endif // USE_KINECT
}

void OpenCLKernel::reshape()
{
    LOG_INFO(3, "OpenCLKernel::reshape");
    GPUKernel::reshape();
    if (m_dRandoms)
        CHECKSTATUS(clReleaseMemObject(m_dRandoms));
    if (m_dPostProcessingBuffer)
        CHECKSTATUS(clReleaseMemObject(m_dPostProcessingBuffer));
    if (m_dPrimitivesXYIds)
        CHECKSTATUS(clReleaseMemObject(m_dPrimitivesXYIds));
    if (m_dBitmap)
        CHECKSTATUS(clReleaseMemObject(m_dBitmap));

    int errorCode;
    m_dBitmap = clCreateBuffer(m_hContext, CL_MEM_READ_WRITE, MAX_BITMAP_SIZE * sizeof(BitmapBuffer) * gColorDepth, 0,
                               &errorCode);
    m_dRandoms = clCreateBuffer(m_hContext, CL_MEM_READ_ONLY, MAX_BITMAP_SIZE * sizeof(RandomBuffer), 0, &errorCode);
    m_dPostProcessingBuffer =
        clCreateBuffer(m_hContext, CL_MEM_READ_WRITE, MAX_BITMAP_SIZE * sizeof(PostProcessingBuffer), 0, &errorCode);
    m_dPrimitivesXYIds =
        clCreateBuffer(m_hContext, CL_MEM_READ_WRITE, MAX_BITMAP_SIZE * sizeof(PrimitiveXYIdBuffer), 0, &errorCode);
}

int OpenCLKernel::getNumPlatforms()
{
    return m_numberOfPlatforms;
}

int OpenCLKernel::getNumDevices(const int platform)
{
    return m_numberOfDevices[platform];
}

std::string OpenCLKernel::getPlatformDescription(const int platform)
{
    return m_platformsDescription[platform];
}

std::string OpenCLKernel::getDeviceDescription(const int platform, const int device)
{
    return m_devicesDescription[platform][device];
}

std::string OpenCLKernel::getGPUDescription()
{
    return m_devicesDescription[m_platform][m_device];
}
void OpenCLKernel::queryDevice()
{
    char buffer[MAX_SOURCE_SIZE];
    size_t len;
    std::stringstream s;

    CHECKSTATUS(clGetPlatformIDs(MAX_PLATFORMS, m_platforms, &m_numberOfPlatforms));
    LOG_INFO(1, "Number of platforms detected: " << m_numberOfPlatforms);

    for (cl_uint platform(0); platform < m_numberOfPlatforms; ++platform)
    {
        // Platform details
        std::string platformDescription;
        LOG_INFO(1, "Platform " << platform);

        CHECKSTATUS(clGetPlatformInfo(m_platforms[platform], CL_PLATFORM_NAME, MAX_SOURCE_SIZE, buffer, &len));
        buffer[len] = 0;
        LOG_INFO(1, "  Name..............: " << buffer);
        platformDescription = buffer;

        CHECKSTATUS(clGetPlatformInfo(m_platforms[platform], CL_PLATFORM_VERSION, MAX_SOURCE_SIZE, buffer, &len));
        buffer[len] = 0;
        LOG_INFO(1, "  Version...........: " << buffer);
        platformDescription += " (";
        platformDescription += buffer;

        CHECKSTATUS(clGetPlatformInfo(m_platforms[platform], CL_PLATFORM_VENDOR, MAX_SOURCE_SIZE, buffer, &len));
        buffer[len] = 0;
        LOG_INFO(1, "  Vendor............: " << buffer);
        platformDescription += ", ";
        platformDescription += buffer;
        platformDescription += ")";

        CHECKSTATUS(clGetPlatformInfo(m_platforms[platform], CL_PLATFORM_PROFILE, MAX_SOURCE_SIZE, buffer, &len));
        buffer[len] = 0;
        LOG_INFO(1, "  Profile...........: " << buffer);

        CHECKSTATUS(clGetPlatformInfo(m_platforms[platform], CL_PLATFORM_EXTENSIONS, MAX_SOURCE_SIZE, buffer, &len));
        buffer[len] = 0;
        LOG_INFO(3, "  Extensions........: " << buffer);

        m_platformsDescription[platform] = platformDescription;

        if (clGetDeviceIDs(m_platforms[platform], CL_DEVICE_TYPE_ALL, MAX_DEVICES, m_devices[platform],
                           &m_numberOfDevices[platform]) == CL_SUCCESS)
        {
            // m_devices
            for (cl_uint device = 0; device < m_numberOfDevices[platform]; ++device)
            {
                LOG_INFO(1, "  --------------------------------------");
                LOG_INFO(1, "  Device " << device << " id=" << m_devices[platform][device]);
                LOG_INFO(1, "  --------------------------------------");
                std::string deviceDescription;
                CHECKSTATUS(clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_NAME, sizeof(buffer), buffer, NULL));
                LOG_INFO(1, "    Name............: " << buffer);
                deviceDescription = buffer;

                CHECKSTATUS(
                    clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_VENDOR, sizeof(buffer), buffer, NULL));
                LOG_INFO(1, "    Vendor..........: " << buffer);
                deviceDescription += " (";
                deviceDescription += buffer;

                CHECKSTATUS(
                    clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_VERSION, sizeof(buffer), buffer, NULL));
                LOG_INFO(1, "    Version.........: " << buffer);
                deviceDescription += ", ";
                deviceDescription += buffer;

                CHECKSTATUS(
                    clGetDeviceInfo(m_devices[platform][device], CL_DRIVER_VERSION, sizeof(buffer), buffer, NULL));
                LOG_INFO(1, "    Driver version..: " << buffer);
                deviceDescription += ", ";
                deviceDescription += buffer;
                deviceDescription += ")";

                m_devicesDescription[platform][device] = deviceDescription;

                cl_uint value;
                cl_uint values[10];
                CHECKSTATUS(clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(value),
                                            &value, NULL));
                LOG_INFO(3, "    Compute units...: " << value);
                CHECKSTATUS(clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                                            sizeof(value), &value, NULL));
                LOG_INFO(3, "    Work item dims..: " << value);
                CHECKSTATUS(clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(values),
                                            &values, NULL));
                LOG_INFO(3, "    Work item size..: " << values[0] << ", " << values[1] << ", " << values[2]);
                CHECKSTATUS(clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(value),
                                            &value, NULL));
                LOG_INFO(1, "    Clock frequency.: " << value << " Hz");

                cl_device_type infoType;
                CHECKSTATUS(
                    clGetDeviceInfo(m_devices[platform][device], CL_DEVICE_TYPE, sizeof(infoType), &infoType, NULL));
                if (infoType & CL_DEVICE_TYPE_DEFAULT)
                {
                    infoType &= ~CL_DEVICE_TYPE_DEFAULT;
                    LOG_INFO(1, "    Type............: Default");
                }
                if (infoType & CL_DEVICE_TYPE_CPU)
                {
                    infoType &= ~CL_DEVICE_TYPE_CPU;
                    LOG_INFO(1, "    Type............: CPU");
                }
                if (infoType & CL_DEVICE_TYPE_GPU)
                {
                    infoType &= ~CL_DEVICE_TYPE_GPU;
                    LOG_INFO(1, "    Type............: GPU");
                }
                if (infoType & CL_DEVICE_TYPE_ACCELERATOR)
                {
                    infoType &= ~CL_DEVICE_TYPE_ACCELERATOR;
                    LOG_INFO(1, "    Type............: Accelerator");
                }
                if (infoType != 0)
                    LOG_INFO(1, "    Type............: Unknown " << infoType);
            }
        }
        else
        {
            LOG_INFO(1, "   No device for this platform");
        }
    }

    LOG_INFO(3, "Data type sizes (in bytes)");
    LOG_INFO(3, "- float             : " << sizeof(float));
    LOG_INFO(3, "- vec2f             : " << sizeof(vec2f));
    LOG_INFO(3, "- vec3f             : " << sizeof(vec3f));
    LOG_INFO(3, "- vec4f             : " << sizeof(vec4f));
    LOG_INFO(3, "- int               : " << sizeof(int));
    LOG_INFO(3, "- vec2i             : " << sizeof(vec2i));
    LOG_INFO(3, "- vec3i             : " << sizeof(vec3i));
    LOG_INFO(3, "- vec4i             : " << sizeof(vec4i));
    LOG_INFO(3, "- SceneInfo         : " << sizeof(SceneInfo));
    LOG_INFO(3, "- Ray               : " << sizeof(Ray));
    LOG_INFO(3, "- LightInformation  : " << sizeof(LightInformation));
    LOG_INFO(3, "- Material          : " << sizeof(Material));
    LOG_INFO(3, "- BoundingBox       : " << sizeof(BoundingBox));
    LOG_INFO(3, "- Primitive         : " << sizeof(Primitive));
    LOG_INFO(3, "- TextureInfo       : " << sizeof(TextureInfo));
    LOG_INFO(3, "- PostProcessingInfo: " << sizeof(PostProcessingInfo));
    LOG_INFO(3, "_______________________________________|");
}
} // namespace solr
