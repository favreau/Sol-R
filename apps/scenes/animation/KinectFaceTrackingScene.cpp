/* Copyright (c) 2011-2014, Cyrille Favreau
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

#include "KinectFaceTrackingScene.h"

#include <opengl/rtgl.h>

#ifdef USE_KINECT
// Include the main Kinect SDK .h file
#include <NuiAPI.h>
// Include the Face Tracking SDK .h file
#include <FaceTrackLib.h>

#pragma comment(lib, "FaceTrackLib.lib")

KinectFaceTrackingScene::KinectFaceTrackingScene(const std::string& name)
    : Scene(name)
    , m_faceTracker(nullptr)
{
}

KinectFaceTrackingScene::~KinectFaceTrackingScene(void) {}

void KinectFaceTrackingScene::doInitialize()
{
    // Create an instance of a face tracker
    m_faceTracker = FTCreateFaceTracker();
}

void KinectFaceTrackingScene::doAnimate()
{
    if (m_faceTracker)
    {
        // Video camera config with width, height, focal length in pixels
        // NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS focal length is computed for 640x480 resolution
        // If you use different resolutions, multiply this focal length by the scaling factor
        FT_CAMERA_CONFIG videoCameraConfig = {640, 480, NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS};

        // Depth camera config with width, height, focal length in pixels
        // NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS focal length is computed for 320x240 resolution
        // If you use different resolutions, multiply this focal length by the scaling factor
        FT_CAMERA_CONFIG depthCameraConfig = {320, 240, NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS};

        // Initialize the face tracker
        HRESULT hr = m_faceTracker->Initialize(&videoCameraConfig, &depthCameraConfig, NULL, NULL);
        if (FAILED(hr))
        {
            // Handle errors
            LOG_ERROR("[" << hr << "] Face tracker could not be initialized");
        }
        else
        {
            // Create a face tracking result interface
            IFTResult* pFTResult = NULL;
            hr = m_faceTracker->CreateFTResult(&pFTResult);
            if (FAILED(hr))
            {
                // Handle errors
            }
            else
            {
                IFTModel* model;
                hr = m_faceTracker->GetFaceModel(&model);
                if (FAILED(hr))
                {
                    // Handle errors
                }
                else
                {
                    const FLOAT shapeUnits[1] = {1.f};
                    const FLOAT animationUnits[1] = {1.f};
                    UINT suCount = model->GetSUCount();
                    UINT auCount = model->GetAUCount();
                    FLOAT scale(1.f);
                    FLOAT rotation[3] = {0.f, 0.f, 0.f};
                    FLOAT translation[3] = {0.f, 0.f, 0.f};
                    UINT vertexCount(0);
                    FT_VECTOR3D* vertices(nullptr);
                    hr = model->Get3DShape(shapeUnits, suCount, animationUnits, auCount, scale, rotation, translation,
                                           vertices, vertexCount);
                    LOG_INFO(1, "Get3DShape: " << vertexCount << " vertices");

                    FT_TRIANGLE* triangles;
                    UINT triangleCount;
                    hr = model->GetTriangles(&triangles, &triangleCount);
                    LOG_INFO(1, "GetTriangles: " << triangleCount << " triangles");
                    if (FAILED(hr))
                    {
                        // Handle errors
                    }
                    else
                    {
                        for (UINT i(0); i < triangleCount; ++i)
                        {
                            glBegin(GL_TRIANGLES);
                            glVertex3f(vertices[triangles[i].i].x, vertices[triangles[i].i].y,
                                       vertices[triangles[i].i].z);
                            glVertex3f(vertices[triangles[i].j].x, vertices[triangles[i].j].y,
                                       vertices[triangles[i].j].z);
                            glVertex3f(vertices[triangles[i].k].x, vertices[triangles[i].k].y,
                                       vertices[triangles[i].k].z);
                            glNormal3f(0.f, 0.f, -1.f);
                            glNormal3f(0.f, 0.f, -1.f);
                            glNormal3f(0.f, 0.f, -1.f);
                            glTexCoord3f(0.f, 0.f, 0.f);
                            glTexCoord3f(1.f, 0.f, 0.f);
                            glTexCoord3f(1.f, 1.f, 0.f);
                            m_nbPrimitives = glEnd();
                        }
                    }
                }
            }
        }
    }
    else
    {
        // Handle errors
        LOG_ERROR("Face tracker could not be instanciated");
    }
}

void KinectFaceTrackingScene::doAddLights()
{
    // lights
    if (m_gpuKernel->getNbActiveLamps() == 0)
    {
        LOG_INFO(3, "Adding sun light");
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, -5000.f, 5000.f, -5000.f, 10.f, 0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    }
}
#endif
