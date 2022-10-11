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

#include "ColumnScene.h"

// System
#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif
#include <math.h>

// Project
#include <common/Utils.h>

// Sol-R
#include <solr/io/FileMarshaller.h>
#include <solr/io/OBJReader.h>

ColumnScene::ColumnScene(const std::string& name)
    : Scene(name)
{
}

ColumnScene::~ColumnScene(void) {}

/*
________________________________________________________________________________

Create simple scene with cylinders
________________________________________________________________________________
*/
void ColumnScene::doInitialize()
{
    const float s = 10.f;
    const float boxScale = 100.f;

    std::string path = std::string(DEFAULT_MEDIA_FOLDER) + "/Meshes";

    vec4f objectPosition = make_vec4f();
    m_objectScale.x = s;
    m_objectScale.y = s;
    m_objectScale.z = s;

    // Scene Bounding Box
    SceneInfo& sceneInfo = m_gpuKernel->getSceneInfo();
    CPUBoundingBox AABB;
    AABB.parameters[0].x = sceneInfo.viewDistance;
    AABB.parameters[0].y = sceneInfo.viewDistance;
    AABB.parameters[0].z = sceneInfo.viewDistance;
    AABB.parameters[1].x = -sceneInfo.viewDistance;
    AABB.parameters[1].y = -sceneInfo.viewDistance;
    AABB.parameters[1].z = -sceneInfo.viewDistance;

    Strings extensions;
    extensions.push_back(".obj");
    extensions.push_back(".mtl");
    const Strings fileNames =
        getFilesFromFolder(std::string(DEFAULT_MEDIA_FOLDER) + "/obj",
                           extensions);

    const std::string star("neuron_12558.obj");
    OBJReader objectReader;
    CPUBoundingBox inAABB;
    memset(&inAABB, 0, sizeof(CPUBoundingBox));
    const vec4f size =
        objectReader.loadModelFromFile(path + "\\" + star, *m_gpuKernel,
                                       objectPosition, false, m_objectScale,
                                       false, 10, false, false, AABB, false,
                                       inAABB);

    const vec4f halfSize =
        make_vec4f((AABB.parameters[1].x - AABB.parameters[0].x) / 2.f,
                   (AABB.parameters[1].y - AABB.parameters[0].y) / 2.f,
                   (AABB.parameters[1].z - AABB.parameters[0].z) / 2.f);

    const vec4f boxCenter =
        make_vec4f((AABB.parameters[0].x + AABB.parameters[1].x) / 2.f,
                   (AABB.parameters[0].y + AABB.parameters[1].y) / 2.f,
                   (AABB.parameters[0].z + AABB.parameters[1].z) / 2.f);

    inAABB.parameters[0].x = (boxCenter.x - halfSize.x * boxScale);
    inAABB.parameters[0].y = (boxCenter.y - halfSize.y * boxScale);
    inAABB.parameters[0].z = (boxCenter.z - halfSize.y * boxScale);
    inAABB.parameters[1].x = (boxCenter.x + halfSize.x * boxScale);
    inAABB.parameters[1].y = (boxCenter.y + halfSize.y * boxScale);
    inAABB.parameters[1].z = (boxCenter.z + halfSize.z * boxScale);

    m_actorPosition = boxCenter;
    LOG_INFO(1, "Actor Position: " << m_actorPosition.x << ","
                                   << m_actorPosition.y << ","
                                   << m_actorPosition.z << ")");

    for (int i(0); i < fileNames.size(); ++i)
    {
        m_currentModel = i;
        m_name = fileNames[m_currentModel];
        LOG_INFO(1, "--- Loading " << m_name << " ---");
        if (m_name.find(star) == -1)
        {
            CPUBoundingBox aabb;
            objectReader.loadModelFromFile(m_name, *m_gpuKernel, objectPosition,
                                           false, m_objectScale, false, 11,
                                           false, false, aabb, true, inAABB);

            m_groundHeight = -size.y / 2.f - sceneInfo.geometryEpsilon;

            if (aabb.parameters[0].x < AABB.parameters[0].x)
                AABB.parameters[0].x = aabb.parameters[0].x;
            if (aabb.parameters[0].y < AABB.parameters[0].y)
                AABB.parameters[0].y = aabb.parameters[0].y;
            if (aabb.parameters[0].z < AABB.parameters[0].z)
                AABB.parameters[0].z = aabb.parameters[0].z;

            if (aabb.parameters[1].x > AABB.parameters[1].x)
                AABB.parameters[1].x = aabb.parameters[1].x;
            if (aabb.parameters[1].y > AABB.parameters[1].y)
                AABB.parameters[1].y = aabb.parameters[1].y;
            if (aabb.parameters[1].z > AABB.parameters[1].z)
                AABB.parameters[1].z = aabb.parameters[1].z;
        }
    }

    // Save to binary format
    FileMarshaller fm;
    fm.saveToFile(*m_gpuKernel, "column.irt");
}

void ColumnScene::doAnimate()
{
    m_rotationAngles.x = 0.02f;
    m_rotationAngles.y = 0.01f;
    m_rotationAngles.z = 0.015f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void ColumnScene::doAddLights()
{
    for (int i(0); i < 5; ++i)
    {
        // lights
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, rand() % 20000 - 10000.f,
                                  rand() % 5000 - m_groundHeight,
                                  rand() % 20000 - 10000.f, 10.f, 0.f, 0.f,
                                  120 + i);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    }
}
