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

#include "ObjScene.h"

#include <common/Utils.h>
#include <io/FileMarshaller.h>
#include <io/OBJReader.h>
#include <io/PDBReader.h>

int m_counter = 0;

ObjScene::ObjScene(const std::string& name, const std::string& filename)
    : Scene(name)
    , m_filename(filename)
{
    m_currentModel = 0;
    m_groundHeight = -2500.f;
}

ObjScene::~ObjScene(void) {}

void ObjScene::doInitialize()
{
    const float s = 10000.f;
    const vec4f objectPosition = make_vec4f();
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

    if (m_filename.length() != 0)
    {
        LOG_INFO(1, "Loading " << m_filename);
        OBJReader objectReader;
        CPUBoundingBox aabb;
        CPUBoundingBox inAABB;
        memset(&inAABB, 0, sizeof(CPUBoundingBox));
        vec4f size = objectReader.loadModelFromFile(m_filename, *m_gpuKernel, objectPosition, true, m_objectScale, true,
                                                    RANDOM_MATERIALS_OFFSET, false, true, aabb, false, inAABB);
        m_groundHeight = -size.y / 2.f - sceneInfo.geometryEpsilon;
    }
    else
    {
        Strings extensions;
        extensions.push_back(".obj");
        const Strings fileNames = getFilesFromFolder(std::string(DEFAULT_MEDIA_FOLDER) + "/obj", extensions);
        if (!fileNames.empty())
        {
            m_filename = fileNames[m_currentModel % fileNames.size()];
            OBJReader objectReader;
            CPUBoundingBox aabb;
            CPUBoundingBox inAABB;
            memset(&inAABB, 0, sizeof(CPUBoundingBox));
            vec4f size = objectReader.loadModelFromFile(m_filename, *m_gpuKernel, objectPosition, true, m_objectScale,
                                                        true, 1000, false, true, aabb, false, inAABB);
            m_groundHeight = -size.y / 2.f - sceneInfo.geometryEpsilon * 10.f;
        }
    }
}

void ObjScene::doPostInitialize() {}

void ObjScene::doAnimate() {}

void ObjScene::doAddLights()
{
    // lights
    if (m_gpuKernel->getNbActiveLamps() == 0)
    {
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 8000.f, 8000.f, -8000.f, 10.f, 0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
    }
}
