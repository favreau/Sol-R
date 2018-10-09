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

#include "SwcScene.h"

#include <common/Utils.h>
#include <io/FileMarshaller.h>

SwcScene::SwcScene(const std::string &name)
    : Scene(name)
    , m_counter(0)
{
}

SwcScene::~SwcScene() {}

/*
________________________________________________________________________________

Create simple scene for box validation
________________________________________________________________________________
*/
void SwcScene::doInitialize()
{
    // initialization
    Strings extensions;
    extensions.push_back(".swc");
    const Strings fileNames = getFilesFromFolder(std::string(DEFAULT_MEDIA_FOLDER) + "/swc", extensions);
    if (fileNames.size() != 0)
    {
        m_currentModel = m_currentModel % fileNames.size();
        const vec4f scale = make_vec4f(10.f, 10.f, 10.f, 10.f);

        // Scene
        const vec4f position = make_vec4f(0.f, 0.f, 0.f);
        for (int i(0); i < fileNames.size(); ++i)
        {
            SWCReader swcReader;
            swcReader.loadMorphologyFromFile(fileNames[i], *m_gpuKernel, position, scale, 1001);

            if (i == 0)
                m_morphologies = swcReader.getMorphologies();
        }
    }
}

void SwcScene::doAnimate()
{
    const int index = m_counter % m_morphologies.size();
    Morphology &m = m_morphologies[index];
    if (m_counter > 0)
        m_gpuKernel->setPrimitiveMaterial(m_previousPrimitiveId, m_previousMaterial);

    m_previousMaterial = m_gpuKernel->getPrimitiveMaterial(m.primitiveId);
    m_gpuKernel->setPrimitiveMaterial(m.primitiveId, DEFAULT_LIGHT_MATERIAL);
    m_previousPrimitiveId = m.primitiveId;

    const int light = m_gpuKernel->getLight(0);
    if (light != -1)
    {
        CPUPrimitive *lamp = m_gpuKernel->getPrimitive(light);
        lamp->p0.x = m.x;
        lamp->p0.y = m.y;
        lamp->p0.z = m.z - 50.f;
        m_gpuKernel->setPrimitiveCenter(light, lamp->p0);
    }

    m_gpuKernel->compactBoxes(false);
    m_counter += 1;
}

void SwcScene::doAddLights()
{
    // lights
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -10000.f, 10000.f, -10000.f, 20.f, 0.f, 0, DEFAULT_LIGHT_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
}
