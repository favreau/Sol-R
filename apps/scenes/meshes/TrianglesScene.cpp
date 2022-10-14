/*
 * Copyright (c) 2011-2022, Cyrille Favreau
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "TrianglesScene.h"

#include <common/Utils.h>
#include <io/FileMarshaller.h>
#include <io/OBJReader.h>
#include <io/PDBReader.h>
#include <math.h>

TrianglesScene::TrianglesScene(const std::string& name)
    : Scene(name)
    , m_frameIndex(0)
{
    m_currentModel = 0;
    m_groundHeight = -2500.f;
}

TrianglesScene::~TrianglesScene(void) {}

void TrianglesScene::doInitialize()
{
    Strings extensions;
    extensions.push_back(".irt");
    const Strings fileNames =
        getFilesFromFolder(std::string(DEFAULT_MEDIA_FOLDER) + "/irt",
                           extensions);
    if (fileNames.size() != 0)
    {
        const vec4f center = make_vec4f();
        const float objectScale = 1.f;
        m_currentModel = m_currentModel % fileNames.size();
        FileMarshaller fm;
        fm.loadFromFile(*m_gpuKernel, fileNames[m_currentModel], center,
                        objectScale * 5000.f);
        m_groundHeight = -2500.f;
    }
}

void TrianglesScene::doAnimate()
{
    const int nbFrames = 120;
    m_rotationAngles.y = static_cast<float>(-2.f * M_PI / nbFrames);
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void TrianglesScene::doAddLights()
{
    // lights
    if (m_gpuKernel->getNbActiveLamps() == 0)
    {
        LOG_INFO(3, "Adding sun light");
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 5000.f, 5000.f, -5000.f, 10.f,
                                  0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    }
}
