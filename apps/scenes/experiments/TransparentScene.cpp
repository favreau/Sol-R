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

#include "TransparentScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

TransparentScene::TransparentScene(const std::string& name)
    : Scene(name)
{
}

TransparentScene::~TransparentScene(void) {}

void TransparentScene::doInitialize()
{
    // Caustic material
    int caustic = 0;
    m_gpuKernel->setMaterial(caustic, 1.f, 1.f, 0.f, 0.f, 0.f, 1.33f, false,
                             false, 0, 0.8f,
                             m_gpuKernel->getSceneInfo().viewDistance,
                             TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
                             TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
                             TEXTURE_NONE, 0.f, 100.f, 0.f, 0.f, 10.f, 10000.f,
                             false);

    vec3f center;
    center.x = 3000.f;
    center.y = m_groundHeight + 2100.f;
    center.z = -1500.f;
    createWorm(center, 8, rand() % 10 + 30);

    center.x = 0.f;
    center.y = m_groundHeight + 2100.f;
    center.z = 0.f;
    createWorm(center, 9, rand() % 10 + 40);

    center.x = -3000.f;
    center.y = m_groundHeight + 2100.f;
    center.z = -1000.f;
    createWorm(center, 10, rand() % 10 + 50);

    center.x = -3000.f;
    center.y = m_groundHeight + 2500.f;
    center.z = -5000.f;
    createDog(center, rand() % 20 + 30, 500.f, 9);
    m_gpuKernel->getSceneInfo().nbRayIterations = 20;
}

void TransparentScene::doAnimate()
{
    m_rotationAngles.y = 0.05f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void TransparentScene::doAddLights()
{
    // lights
    if (m_gpuKernel->getNbActiveLamps() == 0)
    {
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 8000.f, 8000.f, -8000.f,
                                  500.f, 0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    }
}
