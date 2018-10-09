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

#include "CubesScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

CubesScene::CubesScene(const std::string& name)
    : Scene(name)
{
}

CubesScene::~CubesScene(void) {}

void CubesScene::doInitialize()
{
    int s = 5000;
    for (int i(0); i < 10; ++i)
    {
        float X = static_cast<float>(rand() % (2 * s) - s);
        float Y = static_cast<float>(rand() % (2 * s)) - 2500.f;
        float Z = static_cast<float>(rand() % (2 * s) - s);
        for (int j(0); j < 10; j++)
        {
            float x = 0.3f * static_cast<float>(rand() % (2 * s) - s);
            float y = 0.3f * static_cast<float>(rand() % (2 * s));
            float z = 0.3f * static_cast<float>(rand() % (2 * s) - s);
            float r = static_cast<float>(500 + rand() % 500);
            m_gpuKernel->addCube(X + x, Y + y, Z + z, r, 1);
        }
    }
    m_gpuKernel->getSceneInfo().nbRayIterations = 20;
}

void CubesScene::doAnimate() {}

void CubesScene::doAddLights()
{
    // lights
    if (m_gpuKernel->getNbActiveLamps() == 0)
    {
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, 8000.f, 8000.f, -8000.f, 500.f, 0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    }
}
