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

#include "CornellBoxScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

CornellBoxScene::CornellBoxScene(const std::string& name)
    : Scene(name)
{
}

CornellBoxScene::~CornellBoxScene(void) {}

void CornellBoxScene::doInitialize()
{
    m_groundHeight = -5000.f;
    // Spheres
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 2200.f, 0.f, 0.f, 2000.f, 0.f,
                              0.f, rand() % 50);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -2200.f, 0.f, 0.f, 2000.f, 0.f,
                              0.f, rand() % 50);

    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, 2200.f, 0.f, 2000.f, 0.f,
                              0.f, rand() % 50);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, -2200.f, 0.f, 2000.f, 0.f,
                              0.f, rand() % 50);

    m_gpuKernel->getSceneInfo().nbRayIterations = 20;
}

void CornellBoxScene::doAnimate()
{
    m_rotationAngles.x = 0.02f;
    m_rotationAngles.y = 0.01f;
    m_rotationAngles.z = 0.015f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void CornellBoxScene::doAddLights()
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
