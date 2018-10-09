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

#include "DoggyStyleScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

#include <math.h>

DoggyStyleScene::DoggyStyleScene(const std::string& name)
    : Scene(name)
{
}

DoggyStyleScene::~DoggyStyleScene(void) {}

void DoggyStyleScene::doInitialize()
{
    vec3f center = make_vec3f(-3500.f, m_groundHeight + 2500.f, 0.f);
    createDog(center, rand() % 20 + 30, 500.f, 1010);
    center.x = 3500.f;
    createDog(center, rand() % 20 + 30, 500.f, 1011);
    vec4f angles = make_vec4f(0.f, static_cast<float>(M_PI), 0.f);
    m_gpuKernel->rotatePrimitives(center, angles);
}

void DoggyStyleScene::doAnimate() {}

void DoggyStyleScene::doAddLights()
{
    // Laser
    for (int i = 0; i < 3; ++i)
    {
        const vec4f p1 = make_vec4f(rand() % 10000 - 5000.f, 5000.f, rand() % 10000 - 5000.f);
        const vec4f p2 = make_vec4f(-(rand() % 10000 - 5000.f), -5000.f, -(rand() % 10000 - 5000.f));
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
        m_gpuKernel->setPrimitive(m_nbPrimitives, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, 100.f, 100.f, 100.f,
                                  DEFAULT_LIGHT_MATERIAL - i);
    }
}
