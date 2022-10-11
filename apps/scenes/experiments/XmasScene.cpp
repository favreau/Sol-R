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

#include "XmasScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif
#include <math.h>

XmasScene::XmasScene(const std::string& name)
    : Scene(name)
{
}

XmasScene::~XmasScene(void) {}

/*
________________________________________________________________________________

Create tree
________________________________________________________________________________
*/
void XmasScene::createTree(int iteration, int boxId, int maxIterations,
                           vec3f center, int material, float interval,
                           float radius)
{
    if (iteration > 0)
    {
        for (int i(0); i < (2 + rand() % 3); ++i)
        {
            vec3f a = center;
            vec3f b = a;
            b.y += interval;

            int box = (iteration < maxIterations) ? boxId : i;

            m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
            m_gpuKernel->setPrimitive(m_nbPrimitives, a.x, a.y, a.z, b.x, b.y,
                                      b.z, radius / 2.f, 0.f, 0.f, material);

            const vec4f angles = make_vec4f(
                ((i == 0) ? 0.5f : 2.f) * (rand() % 100 / 100.f - 0.5f),
                ((i == 0) ? 0.5f : 2.f) * (rand() % 100 / 100.f - 0.5f),
                ((i == 0) ? 0.5f : 2.f) * (rand() % 100 / 100.f - 0.5f));
            const vec3f cosAngles =
                make_vec3f(cosf(angles.x), cosf(angles.y), cosf(angles.z));
            const vec3f sinAngles =
                make_vec3f(sinf(angles.x), sinf(angles.y), sinf(angles.z));
            CPUPrimitive* p = m_gpuKernel->getPrimitive(m_nbPrimitives);
            m_gpuKernel->rotatePrimitive(*p, a, cosAngles, sinAngles);
            b.y += interval;
            m_gpuKernel->getPrimitiveOtherCenter(m_nbPrimitives, b);

            m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
            m_gpuKernel->setPrimitive(m_nbPrimitives, b.x, b.y, b.z,
                                      radius / 2.f, 0.f, 0.f, material);

            if (iteration == 1 && rand() % 3 == 0)
            {
                // Boule de noel
                m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
                m_gpuKernel->setPrimitive(m_nbPrimitives, a.x, a.y, a.z, a.x,
                                          a.y - 300.f, a.z, 5, 0.f, 0.f, 1016);
                m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
                m_gpuKernel->setPrimitive(m_nbPrimitives, a.x, a.y - 400.f, a.z,
                                          200, 0.f, 0.f, 1010 + rand() % 6);
            }

            if (iteration == 1 && rand() % 3 == 0)
            {
                // Leaves
                m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
                m_gpuKernel->setPrimitive(m_nbPrimitives, b.x, b.y, b.z,
                                          50.f + rand() % 500, 0.f, 0.f, 1002);
            }
            createTree(iteration - 1, box, maxIterations, b, material,
                       interval * (0.8f + rand() % iteration / 10.f),
                       radius * (0.4f + rand() % iteration / 10.f));
        }
    }
}

void XmasScene::doInitialize()
{
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptYZPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -4200.f, 0.f, 0.f, 0.f, 2000.f,
                              200.f, 1000);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXYPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -4100.f, 0.f, 200.f, 100.f,
                              2000.f, 0.f, 1000);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXYPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -4100.f, 0.f, -200.f, 100.f,
                              2000.f, 0.f, 1000);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXZPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -4100.f, 2000.f, 0.f, 100.f, 0.f,
                              200.f, 1000);

    m_nbPrimitives = m_gpuKernel->addPrimitive(ptYZPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 4200.f, 0.f, 0.f, 0.f, 2000.f,
                              200.f, 1000);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXYPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 4100.f, 0.f, 200.f, 100.f, 2000.f,
                              0.f, 1000);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXYPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 4100.f, 0.f, -200.f, 100.f,
                              2000.f, 0.f, 1000);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXZPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 4100.f, 2000.f, 0.f, 100.f, 0.f,
                              200.f, 1000);

    int material = 1001;

    // Tree
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, m_groundHeight, 0.f, 0.f,
                              -1000.f, 0.f, 250.f, 0.f, 0.f, material);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, -1000.f, 0.f, 250.f, 0.f,
                              0.f, material);
    int nbIterations = 2 + rand() % 3;
    vec3f center = make_vec3f(0.f, -1000.f, 0.f);
    createTree(nbIterations, 10, nbIterations, center, material,
               1000.f, //+rand()%800,
               200.f   //+rand()%300
    );
}

void XmasScene::doAnimate() {}

void XmasScene::doAddLights()
{
    // lights
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -4000.f, 0.f, 0.f, 0.f, 2000.f,
                              200.f, DEFAULT_LIGHT_MATERIAL);
}
