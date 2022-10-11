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

#include "FractalScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

const int NB_MODELS = 5;
const int NB_ELEMENTS = 8;

FractalScene::FractalScene(const std::string& name)
    : Scene(name)
{
}

FractalScene::~FractalScene(void) {}

/*
________________________________________________________________________________

Create simple fractal scene
________________________________________________________________________________
*/
void FractalScene::createFractals(int iteration, int modelId, int mode,
                                  int maxIterations, vec4f center, int material,
                                  float interval, float radius)
{
    vec4f positions[NB_MODELS][NB_ELEMENTS] = {
        {make_vec4f(-1.f, -1.f, -1.f, 0.f), make_vec4f(1.f, -1.f, -1.f, 0.f),
         make_vec4f(0.f, -1.f, 1.f, 0.f), make_vec4f(0.f, 1.f, 0.f, 0.f),
         make_vec4f(0.f, 0.f, -1.f, 0.f), make_vec4f(0.f, -1.f, 0.f, 0.f),
         make_vec4f(0.f, 0.f, 0.f, 0.f), make_vec4f(0.f, 0.f, 0.f, 0.f)},
        {make_vec4f(-1.f, -1.f, 1.f, 0.f), make_vec4f(1.f, -1.f, 1.f, 0.f),
         make_vec4f(-1.f, 1.f, 1.f, 0.f), make_vec4f(1.f, 1.f, 1.f, 0.f),
         make_vec4f(0.f, 0.f, 0.f, 0.f), make_vec4f(0.f, 0.f, 0.f, 0.f),
         make_vec4f(0.f, 0.f, 0.f, 0.f), make_vec4f(0.f, 0.f, 0.f, 0.f)},
        {// Cube
         make_vec4f(-1.f, -1.f, -1.f, 0.f), make_vec4f(1.f, -1.f, -1.f, 0.f),
         make_vec4f(-1.f, 1.f, -1.f, 0.f), make_vec4f(1.f, 1.f, -1.f, 0.f),
         make_vec4f(-1.f, -1.f, 1.f, 0.f), make_vec4f(1.f, -1.f, 1.f, 0.f),
         make_vec4f(-1.f, 1.f, 1.f, 0.f), make_vec4f(1.f, 1.f, 1.f, 0.f)},
        {make_vec4f(1.f, 0.f, 0.f, 0.f), make_vec4f(0.f, 0.f, 1.f, 0.f),
         make_vec4f(0.f, 2.f, 0.f, 0.f), make_vec4f(-1.f, 0.f, 0.f, 0.f),
         make_vec4f(0.f, 0.f, -1.f, 0.f), make_vec4f(0.f, -1.f, 0.f, 0.f),
         make_vec4f(0.f, 0.f, 0.f, 0.f), make_vec4f(0.f, 0.f, 0.f, 0.f)},
        {make_vec4f(1.f, 1.f, 0.f, 0.f), make_vec4f(-1.f, 1.f, 0.f, 0.f),
         make_vec4f(1.f, -1.f, 0.f, 0.f), make_vec4f(-1.f, -1.f, 0.f, 0.f),
         make_vec4f(0.f, 0.f, -1.f, 0.f), make_vec4f(0.f, 0.f, 1.f, 0.f),
         make_vec4f(0.f, 0.f, 0.f, 0.f), make_vec4f(0.f, 0.f, 0.f, 0.f)}};

    if (iteration > 0)
    {
        for (int i(0); i < NB_ELEMENTS; i++)
        {
            if (positions[modelId][i].x != 0.f ||
                positions[modelId][i].y != 0.f ||
                positions[modelId][i].z != 0.f)
            {
                vec4f c = make_vec4f(0.f, 0.f, 0.f, 0.f);
                c.x = center.x + positions[modelId][i].x * center.w * interval;
                c.y = center.y + positions[modelId][i].y * center.w * interval;
                c.z = center.z + positions[modelId][i].z * center.w * interval;
                c.w = center.w * radius;

                const vec2f vt0 = make_vec2f(0.f, 0.f);
                const vec2f vt1 = make_vec2f(1.f, 1.f);
                const vec2f vt2 = make_vec2f(0.f, 0.f);

                switch (mode)
                {
                case 0:
                {
                    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
                    m_gpuKernel->setPrimitive(
                        m_nbPrimitives, c.x, c.y, c.z,
                        (iteration == 1) ? c.w / 2.f : c.w,
                        (iteration == 2) ? c.w / 2.f : c.w,
                        (iteration == 3) ? c.w / 2.f : c.w, 1000 + iteration);
                    m_gpuKernel->setPrimitiveTextureCoordinates(m_nbPrimitives,
                                                                vt0, vt1, vt2);
                    m_gpuKernel->setPrimitiveBellongsToModel(m_nbPrimitives,
                                                             true);
                }
                break;
                case 1:
                {
                    m_gpuKernel->addCube(
                        c.x, c.y, c.z, (iteration == 1) ? c.w / 2.f : c.w,
                        1000 + ((maxIterations - iteration) * 10 /*+i*/) % 50);
                }
                break;
                default:
                {
                    if (iteration % 2 == 1)
                    {
                        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
                        m_gpuKernel->setPrimitive(
                            m_nbPrimitives, c.x, c.y, c.z,
                            (iteration == 1) ? c.w / 2.f : c.w,
                            (iteration == 2) ? c.w / 2.f : c.w,
                            (iteration == 3) ? c.w / 2.f : c.w,
                            1000 +
                                ((maxIterations - iteration) * 10 /*+i*/) % 50);
                        m_gpuKernel->setPrimitiveTextureCoordinates(
                            m_nbPrimitives, vt0, vt1, vt2);
                        m_gpuKernel->setPrimitiveBellongsToModel(m_nbPrimitives,
                                                                 true);
                    }
                    else
                    {
                        m_gpuKernel->addCube(
                            c.x, c.y, c.z, (iteration == 1) ? c.w / 2.f : c.w,
                            1000 +
                                ((maxIterations - iteration) * 10 /*+i*/) % 50);
                    }
                }
                break;
                }

                createFractals(iteration - 1, modelId, mode, maxIterations, c,
                               material, interval, radius);
            }
        }
    }
}

void FractalScene::doInitialize()
{
    m_groundHeight = -5000.f;
    int nbIterations = 2 + rand() % 3;
    vec4f center = make_vec4f(0.f, 0.f, 0.f, 2000.f);
    createFractals(nbIterations, rand() % NB_MODELS, rand() % 1, nbIterations,
                   center, 1000 + rand() % 30, 1.0f + rand() % 500 / 1000.f,
                   0.5f + rand() % 200 / 1000.f);
}

void FractalScene::doAnimate()
{
    m_rotationAngles.x = 0.02f;
    m_rotationAngles.y = 0.01f;
    m_rotationAngles.z = 0.015f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
    m_gpuKernel->getSceneInfo().timestamp++;
}

void FractalScene::doAddLights()
{
    // lights
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -15000.f, 15000.f, -15000.f, 10.f,
                              0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
}
