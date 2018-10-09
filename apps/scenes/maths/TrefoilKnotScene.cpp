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

#include "TrefoilKnotScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

#include <math.h>

TrefoilKnotScene::TrefoilKnotScene(const std::string& name)
    : Scene(name)
{
}

TrefoilKnotScene::~TrefoilKnotScene(void) {}

void TrefoilKnotScene::trefoilKnot(float R, float t, vec4f& p)
{
    p.x = R * (sin(t) + 2.f * sin(2.f * t));
    p.y = R * (cos(t) - 2.f * cos(2.f * t));
    p.z = R * (-sin(3.f * t));
}

void TrefoilKnotScene::torus(float R, float t, vec4f& p)
{
    p.x = R * (3.f * cos(t) + cos(10.f * t) * cos(t));
    p.y = R * (3.f * sin(t) + cos(10.f * t) * sin(t));
    p.z = R * sin(10.f * t);
}

void TrefoilKnotScene::star(float R, float t, vec4f& p)
{
    p.x = R * (2.f * sin(3.f * t) * cos(t));
    p.y = R * (2.f * sin(3.f * t) * sin(t));
    p.z = R * sin(3.f * t);
}

void TrefoilKnotScene::spring(float R, float t, vec4f& p)
{
    p.x = R * cos(t);
    p.y = R * sin(t);
    p.z = R * cos(t);
}

void TrefoilKnotScene::heart(float R, float u, float v, vec4f& p)
{
    p.x = R * 4.f * pow(sin(u), 3.f);
    p.y = R * 0.25f * (13 * cos(u) - 5 * cos(2.f * u) - 2.f * cos(3.f * u) - cos(4.f * u));
    p.z = 0.f;
}

void TrefoilKnotScene::thing(float R, float t, vec4f a, vec4f& p)
{
    p.x = R * (sin(t) + a.x * sin(a.y * t));
    p.y = R * (cos(t) - a.x * cos(a.y * t));
    p.z = R * (-sin(a.z * t));
}

void TrefoilKnotScene::moebius(float R, float u, float v, float s, float du, float dv, vec4f& p)
{
    p.x = 4.f * R * (cos(u) + v * cos(u / 2) * cos(u));
    p.y = 4.f * R * (sin(u) + v * cos(u / 2) * sin(u));
    p.z = 8.f * R * (v * sin(u / 2));
}

void TrefoilKnotScene::doInitialize()
{
    m_groundHeight = -2500.f;

    // Knot
    const int material = 30 + rand() % 20;
    int element = 0;
    const float R = 500.f;
    const float r = 50.f + rand() % 800;
    const int geometry = rand() % 10;
    const vec4f a = make_vec4f(1.f + rand() % 4, 1.f + rand() % 4, 1.f + rand() % 4, 1.f + rand() % 4);
    const float s = 1.f;

    vec4f U, V;
    // Moebius
    switch (geometry)
    {
    case 0: // Moebuis
    {
        U.x = 2 * static_cast<float>(M_PI) * s;
        U.y = (2 + 2 * s) * static_cast<float>(M_PI);
        U.z = 90.f;
        V.x = -0.4f;
        V.y = 0.4f;
        V.z = 1.f;
    }
    break;
    case 5: // Heart
    {
        U.x = 0.f;
        U.y = 2.f * static_cast<float>(M_PI) * s;
        U.z = 90.f;
        V.x = 0.f;
        V.y = 1.f;
        V.z = 1.f;
    }
    break;
    default:
    {
        U.x = 0.f;
        U.y = 2.f * static_cast<float>(M_PI) * s;
        U.z = 90.f;
        V.x = 0.f;
        V.y = 1.f;
        V.z = 1.f;
    }
    break;
    }

    U.w = (U.y - U.x) / U.z;
    V.w = (V.y - V.x) / V.z;

    int M(0);
    for (float v(V.x); v < V.y; v += V.w)
    {
        for (float u(U.x); u < U.y; u += U.w)
        {
            vec4f p0, p1, p2, p3;
            switch (geometry)
            {
            case 0:
            {
                moebius(R, u, v, s, U.w, V.w, p0);
                moebius(R, u + U.w, v, s, U.w, V.w, p1);
                moebius(R, u + U.w, v + V.w, s, U.w, V.w, p2);
                moebius(R, u, v + V.w, s, U.w, V.w, p3);
            }
            break;
            case 1:
            {
                torus(R, u, p0);
                torus(R, u + U.w, p1);
                p3 = p0;
                p3.z += R;
                p2 = p1;
                p2.z += R;
            }
            break;
            case 2:
            {
                star(R, u, p0);
                star(R, u + U.w, p1);
                p3 = p0;
                p3.z += R;
                p2 = p1;
                p2.z += R;
            }
            break;
            case 3:
            {
                spring(R, u, p0);
                spring(R, u + U.w, p1);
                p3 = p0;
                p3.z += R;
                p2 = p1;
                p2.z += R;
            }
            break;
            case 4:
            {
                trefoilKnot(R, u, p0);
                trefoilKnot(R, u + U.w, p1);
                p3 = p0;
                p3.z += R;
                p2 = p1;
                p2.z += R;
            }
            break;
            case 5:
            {
                heart(R, u, v, p0);
                heart(R, u + U.w, v, p1);
                p3 = p0;
                p3.z += R * 5.f;
                p2 = p1;
                p2.z += R * 5.f;
            }
            break;
            default:
            {
                thing(R, u, a, p0);
                thing(R, u + U.w, a, p1);
                p3 = p0;
                p3.z += R;
                p2 = p1;
                p2.z += R;
            }
            break;
            }
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptCylinder);
            m_gpuKernel->setPrimitive(m_nbPrimitives, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, r, 0.f, 0.f, material + M);
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
            m_gpuKernel->setPrimitive(m_nbPrimitives, p1.x, p1.y, p1.z, r, 0.f, 0.f, material + M);
            ++element;
        }
    }
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -4000.f, -1000.f, 1000.f, 1500.f, 0.f, 0.f, 114);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 4000.f, -1000.f, 1000.f, 1500.f, 0.f, 0.f, 113);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
    m_gpuKernel->getSceneInfo().nbRayIterations = 20;
}

void TrefoilKnotScene::doAnimate()
{
    m_rotationAngles.y = 0.1f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void TrefoilKnotScene::doAddLights()
{
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, 5000.f, 0.f, 50.f, 0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
}
