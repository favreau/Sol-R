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

#include "WaterScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <math.h>
#endif

#include <io/PDBReader.h>

WaterScene::WaterScene(const std::string& name)
    : Scene(name)
{
}

WaterScene::~WaterScene(void) {}

vec3f WaterScene::F(float x, float z, float stepX, float stepZ)
{
#ifdef WIN32
    float timer = static_cast<float>(GetTickCount()) / 1000.f;
#else
    float timer = m_gpuKernel->getSceneInfo().timestamp;
#endif
    vec3f returnValue;
    float t = x;
    float u = z;

    // Torus
    returnValue.x = cos(t) * (m_scale.x + m_scale.y * cos(u));
    returnValue.y = m_scale.y * (sin(u + timer) + cos(t + 2.f * timer));
    returnValue.z = sin(t) * (m_scale.z + m_scale.y * cos(u));

    return returnValue;
}

vec3f WaterScene::P(float t, float stepX, float stepZ)
{
    vec3f returnValue;
    float tx = t + stepX;
    float tz = t + stepZ;

    // Helix
    returnValue.x = sin(2.f * tx);
    returnValue.y = cos(tz);
    returnValue.z = tx;

    returnValue.x *= m_scale.x;
    returnValue.y *= m_scale.y;
    returnValue.z *= m_scale.z;
    return returnValue;
}

void WaterScene::processCurve(bool update)
{
    float step = static_cast<float>(M_PI) / 12.f;
    float tstep = step * 2.f;
    int primitiveIndex(m_initialIndex - 1);
    float tx = 0.f;
    for (float x = -static_cast<float>(M_PI);
         x < static_cast<float>(M_PI) - step / 2.f; x += step)
    {
        float ty = 0.f;
        for (float z = -static_cast<float>(M_PI);
             z < static_cast<float>(M_PI) - step / 2.f; z += step)
        {
            // Face
            int index(0);
            vec3f vertices[4];
            vec3f verticesNormals[4];
            for (float X(0); X <= step; X += step)
            {
                for (float Z(0); Z <= step; Z += step)
                {
                    // Vertices
                    vertices[index] = F(x + X, z + Z, X, Z);

                    // Normal
                    const vec3f P1p = F(x + X + step, z + Z, X / 2.f, Z / 2.f);
                    const vec3f P3p = F(x + X, z + Z + step, X / 2.f, Z / 2.f);
                    vec3f v1 = make_vec3f(P1p.x - vertices[index].x,
                                          P1p.y - vertices[index].y,
                                          P1p.z - vertices[index].z);
                    vec3f v3 = make_vec3f(P3p.x - vertices[index].x,
                                          P3p.y - vertices[index].y,
                                          P3p.z - vertices[index].z);
                    m_gpuKernel->normalizeVector(v1);
                    m_gpuKernel->normalizeVector(v3);
                    verticesNormals[index] = m_gpuKernel->crossProduct(v1, v3);

                    index++;
                }
            }

            if (update)
                primitiveIndex++;
            else
            {
                primitiveIndex = m_gpuKernel->addPrimitive(ptTriangle);
                m_gpuKernel->setPrimitiveIsMovable(primitiveIndex, false);
                if (m_initialIndex == -1)
                    m_initialIndex = primitiveIndex;
            }
            m_gpuKernel->setPrimitive(
                primitiveIndex, m_objectSize.x * vertices[0].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[0].y,
                m_objectSize.z * vertices[0].z, m_objectSize.x * vertices[2].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[2].y,
                m_objectSize.z * vertices[2].z, m_objectSize.x * vertices[3].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[3].y,
                m_objectSize.z * vertices[3].z, 0.f, 0.f, 0.f, m_material);
            m_gpuKernel->setPrimitiveNormals(primitiveIndex, verticesNormals[0],
                                             verticesNormals[2],
                                             verticesNormals[3]);

            {
                const vec2f tc0 = make_vec2f(tx, ty);
                const vec2f tc1 = make_vec2f(tx + tstep, ty);
                const vec2f tc2 = make_vec2f(tx + tstep, ty + tstep);
                m_gpuKernel->setPrimitiveTextureCoordinates(primitiveIndex, tc0,
                                                            tc1, tc2);
            }

            if (update)
                primitiveIndex++;
            else
            {
                primitiveIndex = m_gpuKernel->addPrimitive(ptTriangle);
                m_gpuKernel->setPrimitiveIsMovable(primitiveIndex, false);
            }
            m_gpuKernel->setPrimitive(
                primitiveIndex, m_objectSize.x * vertices[3].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[3].y,
                m_objectSize.z * vertices[3].z, m_objectSize.x * vertices[1].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[1].y,
                m_objectSize.z * vertices[1].z, m_objectSize.x * vertices[0].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[0].y,
                m_objectSize.z * vertices[0].z, 0.f, 0.f, 0.f, m_material);
            m_gpuKernel->setPrimitiveNormals(primitiveIndex, verticesNormals[3],
                                             verticesNormals[1],
                                             verticesNormals[0]);

            {
                const vec2f tc0 = make_vec2f(tx + tstep, ty + tstep);
                const vec2f tc1 = make_vec2f(tx, ty + tstep);
                const vec2f tc2 = make_vec2f(tx, ty);
                m_gpuKernel->setPrimitiveTextureCoordinates(primitiveIndex, tc0,
                                                            tc1, tc2);
            }
            ty += tstep;
        }
        tx += tstep;
    }
}

void WaterScene::processParametricCurve(bool update)
{
    float tmin = 0.f;
    float tmax = 4.f * static_cast<float>(M_PI);
    float step = (tmax - tmin) / 20.f;
    int primitiveIndex(m_initialIndex - 1);
    for (float t = tmin; t < tmax; t += step)
    {
        // Face
        int index(0);
        vec3f vertices[4];
        vec3f verticesNormals[4];
        for (float X(0); X <= step; X += step)
        {
            for (float Z(0); Z <= step; Z += step)
            {
                // Vertices
                vertices[index] = P(t, X, Z);

                // Normal
                const vec3f P1p = P(t, X / 2.f, Z / 2.f);
                const vec3f P3p = P(t, X / 2.f, Z / 2.f);
                vec3f v1 = make_vec3f(P1p.x - vertices[index].x,
                                      P1p.y - vertices[index].y,
                                      P1p.z - vertices[index].z);
                vec3f v3 = make_vec3f(P3p.x - vertices[index].x,
                                      P3p.y - vertices[index].y,
                                      P3p.z - vertices[index].z);
                m_gpuKernel->normalizeVector(v1);
                m_gpuKernel->normalizeVector(v3);
                verticesNormals[index] = m_gpuKernel->crossProduct(v1, v3);

                index++;
            }

            if (update)
                primitiveIndex++;
            else
            {
                primitiveIndex = m_gpuKernel->addPrimitive(ptTriangle);
                m_gpuKernel->setPrimitiveIsMovable(primitiveIndex, false);
                if (m_initialIndex == -1)
                    m_initialIndex = primitiveIndex;
            }
            m_gpuKernel->setPrimitive(
                primitiveIndex, m_objectSize.x * vertices[0].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[0].y,
                m_objectSize.z * vertices[0].z, m_objectSize.x * vertices[2].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[2].y,
                m_objectSize.z * vertices[2].z, m_objectSize.x * vertices[3].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[3].y,
                m_objectSize.z * vertices[3].z, 0.f, 0.f, 0.f, m_material);
            m_gpuKernel->setPrimitiveNormals(primitiveIndex, verticesNormals[0],
                                             verticesNormals[2],
                                             verticesNormals[3]);

            if (update)
                primitiveIndex++;
            else
            {
                primitiveIndex = m_gpuKernel->addPrimitive(ptTriangle);
                m_gpuKernel->setPrimitiveIsMovable(primitiveIndex, false);
            }
            m_gpuKernel->setPrimitive(
                primitiveIndex, m_objectSize.x * vertices[3].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[3].y,
                m_objectSize.z * vertices[3].z, m_objectSize.x * vertices[1].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[1].y,
                m_objectSize.z * vertices[1].z, m_objectSize.x * vertices[0].x,
                m_groundHeight + 2.f * m_objectSize.y * m_scale.y +
                    m_objectSize.y * vertices[0].y,
                m_objectSize.z * vertices[0].z, 0.f, 0.f, 0.f, m_material);
            m_gpuKernel->setPrimitiveNormals(primitiveIndex, verticesNormals[3],
                                             verticesNormals[1],
                                             verticesNormals[0]);
        }
    }
}

void WaterScene::doInitialize()
{
    // Scene parameters
    m_objectSize.x = fabs(m_groundHeight);
    m_objectSize.y = fabs(m_groundHeight);
    m_objectSize.z = fabs(m_groundHeight);
    m_scale.x = 1.f;
    m_scale.y = 0.5f;
    m_scale.z = 1.f;
    m_initialIndex = -1;
    m_material = 1023;

    processCurve(false);
}

void WaterScene::doAnimate()
{
    processCurve(true);
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void WaterScene::doAddLights()
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
