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

#include "GalleryScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <math.h>
#include <stdlib.h>
#endif // WIN32

#include <io/FileMarshaller.h>

GalleryScene::GalleryScene(const std::string& name)
    : Scene(name)
{
    m_groundHeight = -2500.f;
}

GalleryScene::~GalleryScene(void) {}

void GalleryScene::doInitialize()
{
    // Load photos
    Strings filters;
    filters.push_back(".bmp");
    filters.push_back(".jpg");
    loadTextures(std::string(DEFAULT_MEDIA_FOLDER) + "/photos", filters);

    // Materials
    createRandomMaterials(true, false);

    // Gallery
    const int nbRand(9);
    const vec2f step = make_vec2f(2.f * PI / 6, PI / 2);
    const vec4f position = make_vec4f();
    const vec4f size = make_vec4f(18000.f, 18000.f, 18000.f);
    const vec4f frame = make_vec4f(0.05f, 0.05f, 1.f);
    m_groundHeight = -PI / 4 * size.y;

    int i = 0;
    for (float y(-PI / 4.f); y < PI / 4.f; y += step.y)
    {
        for (float x(-PI); x < PI; x += step.x)
        {
            int m(60 + i % nbRand);

            vec4f p0, p1, p2, p3;
            p0.x = cos(x + frame.x); // cos(y)*cos(x);
            p0.y = y + frame.y;      // sin(y);
            p0.z = sin(x + frame.x); // cos(y)*sin(x);

            p1.x = cos(x + step.x - frame.x);
            p1.y = y + frame.y; // sin(y);
            p1.z = sin(x + step.x - frame.x);

            p2.x = cos(x + step.x - frame.x);
            p2.y = y + step.y - frame.y; // sin(y+step.y);
            p2.z = sin(x + step.x - frame.x);

            p3.x = cos(x + frame.x);
            p3.y = y + step.y - frame.y; // sin(y+step.y);
            p3.z = sin(x + frame.x);

            const vec3f normal0 =
                make_vec3f(position.x - p0.x, position.y - p0.y,
                           position.z - p0.z);
            const vec3f normal1 =
                make_vec3f(position.x - p1.x, position.y - p1.y,
                           position.z - p1.z);
            const vec3f normal2 =
                make_vec3f(position.x - p2.x, position.y - p2.y,
                           position.z - p2.z);
            const vec3f normal3 =
                make_vec3f(position.x - p3.x, position.y - p3.y,
                           position.z - p3.z);
            m_nbPrimitives = m_gpuKernel->addPrimitive(ptTriangle);
            m_gpuKernel->setPrimitive(
                m_nbPrimitives, position.x + p0.x * size.x,
                position.y + p0.y * size.y, position.z + p0.z * size.z,
                position.x + p1.x * size.x, position.y + p1.y * size.y,
                position.z + p1.z * size.z, position.x + p2.x * size.x,
                position.y + p2.y * size.y, position.z + p2.z * size.z, 0.f,
                0.f, 0.f, m);
            m_gpuKernel->setPrimitiveNormals(m_nbPrimitives, normal0, normal1,
                                             normal2);

            {
                const vec2f tc0 = make_vec2f(1.f, 1.f);
                const vec2f tc1 = make_vec2f(0.f, 1.f);
                const vec2f tc2 = make_vec2f(0.f, 0.f);
                m_gpuKernel->setPrimitiveTextureCoordinates(m_nbPrimitives, tc0,
                                                            tc1, tc2);
            }

            m_nbPrimitives = m_gpuKernel->addPrimitive(ptTriangle);
            m_gpuKernel->setPrimitive(
                m_nbPrimitives, position.x + p2.x * size.x,
                position.y + p2.y * size.y, position.z + p2.z * size.z,
                position.x + p3.x * size.x, position.y + p3.y * size.y,
                position.z + p3.z * size.z, position.x + p0.x * size.x,
                position.y + p0.y * size.y, position.z + p0.z * size.z, 0.f,
                0.f, 0.f, m);
            m_gpuKernel->setPrimitiveNormals(m_nbPrimitives, normal2, normal3,
                                             normal0);
            {
                const vec2f tc0 = make_vec2f(0.f, 0.f);
                const vec2f tc1 = make_vec2f(1.f, 0.f);
                const vec2f tc2 = make_vec2f(1.f, 1.f);
                m_gpuKernel->setPrimitiveTextureCoordinates(m_nbPrimitives, tc0,
                                                            tc1, tc2);
            }
            ++i;
        }
    }
    for (int i(0); i < 40; ++i)
    {
        m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, rand() % 10000 - 5000.f,
                                  rand() % 10000 - 5000.f,
                                  rand() % 10000 - 5000.f, 200.f + rand() % 200,
                                  0.f, 0.f, 41 + rand() % 2);
    }
}

void GalleryScene::doAnimate()
{
    m_rotationAngles.y = 0.005f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void GalleryScene::doAddLights()
{
    // Lights
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, 5000.f, 0.f, 0, 0, 0,
                              DEFAULT_LIGHT_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
}
