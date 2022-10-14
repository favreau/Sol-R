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

#include "TesseractScene.h"

#include <opengl/rtgl.h>

TesseractScene::TesseractScene(const std::string& name)
    : Scene(name)
    , _timestamp(0.f)
{
}

TesseractScene::~TesseractScene() {}

vec3f TesseractScene::computeCoordinates(const vec3f& p1, const vec3f& p2,
                                         const vec3f& p3, const vec3f& p4)
{
    vec3f newVec;

    if (_timestamp >= 0.f && _timestamp < 0.25f)
    {
        newVec.x = p4.x + (p1.x - p4.x) * _timestamp * 4.f;
        newVec.y = p4.y + (p1.y - p4.y) * _timestamp * 4.f;
        newVec.z = p4.z + (p1.z - p4.z) * _timestamp * 4.f;
    }
    else if (_timestamp >= 0.25f && _timestamp < 0.5f)
    {
        newVec.x = p1.x + (p2.x - p1.x) * (_timestamp - 0.25f) * 4.f;
        newVec.y = p1.y + (p2.y - p1.y) * (_timestamp - 0.25f) * 4.f;
        newVec.z = p1.z + (p2.z - p1.z) * (_timestamp - 0.25f) * 4.f;
    }
    else if (_timestamp >= 0.5f && _timestamp < 0.75f)
    {
        newVec.x = p2.x + (p3.x - p2.x) * (_timestamp - 0.5f) * 4.f;
        newVec.y = p2.y + (p3.y - p2.y) * (_timestamp - 0.5f) * 4.f;
        newVec.z = p2.z + (p3.z - p2.z) * (_timestamp - 0.5f) * 4.f;
    }
    else if (_timestamp >= 0.75f && _timestamp < 1.f)
    {
        newVec.x = p3.x + (p4.x - p3.x) * (_timestamp - 0.75f) * 4.f;
        newVec.y = p3.y + (p4.y - p3.y) * (_timestamp - 0.75f) * 4.f;
        newVec.z = p3.z + (p4.z - p3.z) * (_timestamp - 0.75f) * 4.f;
    }

    return newVec;
}

void TesseractScene::createGeometry()
{
    vec3f tesseract[16] = {
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5},
        {-0.5, 0.5, -0.5},  {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5},
        {0.5, 0.5, 0.5},    {-0.5, 0.5, 0.5},  {-1, -1, -1},
        {1, -1, -1},        {1, 1, -1},        {-1, 1, -1},
        {-1, -1, 1},        {1, -1, 1},        {1, 1, 1},
        {-1, 1, 1},
    };

    vec3f vertices[16] = {computeCoordinates(tesseract[8], tesseract[9],
                                             tesseract[1], tesseract[0]),
                          computeCoordinates(tesseract[0], tesseract[8],
                                             tesseract[9], tesseract[1]),
                          computeCoordinates(tesseract[3], tesseract[11],
                                             tesseract[10], tesseract[2]),
                          computeCoordinates(tesseract[11], tesseract[10],
                                             tesseract[2], tesseract[3]),
                          computeCoordinates(tesseract[12], tesseract[13],
                                             tesseract[5], tesseract[4]),
                          computeCoordinates(tesseract[4], tesseract[12],
                                             tesseract[13], tesseract[5]),
                          computeCoordinates(tesseract[7], tesseract[15],
                                             tesseract[14], tesseract[6]),
                          computeCoordinates(tesseract[15], tesseract[14],
                                             tesseract[6], tesseract[7]),
                          computeCoordinates(tesseract[9], tesseract[1],
                                             tesseract[0], tesseract[8]),
                          computeCoordinates(tesseract[1], tesseract[0],
                                             tesseract[8], tesseract[9]),
                          computeCoordinates(tesseract[2], tesseract[3],
                                             tesseract[11], tesseract[10]),
                          computeCoordinates(tesseract[10], tesseract[2],
                                             tesseract[3], tesseract[11]),
                          computeCoordinates(tesseract[13], tesseract[5],
                                             tesseract[4], tesseract[12]),
                          computeCoordinates(tesseract[5], tesseract[4],
                                             tesseract[12], tesseract[13]),
                          computeCoordinates(tesseract[6], tesseract[7],
                                             tesseract[15], tesseract[14]),
                          computeCoordinates(tesseract[14], tesseract[6],
                                             tesseract[7], tesseract[15])};

    const float s = 2000.f;
    float radius = s / 10.f;
    for (size_t i = 0; i < 16; ++i)
    {
        vec3f t = vertices[i];
        size_t primitiveIndex = m_gpuKernel->addPrimitive(ptSphere);
        m_gpuKernel->setPrimitive(primitiveIndex, s * t.x, s * t.y, s * t.z,
                                  radius, 0.f, 0.f, METAL_MATERIAL);
    }

    const size_t joints[32][2] = {
        {0, 1},   {1, 2},   {2, 3},   {3, 0},   {0, 4},  {1, 5},   {2, 6},
        {3, 7},   {4, 5},   {5, 6},   {6, 7},   {7, 4},  {0, 8},   {1, 9},
        {2, 10},  {3, 11},  {4, 12},  {5, 13},  {6, 14}, {7, 15},  {8, 9},
        {9, 10},  {10, 11}, {11, 8},  {8, 12},  {9, 13}, {10, 14}, {11, 15},
        {12, 13}, {13, 14}, {14, 15}, {15, 12},
    };

    radius = s / 20.f;
    for (size_t i = 0; i < 32; ++i)
    {
        vec3f t = vertices[joints[i][0]];
        vec3f u = vertices[joints[i][1]];

        size_t primitiveIndex = m_gpuKernel->addPrimitive(ptCylinder);
        m_gpuKernel->setPrimitive(primitiveIndex, s * t.x, s * t.y, s * t.z,
                                  s * u.x, s * u.y, s * u.z, radius, 0.f, 0.f,
                                  METAL_MATERIAL);
    }

    size_t material = BASIC_REFLECTION_MATERIAL_002;
    for (size_t i = 0; i < 32; i += 4)
    {
        vec3f t = vertices[joints[i][0]];
        vec3f u = vertices[joints[i + 1][0]];
        vec3f v = vertices[joints[i + 2][0]];
        vec3f w = vertices[joints[i + 3][0]];

        m_gpuKernel->setCurrentMaterial(material);
        glBegin(GL_TRIANGLES);
        glVertex3f(s * t.x, s * t.y, s * t.z);
        glVertex3f(s * u.x, s * u.y, s * u.z);
        glVertex3f(s * v.x, s * v.y, s * v.z);
        glEnd();

        glBegin(GL_TRIANGLES);
        glVertex3f(s * v.x, s * v.y, s * v.z);
        glVertex3f(s * w.x, s * w.y, s * w.z);
        glVertex3f(s * t.x, s * t.y, s * t.z);
        glEnd();
    }
}

void TesseractScene::doInitialize()
{
    createGeometry();
    doAddLights();
}

void TesseractScene::doAnimate()
{
    m_gpuKernel->resetFrame();
    createGeometry();
    doAddLights();
    addCornellBox(m_cornellBoxType);
    m_gpuKernel->compactBoxes(true);
    _timestamp += 0.005f;
    if (_timestamp > 1.f)
        _timestamp = 0.f;
}

void TesseractScene::doAddLights()
{
    // lights
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, 0.f, 10000.f, -10000.f, 1000.f,
                              0.f, 0, DEFAULT_LIGHT_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
}
