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

#include "SpindizzyScene.h"

#include <opengl/rtgl.h>

SpindizzyScene::SpindizzyScene(const std::string& name)
    : Scene(name)
{
}

SpindizzyScene::~SpindizzyScene(void) {}

void SpindizzyScene::doInitialize()
{
    float size = 1000.f;

    glBegin(GL_TRIANGLES);
    glVertex3f(size, size, 0.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord2f(1.f, 1.f);

    glVertex3f(-size, size, 0.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord2f(0.f, 1.f);

    glVertex3f(-size, -size, 0.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord2f(0.f, 0.f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glVertex3f(-size, -size, 0.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord2f(0.f, 0.f);

    glVertex3f(size, -size, 0.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord2f(1.f, 0.f);

    glVertex3f(size, size, 0.f);
    glNormal3f(0.f, 0.f, -1.f);
    glTexCoord2f(1.f, 1.f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glVertex3f(-size, 0.f, -size);
    glNormal3f(0.f, 1.f, 0.f);
    glTexCoord2f(0.f, 0.f);

    glVertex3f(size, 0.f, -size);
    glNormal3f(0.f, 1.f, 0.f);
    glTexCoord2f(1.f, 0.f);

    glVertex3f(size, 0.f, size);
    glNormal3f(0.f, 1.f, 0.f);
    glTexCoord2f(1.f, 1.f);
    glEnd();

    glBegin(GL_TRIANGLES);
    glVertex3f(size, 0.f, size);
    glNormal3f(0.f, 1.f, 0.f);
    glTexCoord2f(1.f, 1.f);

    glVertex3f(-size, 0.f, size);
    glNormal3f(0.f, 1.f, 0.f);
    glTexCoord2f(0.f, 0.f);

    glVertex3f(-size, 0.f, -size);
    glNormal3f(0.f, 1.f, 0.f);
    glTexCoord2f(1.f, 0.f);
    glEnd();

    m_groundHeight = -3000.f;
}

void SpindizzyScene::doAnimate()
{
    m_rotationAngles.x = 0.08f;
    m_rotationAngles.y = 0.04f;
    m_rotationAngles.z = 0.07f;
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
}

void SpindizzyScene::doAddLights()
{
    // lights
    m_gpuKernel->setMaterial(129, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, false, false, 0, 0.f,
                             m_gpuKernel->getSceneInfo().viewDistance, TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
                             TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE, 1.f, 100.f, 0.f, 1.0f, 50.f,
                             m_gpuKernel->getSceneInfo().viewDistance, false);
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptXZPlane);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -10000.f, 10000.f - m_groundHeight, -10000.f, 500.f, 0.f, 500.f * 4.f,
                              DEFAULT_LIGHT_MATERIAL);
}
