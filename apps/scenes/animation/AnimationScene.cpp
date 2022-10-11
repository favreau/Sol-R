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

#include "AnimationScene.h"

#ifdef WIN32
#include <windows.h>
#else
#include <fstream>
#endif

#include <io/FileMarshaller.h>
#include <io/OBJReader.h>
#include <io/PDBReader.h>

#undef HAND
#ifdef HAND
const int nbFrames(44);
#else
const int nbFrames(136);
#endif // HAND

const int nbModels(1);
const std::string gModels[nbModels] = {"hand"};

AnimationScene::AnimationScene(const std::string& name)
    : Scene(name)
    , m_wait(0)
{
    m_currentFrame = 0;
    m_forward = true;
}

AnimationScene::~AnimationScene(void) {}

void AnimationScene::doInitialize()
{
    int m = 30;
    solr::OBJReader objReader;
    vec4f objectSize = make_vec4f(3000.f, 3000.f, 3000.f);
    m_groundHeight = -1500.f;
    vec4f center = make_vec4f(0.f, -m_groundHeight / objectSize.y, 0.f);

    for (int frame = 0; frame < nbFrames; ++frame)
    {
        std::string fileName(m_fileName);
        char tmp[50];
#ifdef HAND
        sprintf_s(tmp, 50, "%02d", frame);
        fileName += "./animations/hand/hand_";
#else
#ifdef WIN32
        sprintf_s(tmp, 50, "%03d", frame + 1);
#else
        sprintf(tmp, "%03d", frame + 1);
#endif
        fileName += "./animations/08-10/08-10_000";
#endif // HAND
        fileName += tmp;
        fileName += ".obj";

        m_gpuKernel->setFrame(frame);
        m_gpuKernel->resetFrame();
        solr::CPUBoundingBox aabb;
        solr::CPUBoundingBox inAABB;
        objReader.loadModelFromFile(fileName, *m_gpuKernel, center, false,
                                    objectSize, (frame == 0), m, false, true,
                                    aabb, false, inAABB);

        // lights
        m_nbPrimitives = m_gpuKernel->addPrimitive(solr::ptSphere);
        m_gpuKernel->setPrimitive(m_nbPrimitives, -10000.f, 10000.f, -10000.f,
                                  100.f, 100.f, 100.f, DEFAULT_LIGHT_MATERIAL);
        m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);

        addCornellBox(m_cornellBoxType);
        if (frame != 0)
            m_gpuKernel->compactBoxes(true);
    }
    m_currentFrame = 0;
    m_wait = 0;
    m_gpuKernel->setFrame(m_currentFrame);
}

void AnimationScene::doAnimate()
{
    m_gpuKernel->setFrame(m_currentFrame);
    m_gpuKernel->compactBoxes(false);
    ++m_currentFrame;
    m_currentFrame = m_currentFrame % nbFrames;
}

void AnimationScene::doAddLights() {}
