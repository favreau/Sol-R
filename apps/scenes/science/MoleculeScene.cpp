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

#include "MoleculeScene.h"

// System
#ifdef WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif
#include <math.h>

// Project
#include <common/Utils.h>

// Sol-R
#include <io/FileMarshaller.h>
#include <io/PDBReader.h>

MoleculeScene::MoleculeScene(const std::string& name)
    : Scene(name)
{
    m_groundHeight = -5000.f;
}

MoleculeScene::~MoleculeScene(void) {}

void MoleculeScene::doInitialize()
{
    // initialization
    const int geometryType = rand() % 5;
    const int atomMaterialType = 0;
    const float defaultAtomSize = 100.f;
    const float defaultStickSize = 10.f;
    const bool loadModels = true;

    Strings extensions;
    extensions.push_back(".pdb");
    const Strings fileNames =
        getFilesFromFolder(std::string(DEFAULT_MEDIA_FOLDER) + "/pdb",
                           extensions);
    if (fileNames.size() != 0)
    {
        m_currentModel = m_currentModel % fileNames.size();
        const vec4f scale = make_vec4f(200.f, 200.f, 200.f);

        // Scene
        m_name = fileNames[m_currentModel];

        // PDB
        PDBReader pdbReader;
        pdbReader.loadAtomsFromFile(m_name, *m_gpuKernel,
                                    static_cast<GeometryType>(geometryType),
                                    defaultAtomSize, defaultStickSize,
                                    atomMaterialType, scale, loadModels);
    }
}

void MoleculeScene::doAnimate()
{
    const int nbFrames = 120;
    m_rotationAngles.y = static_cast<float>(-2.f * M_PI / nbFrames);
    m_gpuKernel->rotatePrimitives(m_rotationCenter, m_rotationAngles);
    m_gpuKernel->compactBoxes(false);
}

void MoleculeScene::doAddLights()
{
    // Lights
    m_nbPrimitives = m_gpuKernel->addPrimitive(ptSphere);
    m_gpuKernel->setPrimitive(m_nbPrimitives, -5000.f, 5000.f, -15000.f, 1.f,
                              0.f, 0.f, DEFAULT_LIGHT_MATERIAL);
    m_gpuKernel->setPrimitiveIsMovable(m_nbPrimitives, false);
}
