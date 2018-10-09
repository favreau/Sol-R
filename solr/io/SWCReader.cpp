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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "../consts.h"
#include "../Logging.h"

#include "SWCReader.h"

namespace solr
{
SWCReader::SWCReader()
{
}

SWCReader::~SWCReader()
{
}

CPUBoundingBox SWCReader::loadMorphologyFromFile(const std::string &filename, GPUKernel &kernel, const vec4f &position,
                                                 const vec4f &scale, const int materialId)
{
    CPUBoundingBox AABB;
    LOG_INFO(1, "SWC Filename.......: " << filename);

    // Read vertices
    std::ifstream file(filename.c_str());
    if (file.is_open())
    {
        while (file.good())
        {
            std::string line;
            std::getline(file, line);

            std::string A, B, C, D, E, F, G;
            file >> A >> B >> C >> D >> E >> F >> G;
            if (A != "#")
            {
                Morphology morphology;
                int id = atoi(A.c_str());
                morphology.branch = atoi(B.c_str());
                morphology.x = static_cast<float>(scale.x * (position.x + atof(C.c_str())));
                morphology.y = static_cast<float>(scale.y * (position.y + atof(D.c_str())));
                morphology.z = static_cast<float>(scale.z * (position.z + atof(E.c_str())));
                morphology.radius = static_cast<float>(scale.w * atof(F.c_str()));
                morphology.parent = atoi(G.c_str());
                m_morphologies[id] = morphology;

                AABB.parameters[0].x = std::min(AABB.parameters[0].x, morphology.x);
                AABB.parameters[0].y = std::min(AABB.parameters[0].y, morphology.y);
                AABB.parameters[0].z = std::min(AABB.parameters[0].z, morphology.z);
                AABB.parameters[1].x = std::max(AABB.parameters[1].x, morphology.x);
                AABB.parameters[1].y = std::max(AABB.parameters[1].y, morphology.y);
                AABB.parameters[1].z = std::max(AABB.parameters[1].z, morphology.z);
            }
        }
        file.close();
    }

    // Build geometry
    Morphologies::iterator it = m_morphologies.begin();
    while (it != m_morphologies.end())
    {
        Morphology &a = (*it).second;
        if (a.parent == -1)
        {
            const vec2f vt0 = make_vec2f(0.f, 0.f);
            const vec2f vt1 = make_vec2f(2.f, 2.f);
            const vec2f vt2 = make_vec2f(0.f, 0.f);

            a.primitiveId = kernel.addPrimitive(ptSphere, true);
            kernel.setPrimitive(a.primitiveId, a.x, a.y, a.z, a.radius * 1.5f, 0.f, 0.f, materialId);
            kernel.setPrimitiveTextureCoordinates(a.primitiveId, vt0, vt1, vt2);
        }
        else
        {
            Morphology &b = m_morphologies[a.parent];
            if (b.parent != -1)
            {
                const vec2f vt0 = make_vec2f(0.f, 0.f);
                const vec2f vt1 = make_vec2f(1.f, 1.f);
                const vec2f vt2 = make_vec2f(0.f, 0.f);

                const float ra = a.radius;
                const float rb = b.radius;
                b.primitiveId = kernel.addPrimitive(ptCylinder, true);
                kernel.setPrimitive(b.primitiveId, a.x, a.y, a.z, b.x, b.y, b.z, ra, 0.f, 0.f, materialId);
                kernel.setPrimitiveTextureCoordinates(b.primitiveId, vt0, vt1, vt2);
                const int p = kernel.addPrimitive(ptSphere, true);
                kernel.setPrimitive(p, b.x, b.y, b.z, rb, 0.f, 0.f, materialId);
                kernel.setPrimitiveTextureCoordinates(p, vt0, vt1, vt2);
            }
        }
        ++it;
    }

    LOG_INFO(1, " - Points..........: " << m_morphologies.size());
    LOG_INFO(1, " - Bounding box....: (" << AABB.parameters[0].x << "," << AABB.parameters[0].y << ","
                                         << AABB.parameters[0].z << "),(" << AABB.parameters[1].x << ","
                                         << AABB.parameters[1].y << "," << AABB.parameters[1].z << ")");
    return AABB;
}
}
