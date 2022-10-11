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

#pragma once

#include <map>

#include <engines/GPUKernel.h>

namespace solr
{
struct Morphology
{
    int branch;
    float x;
    float y;
    float z;
    float radius;
    int parent;
    int primitiveId;
};
typedef std::map<int, Morphology> Morphologies;

class SOLR_API SWCReader
{
public:
    SWCReader();
    ~SWCReader();

    CPUBoundingBox loadMorphologyFromFile(const std::string &filename,
                                          GPUKernel &cudaKernel,
                                          const vec4f &position,
                                          const vec4f &scale,
                                          const int materialId);

    Morphologies getMorphologies() { return m_morphologies; }

private:
    Morphologies m_morphologies;
};
} // namespace solr
