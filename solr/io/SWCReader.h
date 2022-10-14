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
