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

#include <engines/GPUKernel.h>

namespace solr
{
enum GeometryType
{
    gtAtoms = 0,
    gtFixedSizeAtoms = 1,
    gtSticks = 2,
    gtAtomsAndSticks = 3,
    gtIsoSurface = 4,
    gtBackbone = 5
};

class SOLR_API PDBReader
{
public:
    PDBReader(void);
    virtual ~PDBReader(void);

public:
    vec4f loadAtomsFromFile(const std::string &filename, GPUKernel &cudaKernel,
                            GeometryType geometryType,
                            const float defaultAtomSize,
                            const float defaultStickSize,
                            const int materialType, const vec4f scale,
                            const bool useModels = false);

    int getNbBoxes() { return m_nbBoxes; }
    int getNbPrimitives() { return m_nbPrimitives; }

private:
    int m_nbPrimitives;
    int m_nbBoxes;
};
} // namespace solr
