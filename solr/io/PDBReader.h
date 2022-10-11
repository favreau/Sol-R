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
