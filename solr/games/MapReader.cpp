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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "../Logging.h"
#include "../consts.h"

#include "MapReader.h"

namespace solr
{
MapReader::MapReader(void) {}

MapReader::~MapReader(void) {}

vec3i readInt3(const std::string &value)
{
    vec3i returnValue = make_vec3i(0, 0, 0);
    int item(0);
    std::string tmp;
    for (int i(0); i < value.length(); ++i)
    {
        if (value[i] != ' ')
            tmp += value[i];

        if (value[i] == ' ' || i == (value.length() - 1))
        {
            switch (item)
            {
            case 0:
                returnValue.x = static_cast<int>(atoi(tmp.c_str()));
                break;
            case 1:
                returnValue.y = static_cast<int>(atoi(tmp.c_str()));
                break;
            case 2:
                returnValue.z = static_cast<int>(atoi(tmp.c_str()));
                break;
            }
            ++item;
            tmp = "";
        }
    }
    return returnValue;
}

vec4f MapReader::loadFromFile(const std::string &filename, GPUKernel &kernel)
{
    std::map<int, MapMaterialScheme> materials;
    std::map<int, MapZone> zones;
    std::map<int, MapBlock> blocks;
    std::map<int, MapJewel> jewels;

    vec4f minPos = make_vec4f(100000.f, 100000.f, 100000.f);
    vec4f maxPos = make_vec4f(-100000.f, -100000.f, -100000.f);

    // Load Map
    LOG_INFO(3, "Loading " << filename.c_str());

    int index_zone(-1);
    int index_block(-1);
    int index_jewel(-1);

    int jewelMaterial = 40;
    int blockMaterial = 30;

    std::ifstream file(filename.c_str());
    if (file.is_open())
    {
        while (file.good() /*&& index_zone<30*/) // TO REMOVE!!!
        {
            std::string line;
            std::getline(file, line);
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            if (line.length() > 0)
            {
                std::stringstream s(line);
                std::string item;
                if (line.find("<COLOURSCHEME ") == 0)
                {
                    // Read Color scheme
                }
                else if (line.find("<JEWEL") == 0)
                {
                    ++index_jewel;
                    MapJewel malJewel;
                    s >> item >> item >> malJewel.location.x >>
                        malJewel.location.z >>
                        malJewel.location.y; // Caution y <-> z
                    jewels[index_jewel] = malJewel;
                }
                else if (line.find("<ZONE") == 0)
                {
                    // <ZONE LOCATION 0 0 0 DIMENSIONS 54 37 23 COLOURSCHEME 0>
                    ++index_zone;
                    MapZone mapZone;
                    s >> item >> item >> mapZone.location.x >>
                        mapZone.location.z >>
                        mapZone.location.y >> // Caution y <-> z
                        item >> mapZone.dimension.x >> mapZone.dimension.y >>
                        mapZone.dimension.z >> item >> mapZone.colorScene;
                    zones[index_zone] = mapZone;
                }
                else if (line.find("<BLOCK ") == 0)
                {
                    // Read block
                    LOG_INFO(2, line);
                    // <BLOCK LOCATION 8 0 0 DIMENSION 53 36 0 NORTH 0 SOUTH 0
                    // EAST 0 WEST 0 SPLIT 0 HOLLOW 0>
                    ++index_block;
                    MapBlock block;
                    memset(&block, 0, sizeof(MapBlock));
                    s >> item >> item >> block.location.x >> block.location.z >>
                        block.location.y >> // Caution y <-> z
                        item >> block.dimension.x >> block.dimension.z >>
                        block.dimension.y;

                    if (line.find("TYPE") != -1)
                    {
                        s >> item >> block.type;
                    }
                    else
                    {
                        s >> item >> block.north >> item >> block.south >>
                            item >> block.east >> item >> block.west >> item >>
                            block.split >> item >> block.hollow;
                    }
                    block.zone = index_zone;
                    blocks[index_block] = block;
                }
            }
        }
        file.close();
    }

    vec4f blockSize = make_vec4f(500.f, 500.f, 500.f);

    // Jewels
    std::map<int, MapJewel>::const_iterator itj = jewels.begin();
    while (itj != jewels.end())
    {
        MapJewel jewel = (*itj).second;
        vec4f position;
        position.x = jewel.location.x * blockSize.x;
        position.y = jewel.location.y * blockSize.y;
        position.z = jewel.location.z * blockSize.z;
        int nbPrimitives = kernel.addPrimitive(ptSphere);
        kernel.setPrimitive(nbPrimitives, position.x + blockSize.x / 2.f,
                            position.y + blockSize.x / 2.f,
                            position.z + blockSize.x / 2.f, blockSize.x / 4.f,
                            0.f, 0.f, jewelMaterial);
        ++itj;
    }

#if 0
    // Zones
    std::map<int,MapZone>::const_iterator itz = zones.begin();
    while( itz != zones.end() )
    {
        MapZone zone = (*itz).second;
        vec4f position;
        position.x = zone.location.x*blockSize.x;
        position.y = zone.location.y*blockSize.y - 50.f;
        position.z = zone.location.z*blockSize.z;

        vec4f dimension;
        dimension.x = zone.dimension.x*blockSize.x;
        dimension.y = zone.dimension.y*blockSize.y;
        dimension.z = zone.dimension.z*blockSize.z;

        int nbPrimitives = kernel.addPrimitive( ptTriangle );
        kernel.setPrimitive(
                    nbPrimitives,
                    position.x , position.y, position.z,
                    dimension.x, position.y, position.z,
                    dimension.x, position.y, dimension.z,
                    0.f, 0.f, 0.f,
                    zoneMaterial, 1, 1);
        nbPrimitives = kernel.addPrimitive( ptTriangle );
        kernel.setPrimitive(
                    nbPrimitives,
                    dimension.x, position.y, dimension.z,
                    position.x , position.y, dimension.z,
                    position.x , position.y, position.z,
                    0.f, 0.f, 0.f,
                    zoneMaterial, 1, 1);
        ++itz;
    }
#endif // 0

    std::map<int, MapBlock>::const_iterator it = blocks.begin();
    while (it != blocks.end())
    {
        MapBlock block = (*it).second;
        vec4f position;
        position.x = block.location.x * blockSize.x;
        position.y = block.location.y * blockSize.y + block.type;
        position.z = block.location.z * blockSize.z;

        vec4f dimension;
        dimension.x = block.dimension.x * blockSize.x;
        dimension.y = block.dimension.y * blockSize.y;
        dimension.z = block.dimension.z * blockSize.z;
        unsigned int nbPrimitives;

        blockMaterial = (block.type != 0) ? 60 : 30;
#if 1
        // Front
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, position.x, position.y, position.z,
                            dimension.x, position.y, position.z, dimension.x,
                            dimension.y, position.z, 0.f, 0.f, 0.f,
                            blockMaterial);
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, dimension.x, dimension.y, position.z,
                            position.x, dimension.y, position.z, position.x,
                            position.y, position.z, 0.f, 0.f, 0.f,
                            blockMaterial);

        // Back
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, position.x, position.y, dimension.z,
                            dimension.x, position.y, dimension.z, dimension.x,
                            dimension.y, dimension.z, 0.f, 0.f, 0.f,
                            blockMaterial);
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, dimension.x, dimension.y, dimension.z,
                            position.x, dimension.y, dimension.z, position.x,
                            position.y, dimension.z, 0.f, 0.f, 0.f,
                            blockMaterial);

        // Left
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, position.x, position.y, position.z,
                            position.x, position.y, dimension.z, position.x,
                            dimension.y, dimension.z, 0.f, 0.f, 0.f,
                            blockMaterial);
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, position.x, dimension.y, dimension.z,
                            position.x, dimension.y, position.z, position.x,
                            position.y, position.z, 0.f, 0.f, 0.f,
                            blockMaterial);

        // Right
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, dimension.x, position.y, position.z,
                            dimension.x, position.y, dimension.z, dimension.x,
                            dimension.y, dimension.z, 0.f, 0.f, 0.f,
                            blockMaterial);
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, dimension.x, dimension.y, dimension.z,
                            dimension.x, dimension.y, position.z, dimension.x,
                            position.y, position.z, 0.f, 0.f, 0.f,
                            blockMaterial);

        // Top
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, position.x, dimension.y, position.z,
                            dimension.x, dimension.y, position.z, dimension.x,
                            dimension.y, dimension.z, 0.f, 0.f, 0.f,
                            blockMaterial);
        nbPrimitives = kernel.addPrimitive(ptTriangle);
        kernel.setPrimitive(nbPrimitives, dimension.x, dimension.y, dimension.z,
                            position.x, dimension.y, dimension.z, position.x,
                            dimension.y, position.z, 0.f, 0.f, 0.f,
                            blockMaterial);

#else
        // Blocks
        switch (block.type)
        {
        case 19:
        {
            int nbPrimitives = kernel.addPrimitive(ptTriangle);
            kernel.setPrimitive(nbPrimitives, position.x, position.y,
                                position.z, dimension.x, position.y, position.z,
                                dimension.x, position.y, dimension.z, 0.f, 0.f,
                                0.f, zoneMaterial, 1, 1);
            nbPrimitives = kernel.addPrimitive(ptTriangle);
            kernel.setPrimitive(nbPrimitives, dimension.x, position.y,
                                dimension.z, position.x, position.y,
                                dimension.z, position.x, position.y, position.z,
                                0.f, 0.f, 0.f, zoneMaterial, 1, 1);
            break;
        }
        default:
        {
            break;
        }
        }
#endif // 0

        ++it;
    }
    vec4f objectSize;
    objectSize.x = (maxPos.x - minPos.x);
    objectSize.y = (maxPos.y - minPos.y);
    objectSize.z = (maxPos.z - minPos.z);
    return objectSize;
}
} // namespace solr
