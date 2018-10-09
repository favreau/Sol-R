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

#ifdef WIN32
#include <windows.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include "../consts.h"
#include "../Logging.h"

#include "FileMarshaller.h"

namespace
{
const std::string SCENEINFO = "SCENEINFO";
const std::string PRIMITIVE = "PRIMITIVE";
const std::string MATERIAL = "MATERIAL";
const std::string TEXTURE = "TEXTURE";
#ifdef USE_OPENCL
const size_t FORMAT_VERSION = 1;
#else
const size_t FORMAT_VERSION = 2;
#endif
}

namespace solr
{
vec4f FileMarshaller::loadFromFile(GPUKernel &kernel, const std::string &filename, const vec4f &center,
                                   const float scale)
{
    LOG_INFO(1, "IRT Filename.......: " << filename);

    vec4f returnValue = make_vec4f(0.f, 0.f, 0.f, 0.f);
    vec4f min = make_vec4f(kernel.getSceneInfo().viewDistance, kernel.getSceneInfo().viewDistance,
        kernel.getSceneInfo().viewDistance);
    vec4f max = make_vec4f(-kernel.getSceneInfo().viewDistance, -kernel.getSceneInfo().viewDistance,
        -kernel.getSceneInfo().viewDistance);

    std::ifstream myfile;
    myfile.open(filename.c_str(), std::ifstream::binary);
    if (myfile.is_open())
    {
        // Format
        size_t version;
        myfile.read((char *)&version, sizeof(size_t));
        LOG_INFO(1, " - Version.........: " << version);

        if (version != FORMAT_VERSION)
        {
            LOG_ERROR("File not compatible with current engine");
            myfile.close();
            return returnValue;
        }

        SceneInfo sceneInfo;
        myfile.read((char *)&sceneInfo, sizeof(SceneInfo));

        // --------------------------------------------------------------------------------
        // Primitives
        // --------------------------------------------------------------------------------
        size_t nbElements(0);
        myfile.read((char *)&nbElements, sizeof(size_t));
        LOG_INFO(1, " - Primitives......: " << nbElements);

        for (size_t i(0); i < nbElements; ++i)
        {
            CPUPrimitive primitive;
            myfile.read((char *)&primitive, sizeof(CPUPrimitive));

            int n = kernel.addPrimitive(static_cast<PrimitiveType>(primitive.type));
            kernel.setPrimitive(n, center.x + primitive.p0.x, center.y + primitive.p0.y, center.z + primitive.p0.z,
                                center.x + primitive.p1.x, center.y + primitive.p1.y, center.z + primitive.p1.z,
                                center.x + primitive.p2.x, center.y + primitive.p2.y, center.z + primitive.p2.z,
                                primitive.size.x, primitive.size.y, primitive.size.z, primitive.materialId);
            kernel.setPrimitiveBellongsToModel(n, true);
            kernel.setPrimitiveIsMovable(n, false);
            kernel.setPrimitiveNormals(n, primitive.n0, primitive.n1, primitive.n2);
            kernel.setPrimitiveTextureCoordinates(n, primitive.vt0, primitive.vt1, primitive.vt2);

            vec4f pmin, pmax;
            pmin.x = std::min(std::min(primitive.p0.x, primitive.p1.x), primitive.p2.x);
            pmin.y = std::min(std::min(primitive.p0.y, primitive.p1.y), primitive.p2.y);
            pmin.z = std::min(std::min(primitive.p0.z, primitive.p1.z), primitive.p2.z);
            pmax.x = std::max(std::max(primitive.p0.x, primitive.p1.x), primitive.p2.x);
            pmax.y = std::max(std::max(primitive.p0.y, primitive.p1.y), primitive.p2.y);
            pmax.z = std::max(std::max(primitive.p0.z, primitive.p1.z), primitive.p2.z);

            min.x = (pmin.x < min.x) ? pmin.x : min.x;
            min.y = (pmin.y < min.y) ? pmin.y : min.y;
            min.z = (pmin.z < min.z) ? pmin.z : min.z;

            max.x = (pmax.x > max.x) ? pmax.x : max.x;
            max.y = (pmax.y > max.y) ? pmax.y : max.y;
            max.z = (pmax.z > max.z) ? pmax.z : max.z;
        }

        // --------------------------------------------------------------------------------
        // Textures
        // --------------------------------------------------------------------------------
        size_t nbTextures(0);
        myfile.read((char *)&nbTextures, sizeof(size_t));
        LOG_INFO(1, " - Textures........: " << nbTextures);

        int nbActiveTextures = kernel.getNbActiveTextures();
        for (unsigned int i(0); i < nbTextures; ++i)
        {
            size_t id;
            myfile.read((char *)&id, sizeof(size_t));

            TextureInfo texInfo;
            myfile.read((char *)&texInfo, sizeof(TextureInfo));
            LOG_INFO(3, "Texture with id " << id << " and size: " << texInfo.size.x << "x" << texInfo.size.y << "x"
                                           << texInfo.size.z << " loaded into slot " << nbActiveTextures + i);

            size_t imageSize = texInfo.size.x * texInfo.size.y * texInfo.size.z;
            texInfo.buffer = new BitmapBuffer[imageSize];
            myfile.read((char *)texInfo.buffer, imageSize);

            kernel.setTexture(static_cast<const int>(nbActiveTextures + i), texInfo);
            delete[] texInfo.buffer;
        }

        // --------------------------------------------------------------------------------
        // Materials
        // --------------------------------------------------------------------------------
        size_t nbMaterials(0);
        myfile.read((char *)&nbMaterials, sizeof(size_t));
        LOG_INFO(1, " - Materials.......: " << nbMaterials);

        for (unsigned int i(0); i < nbMaterials; ++i)
        {
            size_t id;
            Material material;
            myfile.read((char *)&id, sizeof(size_t));
            myfile.read((char *)&material, sizeof(Material));
            if (material.textureIds.x != TEXTURE_NONE)
                material.textureIds.x += nbActiveTextures;
            if (material.textureIds.y != TEXTURE_NONE)
                material.textureIds.y += nbActiveTextures;
            if (material.textureIds.z != TEXTURE_NONE)
                material.textureIds.z += nbActiveTextures;
            if (material.textureIds.w != TEXTURE_NONE)
                material.textureIds.w += nbActiveTextures;
            if (material.advancedTextureIds.x != TEXTURE_NONE)
                material.advancedTextureIds.x += nbActiveTextures;
            if (material.advancedTextureIds.y != TEXTURE_NONE)
                material.advancedTextureIds.y += nbActiveTextures;
            if (material.advancedTextureIds.z != TEXTURE_NONE)
                material.advancedTextureIds.z += nbActiveTextures;
            if (material.advancedTextureIds.w != TEXTURE_NONE)
                material.advancedTextureIds.w += nbActiveTextures;
            LOG_INFO(3, "Loading material " << id << " (" << material.textureIds.x << "," << material.textureIds.y
                                            << "," << material.textureIds.z << "," << material.textureIds.w
                                            << material.advancedTextureIds.x << "," << material.advancedTextureIds.y
                                            << "," << material.advancedTextureIds.z << ","
                                            << material.advancedTextureIds.w << ")");
            kernel.setMaterial(static_cast<unsigned int>(id), material);
        }
    }
    myfile.close();
    LOG_INFO(3, "File " << filename << " successfully loaded!");

    // Object size
    returnValue.x = fabs(max.x - min.x);
    returnValue.y = fabs(max.y - min.y);
    returnValue.z = fabs(max.z - min.z);

    // Resize to fit required size
    float ratio = scale / returnValue.y; // std::max(returnValue.x,std::max(returnValue.y,returnValue.z));
    kernel.scalePrimitives(ratio, 0, NB_MAX_BOXES);

    LOG_INFO(3, "Object size: " << returnValue.x << ", " << returnValue.y << ", " << returnValue.z);
    return returnValue;
}

void FileMarshaller::saveToFile(GPUKernel &kernel, const std::string &filename)
{
    LOG_INFO(1, "Saving 3D scene to " << filename);
    std::ofstream myfile;
    myfile.open(filename.c_str(), std::ifstream::binary);
    if (myfile.is_open())
    {
        // Format version
        size_t version = FORMAT_VERSION;
        myfile.write((char *)&version, sizeof(size_t));

        std::map<size_t, Material *> materials;

        // Scene
        SceneInfo &sceneInfo = kernel.getSceneInfo();
        myfile.write((char *)&sceneInfo, sizeof(SceneInfo));

        // Primitives
        // Count primitives belonging to the model
        size_t nbTotalPrimitives = kernel.getNbActivePrimitives();
        size_t nbPrimitives(0);
        for (int i(0); i < nbTotalPrimitives; ++i)
        {
            if (kernel.getPrimitive(i)->belongsToModel)
                ++nbPrimitives;
        }

        myfile.write((char *)&nbPrimitives, sizeof(size_t));
        LOG_INFO(1, "Saving " << nbPrimitives << " primitives");

        // Identify used materials
        std::map<int, int> materialIndexMapping;
        for (int i(0); i < nbPrimitives; ++i)
        {
            CPUPrimitive *primitive = kernel.getPrimitive(i);
            myfile.write((char *)primitive, sizeof(CPUPrimitive));
            materials[primitive->materialId] = kernel.getMaterial(primitive->materialId);
        }

        // Determine textures in use
        std::map<size_t, TextureInfo> textures;
        for (auto material : materials)
        {
            if (material.second->textureIds.x != TEXTURE_NONE)
                textures[material.second->textureIds.x] = kernel.getTextureInformation(material.second->textureIds.x);
            if (material.second->textureIds.y != TEXTURE_NONE)
                textures[material.second->textureIds.y] = kernel.getTextureInformation(material.second->textureIds.y);
            if (material.second->textureIds.z != TEXTURE_NONE)
                textures[material.second->textureIds.z] = kernel.getTextureInformation(material.second->textureIds.z);
            if (material.second->textureIds.w != TEXTURE_NONE)
                textures[material.second->textureIds.w] = kernel.getTextureInformation(material.second->textureIds.w);
            if (material.second->advancedTextureIds.x != TEXTURE_NONE)
                textures[material.second->advancedTextureIds.x] =
                    kernel.getTextureInformation(material.second->advancedTextureIds.x);
            if (material.second->advancedTextureIds.y != TEXTURE_NONE)
                textures[material.second->advancedTextureIds.y] =
                    kernel.getTextureInformation(material.second->advancedTextureIds.y);
            if (material.second->advancedTextureIds.z != TEXTURE_NONE)
                textures[material.second->advancedTextureIds.z] =
                    kernel.getTextureInformation(material.second->advancedTextureIds.z);
            if (material.second->advancedTextureIds.w != TEXTURE_NONE)
                textures[material.second->advancedTextureIds.w] =
                    kernel.getTextureInformation(material.second->advancedTextureIds.w);
        }

        // Write Textures
        size_t nbTextures = textures.size();
        myfile.write((char *)&nbTextures, sizeof(size_t));
        LOG_INFO(1, "Saving " << nbTextures << " textures");

        std::map<size_t, int> idMapping;
        size_t index(0);
        idMapping[TEXTURE_NONE] = TEXTURE_NONE;
        for (const auto &texture : textures)
        {
            TextureInfo texInfo = texture.second;
            BitmapBuffer *savedBuffer = texInfo.buffer;
            texInfo.buffer = 0;
            texInfo.offset = 0;
            size_t id = texture.first;
            idMapping[id] = static_cast<int>(index);
            LOG_INFO(1, "Texture " << id << ": " << texInfo.size.x << "x" << texInfo.size.y << "x" << texInfo.size.z
                                   << " saved with id " << index);
            myfile.write((char *)(&index), sizeof(size_t));
            myfile.write((char *)(&texInfo), sizeof(TextureInfo));
            myfile.write((char *)(savedBuffer), texInfo.size.x * texInfo.size.y * texInfo.size.z);
            ++index;
        }

        // Write Materials
        size_t nbMaterials = materials.size();
        myfile.write((char *)&nbMaterials, sizeof(size_t));
        LOG_INFO(1, "Saving " << nbMaterials << " materials");

        for (const auto &material : materials)
        {
            material.second->textureIds.x = idMapping[material.second->textureIds.x];
            material.second->textureIds.y = idMapping[material.second->textureIds.y];
            material.second->textureIds.z = idMapping[material.second->textureIds.z];
            material.second->textureIds.w = idMapping[material.second->textureIds.w];
            material.second->advancedTextureIds.x = idMapping[material.second->advancedTextureIds.x];
            material.second->advancedTextureIds.y = idMapping[material.second->advancedTextureIds.y];
            material.second->advancedTextureIds.z = idMapping[material.second->advancedTextureIds.z];
            material.second->advancedTextureIds.w = idMapping[material.second->advancedTextureIds.w];
            myfile.write((char *)&(material.first), sizeof(size_t));
            myfile.write((char *)(material.second), sizeof(Material));
            LOG_INFO(1, "Saving material "
                            << material.first << " (" << material.second->textureIds.x << ","
                            << material.second->textureIds.y << "," << material.second->textureIds.z << ","
                            << material.second->textureIds.w << "," << material.second->advancedTextureIds.x << ","
                            << material.second->advancedTextureIds.y << "," << material.second->advancedTextureIds.z
                            << "," << material.second->advancedTextureIds.w << ")");
        }

        myfile.close();
    }
}
}
