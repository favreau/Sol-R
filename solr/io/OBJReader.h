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
struct MaterialMTL
{
    unsigned int index;
    vec4f Ka;
    vec4f Kd;
    vec4f Ks;
    float Ns;
    float reflection;
    float transparency;
    float opacity;
    float refraction;
    float gloss;
    int diffuseTextureId;
    int normalTextureId;
    int bumpTextureId;
    int specularTextureId;
    int reflectionTextureId;
    int transparencyTextureId;
    int ambientOcclusionTextureId;
    float illumination;
    bool isSketchupLightMaterial;
};

class SOLR_API OBJReader
{
public:
    OBJReader();
    ~OBJReader();

    unsigned int loadMaterialsFromFile(
        const std::string &filename,
        std::map<std::string, MaterialMTL> &m_materials, GPUKernel &GPUKernel,
        int materialId);

    vec4f loadModelFromFile(const std::string &filename, GPUKernel &cudaKernel,
                            const vec4f &center, const bool autoScale,
                            const vec4f &scale, bool loadMaterials,
                            int materialId, bool allSpheres, bool autoCenter,
                            CPUBoundingBox &aabb, const bool &checkInAABB,
                            const CPUBoundingBox &inAABB);

private:
    void addLightComponent(GPUKernel &kernel, std::vector<vec4f> &solrVertices,
                           const vec4f &center, const vec4f &objectCenter,
                           const vec4f &objectScale, const int material,
                           CPUBoundingBox &aabb);
};
} // namespace solr
