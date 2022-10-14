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

// Project
#include "TextureMapping.cuh"

// Cuda
#include <cuda_runtime_api.h>
#include <vector_functions.h>

/*
________________________________________________________________________________

Intersection Shader
________________________________________________________________________________
*/
__device__ vec4f intersectionShader(const SceneInfo &sceneInfo,
                                    const Primitive &primitive,
                                    Material *materials, BitmapBuffer *textures,
                                    vec3f &intersection, const vec3f &areas,
                                    vec3f &normal, vec4f &specular,
                                    vec4f &attributes,
                                    vec4f &advancedAttributes)
{
    vec4f colorAtIntersection = materials[primitive.materialId].color;
    colorAtIntersection.w =
        0.f; // w attribute is used to dtermine light intensity of the material

    if (sceneInfo.extendedGeometry)
    {
        switch (primitive.type)
        {
        case ptCone:
        case ptCylinder:
        case ptEnvironment:
        case ptSphere:
        case ptEllipsoid:
        {
            if (materials[primitive.materialId].textureIds.x != TEXTURE_NONE)
            {
                colorAtIntersection =
                    sphereUVMapping(primitive, materials, textures,
                                    intersection, normal, specular, attributes,
                                    advancedAttributes);
            }
            break;
        }
        case ptCheckboard:
        {
            if (materials[primitive.materialId].textureIds.x != TEXTURE_NONE)
            {
                colorAtIntersection =
                    cubeMapping(sceneInfo, primitive, materials, textures,
                                intersection, normal, specular, attributes,
                                advancedAttributes);
            }
            else
            {
                int x = sceneInfo.viewDistance +
                        ((intersection.x - primitive.p0.x) / primitive.size.x);
                int z = sceneInfo.viewDistance +
                        ((intersection.z - primitive.p0.z) / primitive.size.x);
                if (x % 2 == 0)
                {
                    if (z % 2 == 0)
                    {
                        colorAtIntersection.x = 1.f - colorAtIntersection.x;
                        colorAtIntersection.y = 1.f - colorAtIntersection.y;
                        colorAtIntersection.z = 1.f - colorAtIntersection.z;
                    }
                }
                else
                {
                    if (z % 2 != 0)
                    {
                        colorAtIntersection.x = 1.f - colorAtIntersection.x;
                        colorAtIntersection.y = 1.f - colorAtIntersection.y;
                        colorAtIntersection.z = 1.f - colorAtIntersection.z;
                    }
                }
            }
            break;
        }
        case ptXYPlane:
        case ptYZPlane:
        case ptXZPlane:
        case ptCamera:
        {
            if (materials[primitive.materialId].textureIds.x != TEXTURE_NONE)
            {
                colorAtIntersection =
                    cubeMapping(sceneInfo, primitive, materials, textures,
                                intersection, normal, specular, attributes,
                                advancedAttributes);
            }
            break;
        }
        case ptTriangle:
        {
            if (materials[primitive.materialId].textureIds.x != TEXTURE_NONE)
            {
                colorAtIntersection =
                    triangleUVMapping(sceneInfo, primitive, materials, textures,
                                      intersection, areas, normal, specular,
                                      attributes, advancedAttributes);
            }
            break;
        }
        }
    }
    else
    {
        if (materials[primitive.materialId].textureIds.x != TEXTURE_NONE)
        {
            colorAtIntersection =
                triangleUVMapping(sceneInfo, primitive, materials, textures,
                                  intersection, areas, normal, specular,
                                  attributes, advancedAttributes);
        }
    }
    return colorAtIntersection;
}

/*
________________________________________________________________________________

Convert vec4f into OpenGL RGB color
________________________________________________________________________________
*/
__device__ __INLINE__ void makeColor(const SceneInfo &sceneInfo, vec4f &color,
                                     BitmapBuffer *bitmap, int index)
{
    int mdc_index = index * gColorDepth;
    color.x = (color.x > 1.f) ? 1.f : color.x;
    color.y = (color.y > 1.f) ? 1.f : color.y;
    color.z = (color.z > 1.f) ? 1.f : color.z;
    color.x = (color.x < 0.f) ? 0.f : color.x;
    color.y = (color.y < 0.f) ? 0.f : color.y;
    color.z = (color.z < 0.f) ? 0.f : color.z;

    switch (sceneInfo.frameBufferType)
    {
    case ftBGR:
    {
        // Delphi
        int y = index / sceneInfo.size.y;
        int x = index % sceneInfo.size.x;
        int i = (y + 1) * sceneInfo.size.y - x - 1;
        i *= gColorDepth;
        bitmap[i] = (BitmapBuffer)(color.z * 255.f);     // Blue
        bitmap[i + 1] = (BitmapBuffer)(color.y * 255.f); // Green
        bitmap[i + 2] = (BitmapBuffer)(color.x * 255.f); // Red
        break;
    }
    default:
    {
        // OpenGL
        bitmap[mdc_index] = (BitmapBuffer)(color.x * 255.f);     // Red
        bitmap[mdc_index + 1] = (BitmapBuffer)(color.y * 255.f); // Green
        bitmap[mdc_index + 2] = (BitmapBuffer)(color.z * 255.f); // Blue
        break;
    }
    }
}
