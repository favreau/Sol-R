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
#include "../../solr.h"
#include "VectorUtils.cuh"

// ----------
// Normal mapping
// --------------------
__device__ __INLINE__ void normalMap(const int &index, const Material &material,
                                     BitmapBuffer *textures, vec3f &normal,
                                     const float strength)
{
    int i = material.textureOffset.y + index;
    BitmapBuffer r, g;
    r = textures[i];
    g = textures[i + 1];
    normal.x -= strength * (r / 256.f - 0.5f);
    normal.y -= strength * (g / 256.f - 0.5f);
    normal.z = 0.f;
}

// ----------
// Bump mapping
// --------------------
__device__ __INLINE__ void bumpMap(const int &index, const Material &material,
                                   BitmapBuffer *textures, vec3f &intersection,
                                   float &value)
{
    int i = material.textureOffset.z + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    value = 10.f * (r + g + b) / 768.f;
    // intersection.x += value;
    // intersection.y += value;
    // intersection.z += value;
}

// ----------
// Normal mapping
// --------------------
__device__ __INLINE__ void specularMap(const int &index,
                                       const Material &material,
                                       BitmapBuffer *textures, vec4f &specular)
{
    int i = material.textureOffset.w + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    specular.x = r / 256.f;
    specular.y = 1000.f * g / 256.f;
    specular.z = b / 256.f;
}

// ----------
// Reflection mapping
// --------------------
__device__ __INLINE__ void reflectionMap(const int &index,
                                         const Material &material,
                                         BitmapBuffer *textures,
                                         vec4f &attributes)
{
    int i = material.advancedTextureOffset.x + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    attributes.x *= (r + g + b) / 768.f;
}

// ----------
// Transparency mapping
// --------------------
__device__ __INLINE__ void transparencyMap(const int &index,
                                           const Material &material,
                                           BitmapBuffer *textures,
                                           vec4f &attributes)
{
    int i = material.advancedTextureOffset.y + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    attributes.y *= (r + g + b) / 768.f;
    // attributes.z = 10.f*b/256.f;
}

// ----------
// Ambient occlusion
// --------------------
__device__ __INLINE__ void ambientOcclusionMap(const int &index,
                                               const Material &material,
                                               BitmapBuffer *textures,
                                               vec4f &advancedAttributes)
{
    int i = material.advancedTextureOffset.z + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    advancedAttributes.x = (r + g + b) / 768.f;
}

__device__ __INLINE__ void juliaSet(const Primitive &primitive,
                                    Material *materials,
                                    const SceneInfo &sceneInfo, const float x,
                                    const float y, float4 &color)
{
    Material &material = materials[primitive.materialId];
    float W = (float)material.textureMapping.x;
    float H = (float)material.textureMapping.y;

    // pick some values for the constant c, this determines the shape of the
    // Julia Set
    float cRe = -0.7f + 0.4f * sinf(sceneInfo.timestamp / 1500.f);
    float cIm = 0.27015f + 0.4f * cosf(sceneInfo.timestamp / 2000.f);

    // calculate the initial real and imaginary part of z, based on the pixel
    // location and zoom and position values
    float newRe = 1.5f * (x - W / 2.f) / (0.5f * W);
    float newIm = (y - H / 2.f) / (0.5f * H);
    // i will represent the number of iterations
    int n;
    // start the iteration process
    float maxIterations = 40.f + sceneInfo.pathTracingIteration;
    for (n = 0; n < maxIterations; n++)
    {
        // remember value of previous iteration
        float oldRe = newRe;
        float oldIm = newIm;
        // the actual iteration, the real and imaginary part are calculated
        newRe = oldRe * oldRe - oldIm * oldIm + cRe;
        newIm = 2.f * oldRe * oldIm + cIm;
        // if the point is outside the circle with radius 2: stop
        if ((newRe * newRe + newIm * newIm) > 4.f)
            break;
    }
    // use color model conversion to get rainbow palette, make brightness black
    // if maxIterations reached color.x += newRe/4.f; color.z += newIm/4.f;
    color.x = 1.f - color.x * (n / maxIterations);
    color.y = 1.f - color.y * (n / maxIterations);
    color.z = 1.f - color.z * (n / maxIterations);
    color.w = 1.f - (n / maxIterations);
}

__device__ __INLINE__ void mandelbrotSet(const Primitive &primitive,
                                         Material *materials,
                                         const SceneInfo &sceneInfo,
                                         const float x, const float y,
                                         float4 &color)
{
    Material &material = materials[primitive.materialId];
    float W = (float)material.textureMapping.x;
    float H = (float)material.textureMapping.y;

    float MinRe = -2.f;
    float MaxRe = 1.f;
    float MinIm = -1.2f;
    float MaxIm = MinIm + (MaxRe - MinRe) * H / W;
    float Re_factor = (MaxRe - MinRe) / (W - 1.f);
    double Im_factor = (MaxIm - MinIm) / (H - 1.f);
    float maxIterations = NB_MAX_ITERATIONS + sceneInfo.pathTracingIteration;

    float c_im = MaxIm - y * Im_factor;
    float c_re = MinRe + x * Re_factor;
    float Z_re = c_re;
    float Z_im = c_im;
    bool isInside = true;
    unsigned n;
    for (n = 0; isInside && n < maxIterations; ++n)
    {
        float Z_re2 = Z_re * Z_re;
        float Z_im2 = Z_im * Z_im;
        if (Z_re2 + Z_im2 > 4.f)
        {
            isInside = false;
        }
        Z_im = 2.f * Z_re * Z_im + c_im;
        Z_re = Z_re2 - Z_im2 + c_re;
    }

    color.x = 1.f - color.x * (n / maxIterations);
    color.y = 1.f - color.y * (n / maxIterations);
    color.z = 1.f - color.z * (n / maxIterations);
    color.w = 1.f - (n / maxIterations);
}

/*
________________________________________________________________________________

Triangle texture Mapping
________________________________________________________________________________
*/
__device__ __INLINE__ float4 triangleUVMapping(
    const SceneInfo &sceneInfo, const Primitive &primitive, Material *materials,
    BitmapBuffer *textures, vec3f &intersection, const vec3f &areas,
    vec3f &normal, vec4f &specular, vec4f &attributes,
    vec4f &advancedAttributes)
{
    Material &material = materials[primitive.materialId];
    float4 result = material.color;

    vec2f T = (primitive.vt0 * areas.x + primitive.vt1 * areas.y +
               primitive.vt2 * areas.z) /
              (areas.x + areas.y + areas.z);
    float2 mappingOffset = {0.f, 0.f};
    if (material.attributes.y == 1)
    {
        mappingOffset.x = material.mappingOffset.x * sceneInfo.timestamp;
        mappingOffset.y = material.mappingOffset.y * sceneInfo.timestamp;
    }
    int u = T.x * material.textureMapping.x + mappingOffset.x;
    int v = T.y * material.textureMapping.y + mappingOffset.y;

    u = u % material.textureMapping.x;
    v = v % material.textureMapping.y;
    if (u >= 0 && u < material.textureMapping.x && v >= 0 &&
        v < material.textureMapping.y)
    {
        switch (material.textureIds.x)
        {
        case TEXTURE_MANDELBROT:
            mandelbrotSet(primitive, materials, sceneInfo, u, v, result);
            break;
        case TEXTURE_JULIA:
            juliaSet(primitive, materials, sceneInfo, u, v, result);
            break;
        default:
        {
            int A =
                (v * material.textureMapping.x + u) * material.textureMapping.w;
            int B = material.textureMapping.x * material.textureMapping.y *
                    material.textureMapping.w;
            int index = A % B;

            // Diffuse
            int i = material.textureOffset.x + index;
            BitmapBuffer r, g, b;
            r = textures[i];
            g = textures[i + 1];
            b = textures[i + 2];
#ifdef USE_KINECT
            if (material.textureIds.x == KINECT_COLOR_TEXTURE)
            {
                r = textures[index + 2];
                g = textures[index + 1];
                b = textures[index];
            }
#endif // USE_KINECT
            result.x = r / 256.f;
            result.y = g / 256.f;
            result.z = b / 256.f;

            float strength(3.f);
            // Bump mapping
            if (material.textureIds.z != TEXTURE_NONE)
                bumpMap(index, material, textures, intersection, strength);
            // Normal mapping
            if (material.textureIds.y != TEXTURE_NONE)
                normalMap(index, material, textures, normal, strength);
            // Specular mapping
            if (material.textureIds.w != TEXTURE_NONE)
                specularMap(index, material, textures, specular);
            // Reflection mapping
            if (material.advancedTextureIds.x != TEXTURE_NONE)
                reflectionMap(index, material, textures, attributes);
            // Transparency mapping
            if (material.advancedTextureIds.y != TEXTURE_NONE)
                transparencyMap(index, material, textures, attributes);
            // Ambient occulusion mapping
            if (material.advancedTextureIds.z != TEXTURE_NONE)
                ambientOcclusionMap(index, material, textures,
                                    advancedAttributes);
        }
        }
    }
    return result;
}

/*
________________________________________________________________________________

Sphere texture Mapping
________________________________________________________________________________
*/
__device__ __INLINE__ float4 sphereUVMapping(const Primitive &primitive,
                                             Material *materials,
                                             BitmapBuffer *textures,
                                             vec3f &intersection, vec3f &normal,
                                             vec4f &specular, vec4f &attributes,
                                             vec4f &advancedAttributes)
{
    Material &material = materials[primitive.materialId];
    float4 result = material.color;

    vec3f I = normalize(intersection - primitive.p0);
    float U = ((atan2(I.x, I.z) / PI) + 1.f) * .5f;
    float V = (asin(I.y) / PI) + .5f;

    int u = material.textureMapping.x * (U * primitive.vt1.x);
    int v = material.textureMapping.y * (V * primitive.vt1.y);

    if (material.textureMapping.x != 0)
        u = u % material.textureMapping.x;
    if (material.textureMapping.y != 0)
        v = v % material.textureMapping.y;
    if (u >= 0 && u < material.textureMapping.x && v >= 0 &&
        v < material.textureMapping.y)
    {
        int A = (v * material.textureMapping.x + u) * material.textureMapping.w;
        int B = material.textureMapping.x * material.textureMapping.y *
                material.textureMapping.w;
        int index = A % B;

        // Diffuse
        int i = material.textureOffset.x + index;
        BitmapBuffer r, g, b;
        r = textures[i];
        g = textures[i + 1];
        b = textures[i + 2];
        result.x = r / 256.f;
        result.y = g / 256.f;
        result.z = b / 256.f;

        float strength(3.f);
        // Bump mapping
        if (material.textureIds.z != TEXTURE_NONE)
            bumpMap(index, material, textures, intersection, strength);
        // Normal mapping
        if (material.textureIds.y != TEXTURE_NONE)
            normalMap(index, material, textures, normal, strength);
        // Specular mapping
        if (material.textureIds.w != TEXTURE_NONE)
            specularMap(index, material, textures, specular);
        // Reflection mapping
        if (material.advancedTextureIds.x != TEXTURE_NONE)
            reflectionMap(index, material, textures, attributes);
        // Transparency mapping
        if (material.advancedTextureIds.y != TEXTURE_NONE)
            transparencyMap(index, material, textures, attributes);
        // Ambient occulusion mapping
        if (material.advancedTextureIds.z != TEXTURE_NONE)
            ambientOcclusionMap(index, material, textures, advancedAttributes);
    }
    return result;
}

/*
________________________________________________________________________________

Cube texture mapping
________________________________________________________________________________
*/
__device__ __INLINE__ float4 cubeMapping(
    const SceneInfo &sceneInfo, const Primitive &primitive, Material *materials,
    BitmapBuffer *textures, vec3f &intersection, vec3f &normal, vec4f &specular,
    vec4f &attributes, vec4f &advancedAttributes)
{
    Material &material = materials[primitive.materialId];
    float4 result = material.color;

#ifdef USE_KINECT
    if (primitive.type == ptCamera)
    {
        int x = (intersection.x - primitive.p0.x + primitive.size.x) *
                material.textureMapping.x;
        int y = KINECT_COLOR_HEIGHT -
                (intersection.y - primitive.p0.y + primitive.size.y) *
                    material.textureMapping.y;

        x = (x + KINECT_COLOR_WIDTH) % KINECT_COLOR_WIDTH;
        y = (y + KINECT_COLOR_HEIGHT) % KINECT_COLOR_HEIGHT;

        if (x >= 0 && x < KINECT_COLOR_WIDTH && y >= 0 &&
            y < KINECT_COLOR_HEIGHT)
        {
            int index = (y * KINECT_COLOR_WIDTH + x) * KINECT_COLOR_DEPTH;
            index =
                index % (material.textureMapping.x * material.textureMapping.y *
                         material.textureMapping.w);
            BitmapBuffer r = textures[index + 2];
            BitmapBuffer g = textures[index + 1];
            BitmapBuffer b = textures[index + 0];
            result.x = r / 256.f;
            result.y = g / 256.f;
            result.z = b / 256.f;
        }
    }
    else
#endif // USE_KINECT
    {
        int u = ((primitive.type == ptCheckboard) ||
                 (primitive.type == ptXZPlane) || (primitive.type == ptXYPlane))
                    ? (intersection.x - primitive.p0.x + primitive.size.x)
                    : (intersection.z - primitive.p0.z + primitive.size.z);

        int v =
            ((primitive.type == ptCheckboard) || (primitive.type == ptXZPlane))
                ? (intersection.z + primitive.p0.z + primitive.size.z)
                : (intersection.y - primitive.p0.y + primitive.size.y);

        if (material.textureMapping.x != 0)
            u = u % material.textureMapping.x;
        if (material.textureMapping.y != 0)
            v = v % material.textureMapping.y;

        if (u >= 0 && u < material.textureMapping.x && v >= 0 &&
            v < material.textureMapping.x)
        {
            switch (material.textureIds.x)
            {
            case TEXTURE_MANDELBROT:
                mandelbrotSet(primitive, materials, sceneInfo, u, v, result);
                break;
            case TEXTURE_JULIA:
                juliaSet(primitive, materials, sceneInfo, u, v, result);
                break;
            default:
            {
                int A = (v * material.textureMapping.x + u) *
                        material.textureMapping.w;
                int B = material.textureMapping.x * material.textureMapping.y *
                        material.textureMapping.w;
                int index = A % B;
                int i = material.textureOffset.x + index;
                BitmapBuffer r, g, b;
                r = textures[i];
                g = textures[i + 1];
                b = textures[i + 2];
                result.x = r / 256.f;
                result.y = g / 256.f;
                result.z = b / 256.f;

                float strength(3.f);
                // Bump mapping
                if (material.textureIds.z != TEXTURE_NONE)
                    bumpMap(index, material, textures, intersection, strength);
                // Normal mapping
                if (material.textureIds.y != TEXTURE_NONE)
                    normalMap(index, material, textures, normal, strength);
                // Specular mapping
                if (material.textureIds.w != TEXTURE_NONE)
                    specularMap(index, material, textures, specular);
                // Reflection mapping
                if (material.advancedTextureIds.x != TEXTURE_NONE)
                    reflectionMap(index, material, textures, attributes);
                // Transparency mapping
                if (material.advancedTextureIds.y != TEXTURE_NONE)
                    transparencyMap(index, material, textures, attributes);
                // Ambient occulusion mapping
                if (material.advancedTextureIds.z != TEXTURE_NONE)
                    ambientOcclusionMap(index, material, textures,
                                        advancedAttributes);
            }
            break;
            }
        }
    }
    return result;
}

__device__ __INLINE__ bool wireFrameMapping(float x, float y, int width,
                                            const Primitive &primitive)
{
    int X = abs(x);
    int Y = abs(y);
    int A = 100; // TODO
    int B = 100; // TODO
    return (X % A <= width) || (Y % B <= width);
}
