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

// Project
#include "GeometryIntersections.cuh"
#include "TextureMapping.cuh"
#include "VectorUtils.cuh"
#include <Logging.h>
#include <solr.h>

// Device resources
#ifndef USE_MANAGED_MEMORY
BoundingBox* d_boundingBoxes[MAX_GPU_COUNT];
Primitive* d_primitives[MAX_GPU_COUNT];
#endif
Lamp* d_lamps[MAX_GPU_COUNT];
Material* d_materials[MAX_GPU_COUNT];
BitmapBuffer* d_textures[MAX_GPU_COUNT];
LightInformation* d_lightInformation[MAX_GPU_COUNT];
RandomBuffer* d_randoms[MAX_GPU_COUNT];
PostProcessingBuffer* d_postProcessingBuffer[MAX_GPU_COUNT];
BitmapBuffer* d_bitmap[MAX_GPU_COUNT];
PrimitiveXYIdBuffer* d_primitivesXYIds[MAX_GPU_COUNT];
cudaStream_t d_streams[MAX_GPU_COUNT][MAX_STREAM_COUNT];

#define FREECUDARESOURCE(__x)           \
    if (__x != 0)                       \
    {                                   \
        checkCudaErrors(cudaFree(__x)); \
        __x = 0;                        \
    }

__device__ __INLINE__ float4 launchVolumeRendering(
    const int& index, BoundingBox* boundingBoxes, const int& nbActiveBoxes,
    Primitive* primitives, const int& nbActivePrimitives,
    LightInformation* lightInformation, const int& lightInformationSize,
    const int& nbActiveLamps, Material* materials, BitmapBuffer* textures,
    RandomBuffer* randoms, const Ray& ray, const SceneInfo& sceneInfo,
    const PostProcessingInfo& postProcessingInfo, float& depthOfField,
    PrimitiveXYIdBuffer& primitiveXYId)
{
    primitiveXYId.x = -1;
    primitiveXYId.y = 1;
    primitiveXYId.z = 0;
    float4 intersectionColor = intersectionsWithPrimitives(
        index, sceneInfo, boundingBoxes, nbActiveBoxes, primitives,
        nbActivePrimitives, materials, textures, lightInformation,
        lightInformationSize, nbActiveLamps, randoms, postProcessingInfo, ray);
    return intersectionColor;
}

__device__ __INLINE__ float4 launchRayTracing(
    const int& index, BoundingBox* boundingBoxes, const int& nbActiveBoxes,
    Primitive* primitives, const int& nbActivePrimitives,
    LightInformation* lightInformation, const int& lightInformationSize,
    const int& nbActiveLamps, Material* materials, BitmapBuffer* textures,
    RandomBuffer* randoms, const Ray& ray, const SceneInfo& sceneInfo,
    const PostProcessingInfo& postProcessingInfo, float& depthOfField,
    PrimitiveXYIdBuffer& primitiveXYId)
{
    float4 intersectionColor = {0.f, 0.f, 0.f, 0.f};
    vec3f closestIntersection = {0.f, 0.f, 0.f};
    vec3f firstIntersection = {0.f, 0.f, 0.f};
    vec3f normal = {0.f, 0.f, 0.f};
    int closestPrimitive = -1;
    bool carryon = true;
    Ray rayOrigin = ray;
    float initialRefraction = 1.f;
    int iteration = 0;
    primitiveXYId.x = -1;
    primitiveXYId.z = 0;
    primitiveXYId.w = 0;
    int currentMaterialId = -2;

    // TODO
    float colorContributions[NB_MAX_ITERATIONS + 1];
    float4 colors[NB_MAX_ITERATIONS + 1];
    memset(&colorContributions[0], 0, sizeof(float) * (NB_MAX_ITERATIONS + 1));
    memset(&colors[0], 0, sizeof(float4) * (NB_MAX_ITERATIONS + 1));

    float4 recursiveBlinn = {0.f, 0.f, 0.f, 0.f};

    // Variable declarations
    float shadowIntensity = 0.f;
    float4 refractionFromColor;
    vec3f reflectedTarget;
    float4 closestColor = {0.f, 0.f, 0.f, 0.f};
    float4 colorBox = {0.f, 0.f, 0.f, 0.f};
    vec3f latestIntersection = ray.origin;
    float rayLength = 0.f;
    depthOfField = sceneInfo.viewDistance;

    // Reflected rays
    int reflectedRays = -1;
    Ray reflectedRay;
    float reflectedRatio;

    // Global Illumination
    Ray pathTracingRay;
    float pathTracingRatio = 0.f;
    float4 pathTracingColor = {0.f, 0.f, 0.f, 0.f};
    bool processGI = false;

    float4 rBlinn = {0.f, 0.f, 0.f, 0.f};
    int currentMaxIteration =
        (sceneInfo.graphicsLevel < glReflectionsAndRefractions)
            ? 1
            : sceneInfo.nbRayIterations + sceneInfo.pathTracingIteration;
    currentMaxIteration = (currentMaxIteration > NB_MAX_ITERATIONS)
                              ? NB_MAX_ITERATIONS
                              : currentMaxIteration;

    while (iteration < currentMaxIteration &&
           rayLength < sceneInfo.viewDistance && carryon)
    {
        vec3f areas = {0.f, 0.f, 0.f};
        // If no intersection with lamps detected. Now compute intersection with
        // Primitives
        if (carryon)
            carryon = intersectionWithPrimitives(
                sceneInfo, postProcessingInfo, boundingBoxes, nbActiveBoxes,
                primitives, nbActivePrimitives, materials, textures, rayOrigin,
                iteration, closestPrimitive, closestIntersection, normal, areas,
                closestColor, colorBox, currentMaterialId);

        if (carryon)
        {
            currentMaterialId = primitives[closestPrimitive].materialId;

            vec4f attributes;
            attributes.x =
                materials[primitives[closestPrimitive].materialId].reflection;
            attributes.y =
                materials[primitives[closestPrimitive].materialId].transparency;
            attributes.z =
                materials[primitives[closestPrimitive].materialId].refraction;
            attributes.w =
                materials[primitives[closestPrimitive].materialId].opacity;

            if (iteration == 0)
            {
                colors[iteration].x = 0.f;
                colors[iteration].y = 0.f;
                colors[iteration].z = 0.f;
                colors[iteration].w = 0.f;
                colorContributions[iteration] = 1.f;

                firstIntersection = closestIntersection;
                latestIntersection = closestIntersection;
                depthOfField = length(firstIntersection - ray.origin);

                if (materials[primitives[closestPrimitive].materialId]
                            .innerIllumination.x == 0.f &&
                    sceneInfo.advancedIllumination != aiNone)
                {
                    // Global illumination
                    const int t = (index + sceneInfo.timestamp) %
                                  (sceneInfo.size.x * sceneInfo.size.y - 3);
                    pathTracingRay.origin =
                        closestIntersection + normal * sceneInfo.rayEpsilon;
                    pathTracingRay.direction.x = randoms[t];
                    pathTracingRay.direction.y = randoms[t + 1];
                    pathTracingRay.direction.z = randoms[t + 2];
                    pathTracingRay.direction =
                        normalize(pathTracingRay.direction);

                    const float cos_theta =
                        dot(pathTracingRay.direction, normal);
                    if (cos_theta < 0.f)
                        pathTracingRay.direction = -pathTracingRay.direction;
                    pathTracingRay.direction += closestIntersection;
                    pathTracingRatio = (1.f - attributes.y) * abs(cos_theta);
                    processGI = true;
                }

                // Primitive ID for current pixel
                primitiveXYId.x = primitives[closestPrimitive].index;
            }

            // Get object color
            rBlinn.w = attributes.y;
            colors[iteration] =
                primitiveShader(index, sceneInfo, postProcessingInfo,
                                boundingBoxes, nbActiveBoxes, primitives,
                                nbActivePrimitives, lightInformation,
                                lightInformationSize, nbActiveLamps, materials,
                                textures, randoms, rayOrigin.origin, normal,
                                closestPrimitive, closestIntersection, areas,
                                closestColor, iteration, refractionFromColor,
                                shadowIntensity, rBlinn, attributes);

            // Primitive illumination
            Material& material =
                materials[primitives[closestPrimitive].materialId];
            primitiveXYId.z += material.innerIllumination.x * 256;

            float segmentLength =
                length(closestIntersection - latestIntersection);
            latestIntersection = closestIntersection;

            // Refraction
            float transparency = attributes.y;
            float a = 0.f;
            if (attributes.y != 0.f) // Transparency
            {
                // Back of the object? If so, reset refraction to 1.f (air)
                float refraction = attributes.z;

                if (initialRefraction == refraction)
                {
                    // Opacity
                    refraction = 1.f;
                    float length =
                        segmentLength * (attributes.w * (1.f - transparency));
                    rayLength += length;
                    rayLength = (rayLength > sceneInfo.viewDistance)
                                    ? sceneInfo.viewDistance
                                    : rayLength;
                    a = (rayLength / sceneInfo.viewDistance);
                    colors[iteration].x -= a;
                    colors[iteration].y -= a;
                    colors[iteration].z -= a;
                }

                // Actual refraction
                vec3f O_E = normalize(closestIntersection - rayOrigin.origin);
                vectorRefraction(reflectedTarget, O_E, refraction, normal,
                                 initialRefraction);

                colorContributions[iteration] = transparency - a;

                // Prepare next ray
                initialRefraction = refraction;

                if (reflectedRays == -1 && attributes.x != 0.f)
                {
                    vectorReflection(reflectedRay.direction, O_E, normal);
                    reflectedRay.origin =
                        closestIntersection +
                        reflectedRay.direction * sceneInfo.rayEpsilon;
                    reflectedRay.direction =
                        closestIntersection + reflectedRay.direction;
                    reflectedRatio = attributes.x;
                    reflectedRays = iteration;
                }
            }
            else if (attributes.x != 0.f) // Reflection
            {
                vec3f O_E = normalize(closestIntersection - rayOrigin.origin);
                vectorReflection(reflectedTarget, O_E, normal);
                colorContributions[iteration] = attributes.x;
            }
            else
            {
                carryon = false;
                colorContributions[iteration] = 1.f;
            }

            // Contribute to final color
            rBlinn /= (iteration + 1);
            recursiveBlinn.x =
                (rBlinn.x > recursiveBlinn.x) ? rBlinn.x : recursiveBlinn.x;
            recursiveBlinn.y =
                (rBlinn.y > recursiveBlinn.y) ? rBlinn.y : recursiveBlinn.y;
            recursiveBlinn.z =
                (rBlinn.z > recursiveBlinn.z) ? rBlinn.z : recursiveBlinn.z;

            rayOrigin.origin =
                closestIntersection + reflectedTarget * sceneInfo.rayEpsilon;
            rayOrigin.direction = closestIntersection + reflectedTarget;

            // Gloss management
            if (sceneInfo.pathTracingIteration != 0 &&
                materials[primitives[closestPrimitive].materialId].color.w !=
                    0.f)
            {
                // Randomize view
                float ratio =
                    materials[primitives[closestPrimitive].materialId].color.w;
                ratio *= (attributes.y == 0.f) ? 1000.f : 1.f;
                int rindex =
                    (index + sceneInfo.timestamp) % (MAX_BITMAP_SIZE - 3);
                rayOrigin.direction.x += randoms[rindex] * ratio;
                rayOrigin.direction.y += randoms[rindex + 1] * ratio;
                rayOrigin.direction.z += randoms[rindex + 2] * ratio;
            }
        }
        else
        {
            if (sceneInfo.skyboxMaterialId != MATERIAL_NONE)
            {
                colors[iteration] =
                    skyboxMapping(sceneInfo, materials, textures, rayOrigin);
                float rad = colors[iteration].x + colors[iteration].y +
                            colors[iteration].z;
                primitiveXYId.z += (rad > 2.5f) ? rad * 256.f : 0.f;
            }
            else if (sceneInfo.gradientBackground)
            {
                // Background
                vec3f normal = {0.f, 1.f, 0.f};
                vec3f dir = normalize(rayOrigin.direction - rayOrigin.origin);
                float angle = 0.5f - dot(normal, dir);
                angle = (angle > 1.f) ? 1.f : angle;
                colors[iteration] = (1.f - angle) * sceneInfo.backgroundColor;
            }
            else
            {
                colors[iteration] = sceneInfo.backgroundColor;
            }
            colorContributions[iteration] = 1.f;
        }
        iteration++;
    }

    vec3f areas = {0.f, 0.f, 0.f};
    if (sceneInfo.graphicsLevel >= glReflectionsAndRefractions &&
        reflectedRays != -1) // TODO: Draft mode should only test
                             // "sceneInfo.pathTracingIteration==iteration"
        // TODO: Dodgy implementation
        if (intersectionWithPrimitives(sceneInfo, postProcessingInfo,
                                       boundingBoxes, nbActiveBoxes, primitives,
                                       nbActivePrimitives, materials, textures,
                                       reflectedRay, reflectedRays,
                                       closestPrimitive, closestIntersection,
                                       normal, areas, closestColor, colorBox,
                                       currentMaterialId))
        {
            vec4f attributes;
            attributes.x =
                materials[primitives[closestPrimitive].materialId].reflection;
            float4 color = primitiveShader(
                index, sceneInfo, postProcessingInfo, boundingBoxes,
                nbActiveBoxes, primitives, nbActivePrimitives, lightInformation,
                lightInformationSize, nbActiveLamps, materials, textures,
                randoms, reflectedRay.origin, normal, closestPrimitive,
                closestIntersection, areas, closestColor, reflectedRays,
                refractionFromColor, shadowIntensity, rBlinn, attributes);
            colors[reflectedRays] += color * reflectedRatio;

            primitiveXYId.w = shadowIntensity * 255;
        }

    if (processGI && sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        float alphaIntensity = 1.f;
        if (sceneInfo.advancedIllumination == aiFull)
        {
            // Global illumination
            if (intersectionWithPrimitives(
                    sceneInfo, postProcessingInfo, boundingBoxes, nbActiveBoxes,
                    primitives, nbActivePrimitives, materials, textures,
                    pathTracingRay,
                    10, // Only consider nearby geometry (max distance / 10)
                    closestPrimitive, closestIntersection, normal, areas,
                    closestColor, colorBox, MATERIAL_NONE))
            {
                // Ambient occlusion and material emission
                if (primitives[closestPrimitive].materialId != MATERIAL_NONE)
                {
                    Material& material =
                        materials[primitives[closestPrimitive].materialId];
                    const float distanceToPrimitive =
                        length(closestIntersection - pathTracingRay.origin);
                    const float normalizedDistanceToPrimitive =
                        1.f -
                        min(1.f, distanceToPrimitive / sceneInfo.viewDistance);
                    vec4f attributes;
                    pathTracingColor = primitiveShader(
                        index, sceneInfo, postProcessingInfo, boundingBoxes,
                        nbActiveBoxes, primitives, nbActivePrimitives,
                        lightInformation, lightInformationSize, nbActiveLamps,
                        materials, textures, randoms, pathTracingRay.origin,
                        normal, closestPrimitive, closestIntersection, areas,
                        closestColor, iteration, refractionFromColor,
                        shadowIntensity, rBlinn, attributes);
                    alphaIntensity -= sceneInfo.shadowIntensity *
                                      normalizedDistanceToPrimitive;
                    pathTracingRatio *= (MATERIAL_DEFAULT_EMMISION_STRENGTH +
                                         material.innerIllumination.x) *
                                        normalizedDistanceToPrimitive;
                }
            }
            else if (sceneInfo.skyboxMaterialId != MATERIAL_NONE)
            {
                // Background
                pathTracingColor = skyboxMapping(sceneInfo, materials, textures,
                                                 pathTracingRay);
                pathTracingRatio *= SKYBOX_LUNINANCE_STRENGTH;
            }
        }
        else if (sceneInfo.skyboxMaterialId != MATERIAL_NONE)
        {
            // Background
            pathTracingColor =
                skyboxMapping(sceneInfo, materials, textures, pathTracingRay);
            pathTracingRatio *= SKYBOX_LUNINANCE_STRENGTH;
        }

        colors[0] =
            colors[0] * alphaIntensity + pathTracingColor * pathTracingRatio;
    }

    for (int i = iteration - 2; i >= 0; --i)
        colors[i] = colors[i] * (1.f - colorContributions[i]) +
                    colors[i + 1] * colorContributions[i];
    intersectionColor = colors[0];
    intersectionColor += recursiveBlinn;

    // Background color
    float D1 = sceneInfo.viewDistance * 0.95f;
    if (sceneInfo.atmosphericEffect == aeFog && depthOfField > D1)
    {
        float D2 = sceneInfo.viewDistance * 0.05f;
        float a = depthOfField - D1;
        float b = 1.f - (a / D2);
        intersectionColor =
            intersectionColor * b + sceneInfo.backgroundColor * (1.f - b);
    }

    // Primitive information
    primitiveXYId.y = iteration;

    // Depth of field
    intersectionColor -= colorBox;

    // Ambient light
    return intersectionColor;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------------
 * \brief      This kernel processes a "standard" image, meaning that the screen
 * is a single image for which every pixel is a ray of light entering the same
 * camera.
 * ------------------------------------------------------------------------------------------------------------------------
 * \param[in]  occupancyParameters Contains the number of GPUs and streams
 * involded in the GPU processing \param[in]  device_split Y coordinate from
 * where the current GPU should start working \param[in]  stream_split Y
 * coordinate from where the current stream should start working \param[in]
 * BoundingBoxes Pointer to the array of bounding boxes \param[in] nbActiveBoxes
 * Number of bounding boxes \param[in]  primitives Pointer to the array of
 * primitives \param[in]  nbActivePrimitives Number of primitives \param[in]
 * lightInformation Pointer to the array of light positions and intensities
 * (Used for global illumination) \param[in]  lightInformationSize Number of
 * lights \param[in]  nbActiveLamps Number of lamps \param[in]  materials
 * Pointer to the array of materials \param[in]  textures Pointer to the array
 * of textures \param[in]  randoms Pointer to the array of random floats (GPUs
 * are not good at generating numbers, done by the CPU) \param[in]  origin
 * Camera position \param[in]  direction Camera LookAt \param[in]  angles Angles
 * applied to the camera. The rotation center is {0,0,0} \param[in]  sceneInfo
 * Information about the scene and environment \param[in]  postProcessingInfo
 * Information about PostProcessing effect \param[out] postProcessingBuffer
 * Pointer to the output array of color information \param[out] primitiveXYIds
 * Pointer to the array containing the Id of the primitivive for each pixel
 * ------------------------------------------------------------------------------------------------------------------------
 */
__global__ void k_standardRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    BoundingBox* BoundingBoxes, int nbActiveBoxes, Primitive* primitives,
    int nbActivePrimitives, LightInformation* lightInformation,
    int lightInformationSize, int nbActiveLamps, Material* materials,
    BitmapBuffer* textures, RandomBuffer* randoms, vec3f origin,
    vec3f direction, vec4f angles, SceneInfo sceneInfo,
    PostProcessingInfo postProcessingInfo,
    PostProcessingBuffer* postProcessingBuffer,
    PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = (stream_split + y) * sceneInfo.size.x + x;

    // Antialisazing
    float2 AArotatedGrid[4] = {{3.f, 5.f},
                               {5.f, -3.f},
                               {-3.f, -5.f},
                               {-5.f, 3.f}};

    // Beware out of bounds error! \[^_^]/
    // And only process pixels that need extra rendering
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS))
        return;

    Ray ray;
    ray.origin = origin;
    ray.direction = direction;

    vec3f rotationCenter = {0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    bool antialiasingActivated = (sceneInfo.cameraType == ctAntialiazed);

#ifdef NATURAL_DEPTHOFFIELD
    if (postProcessingInfo.type != ppe_depthOfField &&
        sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        // Randomize view for natural depth of field
        float a = (postProcessingInfo.param1 / 20000.f);
        int rindex = index + sceneInfo.timestamp % (MAX_BITMAP_SIZE - 2);
        ray.origin.x +=
            randoms[rindex] * postProcessingBuffer[index].colorInfo.w * a;
        ray.origin.y +=
            randoms[rindex + 1] * postProcessingBuffer[index].colorInfo.w * a;
    }
#endif // NATURAL_DEPTHOFFIELD

    float dof = 0.f;
    if (sceneInfo.cameraType == ctOrthographic)
    {
        ray.direction.x = ray.origin.z * 0.001f * (x - (sceneInfo.size.x / 2));
        ray.direction.y =
            -ray.origin.z * 0.001f *
            (device_split + stream_split + y - (sceneInfo.size.y / 2));
        ray.origin.x = ray.direction.x;
        ray.origin.y = ray.direction.y;
    }
    else
    {
        float ratio = (float)sceneInfo.size.x / (float)sceneInfo.size.y;
        float2 step;
        step.x = ratio * angles.w / (float)sceneInfo.size.x;
        step.y = angles.w / (float)sceneInfo.size.y;
        ray.direction.x =
            ray.direction.x - step.x * (x - (sceneInfo.size.x / 2));
        ray.direction.y =
            ray.direction.y +
            step.y * (device_split + stream_split + y - (sceneInfo.size.y / 2));
    }

    vectorRotation(ray.origin, rotationCenter, angles);
    vectorRotation(ray.direction, rotationCenter, angles);

    float4 color = {0.f, 0.f, 0.f, 0.f};
    Ray r = ray;
    if (antialiasingActivated)
        for (int I = 0; I < 4; ++I)
        {
            r.origin.x += AArotatedGrid[I].x;
            r.origin.y += AArotatedGrid[I].y;
            float4 c;
            c = launchRayTracing(index, BoundingBoxes, nbActiveBoxes,
                                 primitives, nbActivePrimitives,
                                 lightInformation, lightInformationSize,
                                 nbActiveLamps, materials, textures, randoms, r,
                                 sceneInfo, postProcessingInfo, dof,
                                 primitiveXYIds[index]);
            color += c;
        }
    else if (sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        // Antialiazing
        r.direction.x += AArotatedGrid[sceneInfo.pathTracingIteration % 4].x;
        r.direction.y += AArotatedGrid[sceneInfo.pathTracingIteration % 4].y;
        // r.origin.x += AArotatedGrid[sceneInfo.pathTracingIteration%4].x;
        // r.origin.y += AArotatedGrid[sceneInfo.pathTracingIteration%4].y;
    }
    color += launchRayTracing(index, BoundingBoxes, nbActiveBoxes, primitives,
                              nbActivePrimitives, lightInformation,
                              lightInformationSize, nbActiveLamps, materials,
                              textures, randoms, r, sceneInfo,
                              postProcessingInfo, dof, primitiveXYIds[index]);

    if (sceneInfo.advancedIllumination == aiRandomIllumination)
    {
        // Randomize light intensity
        int rindex = (index + sceneInfo.timestamp) % MAX_BITMAP_SIZE;
        color += sceneInfo.backgroundColor * randoms[rindex] * 5.f;
    }

    if (antialiasingActivated)
        color /= 5.f;

    if (sceneInfo.pathTracingIteration == 0)
        postProcessingBuffer[index].colorInfo.w = dof;

    if (sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS)
    {
        postProcessingBuffer[index].colorInfo.x = color.x;
        postProcessingBuffer[index].colorInfo.y = color.y;
        postProcessingBuffer[index].colorInfo.z = color.z;

        postProcessingBuffer[index].sceneInfo.x = color.x;
        postProcessingBuffer[index].sceneInfo.y = color.y;
        postProcessingBuffer[index].sceneInfo.z = color.z;
    }
    else
    {
        postProcessingBuffer[index].sceneInfo.x =
            (primitiveXYIds[index].z > 0)
                ? max(postProcessingBuffer[index].sceneInfo.x, color.x)
                : color.x;
        postProcessingBuffer[index].sceneInfo.y =
            (primitiveXYIds[index].z > 0)
                ? max(postProcessingBuffer[index].sceneInfo.y, color.y)
                : color.y;
        postProcessingBuffer[index].sceneInfo.z =
            (primitiveXYIds[index].z > 0)
                ? max(postProcessingBuffer[index].sceneInfo.z, color.z)
                : color.z;

        postProcessingBuffer[index].colorInfo.x +=
            postProcessingBuffer[index].sceneInfo.x;
        postProcessingBuffer[index].colorInfo.y +=
            postProcessingBuffer[index].sceneInfo.y;
        postProcessingBuffer[index].colorInfo.z +=
            postProcessingBuffer[index].sceneInfo.z;
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------------
 * \brief      This kernel processes a "standard" image, meaning that the screen
 * is a single image for which every pixel is a ray of light entering the same
 * camera.
 * ------------------------------------------------------------------------------------------------------------------------
 * \param[in]  occupancyParameters Contains the number of GPUs and streams
 * involded in the GPU processing \param[in]  device_split Y coordinate from
 * where the current GPU should start working \param[in]  stream_split Y
 * coordinate from where the current stream should start working \param[in]
 * BoundingBoxes Pointer to the array of bounding boxes \param[in] nbActiveBoxes
 * Number of bounding boxes \param[in]  primitives Pointer to the array of
 * primitives \param[in]  nbActivePrimitives Number of primitives \param[in]
 * lightInformation Pointer to the array of light positions and intensities
 * (Used for global illumination) \param[in]  lightInformationSize Number of
 * lights \param[in]  nbActiveLamps Number of lamps \param[in]  materials
 * Pointer to the array of materials \param[in]  textures Pointer to the array
 * of textures \param[in]  randoms Pointer to the array of random floats (GPUs
 * are not good at generating numbers, done by the CPU) \param[in]  origin
 * Camera position \param[in]  direction Camera LookAt \param[in]  angles Angles
 * applied to the camera. The rotation center is {0,0,0} \param[in]  sceneInfo
 * Information about the scene and environment \param[in]  postProcessingInfo
 * Information about PostProcessing effect \param[out] postProcessingBuffer
 * Pointer to the output array of color information \param[out] primitiveXYIds
 * Pointer to the array containing the Id of the primitivive for each pixel
 * ------------------------------------------------------------------------------------------------------------------------
 */
__global__ void k_volumeRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    BoundingBox* BoundingBoxes, int nbActiveBoxes, Primitive* primitives,
    int nbActivePrimitives, LightInformation* lightInformation,
    int lightInformationSize, int nbActiveLamps, Material* materials,
    BitmapBuffer* textures, RandomBuffer* randoms, vec3f origin,
    vec3f direction, vec4f angles, SceneInfo sceneInfo,
    PostProcessingInfo postProcessingInfo,
    PostProcessingBuffer* postProcessingBuffer,
    PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = (stream_split + y) * sceneInfo.size.x + x;

    // Antialisazing
    float2 AArotatedGrid[4] = {{3.f, 5.f},
                               {5.f, -3.f},
                               {-3.f, -5.f},
                               {-5.f, 3.f}};

    // Beware out of bounds error! \[^_^]/
    // And only process pixels that need extra rendering
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS))
        return;

    Ray ray;
    ray.origin = origin;
    ray.direction = direction;

    vec3f rotationCenter = {0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    bool antialiasingActivated = (sceneInfo.cameraType == ctAntialiazed);

    if (postProcessingInfo.type != ppe_depthOfField &&
        sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        // Randomize view for natural depth of field
        float a = (postProcessingInfo.param1 / 20000.f);
        int rindex = index + sceneInfo.timestamp % (MAX_BITMAP_SIZE - 2);
        ray.origin.x +=
            randoms[rindex] * postProcessingBuffer[index].colorInfo.w * a;
        ray.origin.y +=
            randoms[rindex + 1] * postProcessingBuffer[index].colorInfo.w * a;
    }

    float dof = 0.f;
    if (sceneInfo.cameraType == ctOrthographic)
    {
        ray.direction.x = ray.origin.z * 0.001f * (x - (sceneInfo.size.x / 2));
        ray.direction.y =
            -ray.origin.z * 0.001f *
            (device_split + stream_split + y - (sceneInfo.size.y / 2));
        ray.origin.x = ray.direction.x;
        ray.origin.y = ray.direction.y;
    }
    else
    {
        float ratio = (float)sceneInfo.size.x / (float)sceneInfo.size.y;
        float2 step;
        step.x = ratio * angles.w / (float)sceneInfo.size.x;
        step.y = angles.w / (float)sceneInfo.size.y;
        ray.direction.x =
            ray.direction.x - step.x * (x - (sceneInfo.size.x / 2));
        ray.direction.y =
            ray.direction.y +
            step.y * (device_split + stream_split + y - (sceneInfo.size.y / 2));
    }

    vectorRotation(ray.origin, rotationCenter, angles);
    vectorRotation(ray.direction, rotationCenter, angles);

    float4 color = {0.f, 0.f, 0.f, 0.f};
    Ray r = ray;
    if (antialiasingActivated)
        for (int I = 0; I < 4; ++I)
        {
            r.direction.x = ray.direction.x + AArotatedGrid[I].x;
            r.direction.y = ray.direction.y + AArotatedGrid[I].y;
            float4 c;
            c = launchVolumeRendering(index, BoundingBoxes, nbActiveBoxes,
                                      primitives, nbActivePrimitives,
                                      lightInformation, lightInformationSize,
                                      nbActiveLamps, materials, textures,
                                      randoms, r, sceneInfo, postProcessingInfo,
                                      dof, primitiveXYIds[index]);
            color += c;
        }
    else
    {
        r.direction.x = ray.direction.x +
                        AArotatedGrid[sceneInfo.pathTracingIteration % 4].x;
        r.direction.y = ray.direction.y +
                        AArotatedGrid[sceneInfo.pathTracingIteration % 4].y;
    }
    color +=
        launchVolumeRendering(index, BoundingBoxes, nbActiveBoxes, primitives,
                              nbActivePrimitives, lightInformation,
                              lightInformationSize, nbActiveLamps, materials,
                              textures, randoms, r, sceneInfo,
                              postProcessingInfo, dof, primitiveXYIds[index]);

    if (sceneInfo.advancedIllumination == aiRandomIllumination)
    {
        // Randomize light intensity
        int rindex = (index + sceneInfo.timestamp) % MAX_BITMAP_SIZE;
        color += sceneInfo.backgroundColor * randoms[rindex] * 5.f;
    }

    if (antialiasingActivated)
        color /= 5.f;

    if (sceneInfo.pathTracingIteration == 0)
        postProcessingBuffer[index].colorInfo.w = dof;

    if (sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS)
    {
        postProcessingBuffer[index].colorInfo.x = color.x;
        postProcessingBuffer[index].colorInfo.y = color.y;
        postProcessingBuffer[index].colorInfo.z = color.z;

        postProcessingBuffer[index].sceneInfo.x = color.x;
        postProcessingBuffer[index].sceneInfo.y = color.y;
        postProcessingBuffer[index].sceneInfo.z = color.z;
    }
    else
    {
        postProcessingBuffer[index].sceneInfo.x =
            (primitiveXYIds[index].z > 0)
                ? max(postProcessingBuffer[index].sceneInfo.x, color.x)
                : color.x;
        postProcessingBuffer[index].sceneInfo.y =
            (primitiveXYIds[index].z > 0)
                ? max(postProcessingBuffer[index].sceneInfo.y, color.y)
                : color.y;
        postProcessingBuffer[index].sceneInfo.z =
            (primitiveXYIds[index].z > 0)
                ? max(postProcessingBuffer[index].sceneInfo.z, color.z)
                : color.z;

        postProcessingBuffer[index].colorInfo.x +=
            postProcessingBuffer[index].sceneInfo.x;
        postProcessingBuffer[index].colorInfo.y +=
            postProcessingBuffer[index].sceneInfo.y;
        postProcessingBuffer[index].colorInfo.z +=
            postProcessingBuffer[index].sceneInfo.z;
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------------
 * \brief      This kernel processes a fisheye image
 * ------------------------------------------------------------------------------------------------------------------------
 * \param[in]  occupancyParameters Contains the number of GPUs and streams
 * involded in the GPU processing \param[in]  device_split Y coordinate from
 * where the current GPU should start working \param[in]  stream_split Y
 * coordinate from where the current stream should start working \param[in]
 * BoundingBoxes Pointer to the array of bounding boxes \param[in] nbActiveBoxes
 * Number of bounding boxes \param[in]  primitives Pointer to the array of
 * primitives \param[in]  nbActivePrimitives Number of primitives \param[in]
 * lightInformation Pointer to the array of light positions and intensities
 * (Used for global illumination) \param[in]  lightInformationSize Number of
 * lights \param[in]  nbActiveLamps Number of lamps \param[in]  materials
 * Pointer to the array of materials \param[in]  textures Pointer to the array
 * of textures \param[in]  randoms Pointer to the array of random floats (GPUs
 * are not good at generating numbers, done by the CPU) \param[in]  origin
 * Camera position \param[in]  direction Camera LookAt \param[in]  angles Angles
 * applied to the camera. The rotation center is {0,0,0} \param[in]  sceneInfo
 * Information about the scene and environment \param[in]  postProcessingInfo
 * Information about PostProcessing effect \param[out] postProcessingBuffer
 * Pointer to the output array of color information \param[out] primitiveXYIds
 * Pointer to the array containing the Id of the primitivive for each pixel
 * ------------------------------------------------------------------------------------------------------------------------
 */
__global__ void k_fishEyeRenderer(
    const int2 occupancyParameters, int split_y, BoundingBox* BoundingBoxes,
    int nbActiveBoxes, Primitive* primitives, int nbActivePrimitives,
    LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, Material* materials, BitmapBuffer* textures,
    RandomBuffer* randoms, vec3f origin, vec3f direction, vec4f angles,
    SceneInfo sceneInfo, PostProcessingInfo postProcessingInfo,
    PostProcessingBuffer* postProcessingBuffer,
    PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    // And only process pixels that need extra rendering
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS))
        return;

    Ray ray;
    ray.origin = origin;
    ray.direction = direction;

    // Randomize view for natural depth of field
    if (sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        int rindex = (index + sceneInfo.timestamp) % (MAX_BITMAP_SIZE - 3);
        float a = float(sceneInfo.pathTracingIteration) /
                  float(sceneInfo.maxPathTracingIterations);
        ray.direction.x += randoms[rindex] *
                           postProcessingBuffer[index].colorInfo.w *
                           postProcessingInfo.param2 * a;
        ray.direction.y += randoms[rindex + 1] *
                           postProcessingBuffer[index].colorInfo.w *
                           postProcessingInfo.param2 * a;
        ray.direction.z += randoms[rindex + 2] *
                           postProcessingBuffer[index].colorInfo.w *
                           postProcessingInfo.param2 * a;
    }

    float dof = 0.f;

    // Normal Y axis
    float2 step;
    step.y = angles.w / (float)sceneInfo.size.y;
    ray.direction.y = ray.direction.y +
                      step.y * (float)(split_y + y - (sceneInfo.size.y / 2));

    // 360° X axis
    step.x = 2.f * PI / sceneInfo.size.x;
    step.y = 2.f * PI / sceneInfo.size.y;

    vec4f fishEyeAngles = {0.f, 0.f, 0.f, 0.f};
    fishEyeAngles.y = angles.y + step.x * (float)x;

    vectorRotation(ray.direction, ray.origin, fishEyeAngles);

    float4 color = {0.f, 0.f, 0.f, 0.f};
    color += launchRayTracing(index, BoundingBoxes, nbActiveBoxes, primitives,
                              nbActivePrimitives, lightInformation,
                              lightInformationSize, nbActiveLamps, materials,
                              textures, randoms, ray, sceneInfo,
                              postProcessingInfo, dof, primitiveXYIds[index]);

    if (sceneInfo.pathTracingIteration == 0)
        postProcessingBuffer[index].colorInfo.w = dof;

    if (sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS)
    {
        postProcessingBuffer[index].colorInfo.x = color.x;
        postProcessingBuffer[index].colorInfo.y = color.y;
        postProcessingBuffer[index].colorInfo.z = color.z;
    }
    else
    {
        postProcessingBuffer[index].colorInfo.x += color.x;
        postProcessingBuffer[index].colorInfo.y += color.y;
        postProcessingBuffer[index].colorInfo.z += color.z;
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------------
 * \brief      This kernel processes an anaglyph image. The
 * sceneInfo.eyeSeparation parameter specifies the distance between both eyes.
 * ------------------------------------------------------------------------------------------------------------------------
 * \param[in]  occupancyParameters Contains the number of GPUs and streams
 * involded in the GPU processing \param[in]  BoundingBoxes Pointer to the array
 * of bounding boxes \param[in]  nbActiveBoxes Number of bounding boxes
 * \param[in]  primitives Pointer to the array of primitives
 * \param[in]  nbActivePrimitives Number of primitives
 * \param[in]  lightInformation Pointer to the array of light positions and
 * intensities (Used for global illumination) \param[in]  lightInformationSize
 * Number of lights \param[in]  nbActiveLamps Number of lamps \param[in]
 * materials Pointer to the array of materials \param[in]  textures Pointer to
 * the array of textures \param[in]  randoms Pointer to the array of random
 * floats (GPUs are not good at generating numbers, done by the CPU) \param[in]
 * origin Camera position \param[in]  direction Camera LookAt \param[in]  angles
 * Angles applied to the camera. The rotation center is {0,0,0} \param[in]
 * sceneInfo Information about the scene and environment \param[in]
 * postProcessingInfo Information about PostProcessing effect \param[out]
 * postProcessingBuffer Pointer to the output array of color information
 * \param[out] primitiveXYIds Pointer to the array containing the Id of the
 * primitivive for each pixel
 * ------------------------------------------------------------------------------------------------------------------------
 */
__global__ void k_anaglyphRenderer(
    const int2 occupancyParameters, BoundingBox* boundingBoxes,
    int nbActiveBoxes, Primitive* primitives, int nbActivePrimitives,
    LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, Material* materials, BitmapBuffer* textures,
    RandomBuffer* randoms, vec3f origin, vec3f direction, vec4f angles,
    SceneInfo sceneInfo, PostProcessingInfo postProcessingInfo,
    PostProcessingBuffer* postProcessingBuffer,
    PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    // And only process pixels that need extra rendering
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS))
        return;

    vec3f rotationCenter = {0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    float dof = 0.f;
    Ray eyeRay;

    float ratio = (float)sceneInfo.size.x / (float)sceneInfo.size.y;
    float2 step;
    step.x = ratio * angles.w / (float)sceneInfo.size.x;
    step.y = angles.w / (float)sceneInfo.size.y;

    // Left eye
    eyeRay.origin.x = origin.x - sceneInfo.eyeSeparation;
    eyeRay.origin.y = origin.y;
    eyeRay.origin.z = origin.z;

    eyeRay.direction.x =
        direction.x - step.x * (float)(x - (sceneInfo.size.x / 2));
    eyeRay.direction.y =
        direction.y + step.y * (float)(y - (sceneInfo.size.y / 2));
    eyeRay.direction.z = direction.z;

    vectorRotation(eyeRay.origin, rotationCenter, angles);
    vectorRotation(eyeRay.direction, rotationCenter, angles);

    float4 colorLeft =
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, eyeRay, sceneInfo,
                         postProcessingInfo, dof, primitiveXYIds[index]);

    // Right eye
    eyeRay.origin.x = origin.x + sceneInfo.eyeSeparation;
    eyeRay.origin.y = origin.y;
    eyeRay.origin.z = origin.z;

    eyeRay.direction.x =
        direction.x - step.x * (float)(x - (sceneInfo.size.x / 2));
    eyeRay.direction.y =
        direction.y + step.y * (float)(y - (sceneInfo.size.y / 2));
    eyeRay.direction.z = direction.z;

    vectorRotation(eyeRay.origin, rotationCenter, angles);
    vectorRotation(eyeRay.direction, rotationCenter, angles);

    float4 colorRight =
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, eyeRay, sceneInfo,
                         postProcessingInfo, dof, primitiveXYIds[index]);

    float r1 =
        colorLeft.x * 0.299f + colorLeft.y * 0.587f + colorLeft.z * 0.114f;
    float b1 = 0.f;
    float g1 = 0.f;

    float r2 = 0.f;
    float g2 = colorRight.y;
    float b2 = colorRight.z;

    if (sceneInfo.pathTracingIteration == 0)
        postProcessingBuffer[index].colorInfo.w = dof;

    if (sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS)
    {
        postProcessingBuffer[index].colorInfo.x = r1 + r2;
        postProcessingBuffer[index].colorInfo.y = g1 + g2;
        postProcessingBuffer[index].colorInfo.z = b1 + b2;
    }
    else
    {
        postProcessingBuffer[index].colorInfo.x += r1 + r2;
        postProcessingBuffer[index].colorInfo.y += g1 + g2;
        postProcessingBuffer[index].colorInfo.z += b1 + b2;
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------------
 * \brief      This kernel processes two images in a side-by-side format. The
 * sceneInfo.eyeSeparation parameter specifies the distance between both eyes.
 * ------------------------------------------------------------------------------------------------------------------------
 * \param[in]  occupancyParameters Contains the number of GPUs and streams
 * involded in the GPU processing \param[in]  BoundingBoxes Pointer to the array
 * of bounding boxes \param[in]  nbActiveBoxes Number of bounding boxes
 * \param[in]  primitives Pointer to the array of primitives
 * \param[in]  nbActivePrimitives Number of primitives
 * \param[in]  lightInformation Pointer to the array of light positions and
 * intensities (Used for global illumination) \param[in]  lightInformationSize
 * Number of lights \param[in]  nbActiveLamps Number of lamps \param[in]
 * materials Pointer to the array of materials \param[in]  textures Pointer to
 * the array of textures \param[in]  randoms Pointer to the array of random
 * floats (GPUs are not good at generating numbers, done by the CPU) \param[in]
 * origin Camera position \param[in]  direction Camera LookAt \param[in]  angles
 * Angles applied to the camera. The rotation center is {0,0,0} \param[in]
 * sceneInfo Information about the scene and environment \param[in]
 * postProcessingInfo Information about PostProcessing effect \param[out]
 * postProcessingBuffer Pointer to the output array of color information
 * \param[out] primitiveXYIds Pointer to the array containing the Id of the
 * primitivive for each pixel
 * ------------------------------------------------------------------------------------------------------------------------
 */
__global__ void k_3DVisionRenderer(
    const int2 occupancyParameters, BoundingBox* boundingBoxes,
    int nbActiveBoxes, Primitive* primitives, int nbActivePrimitives,
    LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, Material* materials, BitmapBuffer* textures,
    RandomBuffer* randoms, vec3f origin, vec3f direction, vec4f angles,
    SceneInfo sceneInfo, PostProcessingInfo postProcessingInfo,
    PostProcessingBuffer* postProcessingBuffer,
    PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    // And only process pixels that need extra rendering
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS))
        return;

    float focus =
        fabs(postProcessingBuffer[sceneInfo.size.x / 2 * sceneInfo.size.y / 2]
                 .colorInfo.w -
             origin.z);
    float eyeSeparation = sceneInfo.eyeSeparation * (direction.z / focus);

    vec3f rotationCenter = {0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    float dof = postProcessingInfo.param1;
    int halfWidth = sceneInfo.size.x / 2;

    float ratio = (float)sceneInfo.size.x / (float)sceneInfo.size.y;
    float2 step;
    step.x = ratio * angles.w / (float)sceneInfo.size.x;
    step.y = angles.w / (float)sceneInfo.size.y;

    Ray eyeRay;
    if (x < halfWidth)
    {
        // Left eye
        eyeRay.origin.x = origin.x + eyeSeparation;
        eyeRay.origin.y = origin.y;
        eyeRay.origin.z = origin.z;

        eyeRay.direction.x =
            direction.x -
            step.x * (float)(x - (sceneInfo.size.x / 2) + halfWidth / 2) +
            sceneInfo.eyeSeparation;
        eyeRay.direction.y =
            direction.y + step.y * (float)(y - (sceneInfo.size.y / 2));
        eyeRay.direction.z = direction.z;
    }
    else
    {
        // Right eye
        eyeRay.origin.x = origin.x - eyeSeparation;
        eyeRay.origin.y = origin.y;
        eyeRay.origin.z = origin.z;

        eyeRay.direction.x =
            direction.x -
            step.x * (float)(x - (sceneInfo.size.x / 2) - halfWidth / 2) -
            sceneInfo.eyeSeparation;
        eyeRay.direction.y =
            direction.y + step.y * (float)(y - (sceneInfo.size.y / 2));
        eyeRay.direction.z = direction.z;
    }

    vectorRotation(eyeRay.origin, rotationCenter, angles);
    vectorRotation(eyeRay.direction, rotationCenter, angles);

    float4 color =
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, eyeRay, sceneInfo,
                         postProcessingInfo, dof, primitiveXYIds[index]);

    if (sceneInfo.advancedIllumination == aiRandomIllumination)
    {
        // Randomize light intensity
        int rindex = (index + sceneInfo.timestamp) % MAX_BITMAP_SIZE;
        color += sceneInfo.backgroundColor * randoms[rindex] * 5.f;
    }

    // Contribute to final image
    if (sceneInfo.pathTracingIteration == 0)
        postProcessingBuffer[index].colorInfo.w = dof;

    if (sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS)
    {
        postProcessingBuffer[index].colorInfo.x = color.x;
        postProcessingBuffer[index].colorInfo.y = color.y;
        postProcessingBuffer[index].colorInfo.z = color.z;
    }
    else
    {
        postProcessingBuffer[index].colorInfo.x += color.x;
        postProcessingBuffer[index].colorInfo.y += color.y;
        postProcessingBuffer[index].colorInfo.z += color.z;
    }
}

/*!
 * -------------------------------------------------------------------------------------------------
 * \brief      This post-processing kernel simply converts the contents of the
 * postProcessingBuffer into a bitmap
 * -------------------------------------------------------------------------------------------------
 * \param[in]  occupancyParameters Contains the number of GPUs and streams
 * involded in the GPU processing \param[in]  sceneInfo Information about the
 * scene and environment \param[in]  postProcessingInfo Information about
 * PostProcessing effect \param[in]  postProcessingBuffer Pointer to the output
 * array of color information \param[out] Bitmap Pointer to a bitmap. The bitmap
 * is encoded according to the value of the sceneInfo.frameBufferType parameter
 * -------------------------------------------------------------------------------------------------
 */
__global__ void k_default(const int2 occupancyParameters, SceneInfo sceneInfo,
                          PostProcessingInfo PostProcessingInfo,
                          PostProcessingBuffer* postProcessingBuffer,
                          BitmapBuffer* bitmap)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float4 localColor = postProcessingBuffer[index].colorInfo;
    if (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
        localColor /=
            (float)(sceneInfo.pathTracingIteration - NB_MAX_ITERATIONS + 1);

    makeColor(sceneInfo, localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Depth of field
________________________________________________________________________________
*/
__global__ void k_depthOfField(const int2 occupancyParameters,
                               SceneInfo sceneInfo,
                               PostProcessingInfo postProcessingInfo,
                               PostProcessingBuffer* postProcessingBuffer,
                               RandomBuffer* randoms, BitmapBuffer* bitmap)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float4 localColor = {0.f, 0.f, 0.f};
    float depth = fabs(postProcessingBuffer[index].colorInfo.w -
                       postProcessingInfo.param1) /
                  sceneInfo.viewDistance;
    int wh = sceneInfo.size.x * sceneInfo.size.y;

    for (int i = 0; i < postProcessingInfo.param3; ++i)
    {
        int ix = i % wh;
        int iy = (i + 1000) % wh;
        int xx = x + depth * randoms[ix] * postProcessingInfo.param2;
        int yy = y + depth * randoms[iy] * postProcessingInfo.param2;
        if (xx >= 0 && xx < sceneInfo.size.x && yy >= 0 &&
            yy < sceneInfo.size.y)
        {
            int localIndex = yy * sceneInfo.size.x + xx;
            if (localIndex >= 0 && localIndex < wh)
                localColor += postProcessingBuffer[localIndex].colorInfo;
        }
        else
            localColor += postProcessingBuffer[index].colorInfo;
    }
    localColor /= postProcessingInfo.param3;

    if (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
        localColor /=
            (float)(sceneInfo.pathTracingIteration - NB_MAX_ITERATIONS + 1);

    localColor.w = 1.f;

    makeColor(sceneInfo, localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Ambiant Occlusion
________________________________________________________________________________
*/
__global__ void k_ambiantOcclusion(const int2 occupancyParameters,
                                   SceneInfo sceneInfo,
                                   PostProcessingInfo postProcessingInfo,
                                   PostProcessingBuffer* postProcessingBuffer,
                                   RandomBuffer* randoms, BitmapBuffer* bitmap)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    int wh = sceneInfo.size.x * sceneInfo.size.y;
    float occ = 0.f;
    float4 localColor = postProcessingBuffer[index].colorInfo;
    float depth = localColor.w;
    const int step = 16;
    int i = 0;
    float c = 0.f;
    for (int X = -step; X < step; X += 2)
        for (int Y = -step; Y < step; Y += 2)
        {
            int ix = i % wh;
            int iy = (i + 100) % wh;
            ++i;
            c += 1.f;
            int xx = x + (X * postProcessingInfo.param2 * randoms[ix] / 10.f);
            int yy = y + (Y * postProcessingInfo.param2 * randoms[iy] / 10.f);
            if (xx >= 0 && xx < sceneInfo.size.x && yy >= 0 &&
                yy < sceneInfo.size.y)
            {
                int localIndex = yy * sceneInfo.size.x + xx;
                if (postProcessingBuffer[localIndex].colorInfo.w >= depth)
                    occ += 1.f;
            }
            else
                occ += 1.f;
        }

    occ /= (float)c;
    occ += 0.3f; // Ambient light
    if (occ < 1.f)
    {
        localColor.x *= occ;
        localColor.y *= occ;
        localColor.z *= occ;
    }
    if (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
        localColor /=
            (float)(sceneInfo.pathTracingIteration - NB_MAX_ITERATIONS + 1);

    saturateVector(localColor);
    localColor.w = 1.f;

    makeColor(sceneInfo, localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Radiosity
________________________________________________________________________________
*/
__global__ void k_radiosity(const int2 occupancyParameters, SceneInfo sceneInfo,
                            PostProcessingInfo postProcessingInfo,
                            PrimitiveXYIdBuffer* primitiveXYIds,
                            PostProcessingBuffer* postProcessingBuffer,
                            RandomBuffer* randoms, BitmapBuffer* bitmap)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    int wh = sceneInfo.size.x * sceneInfo.size.y;

    int div = (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
                  ? (sceneInfo.pathTracingIteration - NB_MAX_ITERATIONS + 1)
                  : 1;

    float4 localColor = {0.f, 0.f, 0.f, 0.f};
    for (int i = 0; i < postProcessingInfo.param3; ++i)
    {
        int ix = (i + sceneInfo.pathTracingIteration) % wh;
        int iy = (i + 100 + sceneInfo.pathTracingIteration) % wh;
        int xx = x + randoms[ix] * postProcessingInfo.param2;
        int yy = y + randoms[iy] * postProcessingInfo.param2;
        localColor += postProcessingBuffer[index].colorInfo;
        if (xx >= 0 && xx < sceneInfo.size.x && yy >= 0 &&
            yy < sceneInfo.size.y)
        {
            int localIndex = yy * sceneInfo.size.x + xx;
            float4 lightColor = postProcessingBuffer[localIndex].colorInfo;
            localColor +=
                lightColor * float(primitiveXYIds[localIndex].z) / 256.f;
        }
    }
    localColor /= postProcessingInfo.param3;
    localColor /= div;
    saturateVector(localColor);
    localColor.w = 1.f;

    makeColor(sceneInfo, localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Filters
________________________________________________________________________________
*/
__global__ void k_filter(const int2 occupancyParameters, SceneInfo sceneInfo,
                         PostProcessingInfo postProcessingInfo,
                         PostProcessingBuffer* postProcessingBuffer,
                         BitmapBuffer* bitmap)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    // Filters
    const uint NB_FILTERS = 6;
    const int2 filterSize[NB_FILTERS] = {{3, 3}, {5, 5}, {3, 3},
                                         {3, 3}, {5, 5}, {5, 5}};

    const float2 filterFactors[NB_FILTERS] = {
        {1.f, 128.f}, {1.f, 0.f},  {1.f, 0.f},
        {1.f, 0.f},   {0.2f, 0.f}, {0.125f, 0.f}}; // Factor and Bias

    const float filterInfo[NB_FILTERS][5][5] = {
        {// Emboss
         {-1.0f, -1.0f, 0.0f, 0.0f, 0.0f},
         {-1.0f, 0.0f, 1.0f, 0.0f, 0.0f},
         {0.0f, 1.0f, 1.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {// Find edges
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {-1.0f, -1.0f, 2.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {// Sharpen
         {-1.0f, -1.0f, -1.0f, 0.0f, 0.0f},
         {-1.0f, 9.0f, -1.0f, 0.0f, 0.0f},
         {-1.0f, -1.0f, -1.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {// Blur
         {0.0f, 0.2f, 0.0f, 0.0f, 0.0f},
         {0.2f, 0.2f, 0.2f, 0.0f, 0.0f},
         {0.0f, 0.2f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {// Motion Blur
         {1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
         {0.0f, 0.0f, 0.0f, 0.0f, 1.0f}},
        {// Subtle Sharpen
         {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f},
         {-1.0f, 2.0f, 2.0f, 2.0f, -1.0f},
         {-1.0f, 2.0f, 8.0f, 2.0f, -1.0f},
         {-1.0f, 2.0f, 2.0f, 2.0f, -1.0f},
         {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f}}};

    float4 localColor = {0.f, 0.f, 0.f, 0.f};
    float4 color = {0.f, 0.f, 0.f, 0.f};
    if (postProcessingInfo.param3 < NB_FILTERS)
    {
        // multiply every value of the filter with corresponding image pixel
        for (int filterX = 0; filterX < filterSize[postProcessingInfo.param3].x;
             filterX++)
            for (int filterY = 0;
                 filterY < filterSize[postProcessingInfo.param3].y; filterY++)
            {
                int imageX = (x - filterSize[postProcessingInfo.param3].x / 2 +
                              filterX + sceneInfo.size.x) %
                             sceneInfo.size.x;
                int imageY = (y - filterSize[postProcessingInfo.param3].y / 2 +
                              filterY + sceneInfo.size.y) %
                             sceneInfo.size.y;
                int localIndex = imageY * sceneInfo.size.x + imageX;
                float4 c = postProcessingBuffer[localIndex].colorInfo;
                if (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
                {
                    c /= (float)(sceneInfo.pathTracingIteration -
                                 NB_MAX_ITERATIONS + 1);
                }
                localColor.x +=
                    c.x *
                    filterInfo[postProcessingInfo.param3][filterX][filterY];
                localColor.y +=
                    c.y *
                    filterInfo[postProcessingInfo.param3][filterX][filterY];
                localColor.z +=
                    c.z *
                    filterInfo[postProcessingInfo.param3][filterX][filterY];
            }

        // truncate values smaller than zero and larger than 255
        color.x +=
            min(max(filterFactors[postProcessingInfo.param3].x * localColor.x +
                        filterFactors[postProcessingInfo.param3].y / 255.f,
                    0.f),
                1.f);
        color.y +=
            min(max(filterFactors[postProcessingInfo.param3].x * localColor.y +
                        filterFactors[postProcessingInfo.param3].y / 255.f,
                    0.f),
                1.f);
        color.z +=
            min(max(filterFactors[postProcessingInfo.param3].x * localColor.z +
                        filterFactors[postProcessingInfo.param3].y / 255.f,
                    0.f),
                1.f);
    }

    saturateVector(color);
    color.w = 1.f;

    makeColor(sceneInfo, color, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Filters
________________________________________________________________________________
*/
__global__ void k_cartoon(const int2 occupancyParameters, SceneInfo sceneInfo,
                          PostProcessingInfo postProcessingInfo,
                          PostProcessingBuffer* postProcessingBuffer,
                          BitmapBuffer* bitmap)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error! \[^_^]/
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float depth =
        sceneInfo.viewDistance / fabs(postProcessingBuffer[index].colorInfo.w -
                                      postProcessingInfo.param1);
    float4 color = {depth, depth, depth, 0.f};
    saturateVector(color);
    color.w = 1.f;

    makeColor(sceneInfo, color, bitmap, index);
}

extern "C" void reshape_scene(int2 occupancyParameters, SceneInfo sceneInfo)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        size_t totalMemoryAllocation(0);
        checkCudaErrors(cudaSetDevice(device));

        // Select device
        FREECUDARESOURCE(d_randoms[device]);
        FREECUDARESOURCE(d_postProcessingBuffer[device]);
        FREECUDARESOURCE(d_bitmap[device]);
        FREECUDARESOURCE(d_primitivesXYIds[device]);

        // Randoms
        size_t size =
            MAX_BITMAP_WIDTH * MAX_BITMAP_HEIGHT * sizeof(RandomBuffer);
        LOG_INFO(3, "d_randoms: " << size << " bytes");
        checkCudaErrors(cudaMalloc((void**)&d_randoms[device], size));
        totalMemoryAllocation += size;

        // Post-processing
        size = MAX_BITMAP_WIDTH * MAX_BITMAP_HEIGHT *
               sizeof(PostProcessingBuffer) / occupancyParameters.x;
        LOG_INFO(3, "d_postProcessingBuffer: " << size << " bytes");
        checkCudaErrors(
            cudaMalloc((void**)&d_postProcessingBuffer[device], size));
        totalMemoryAllocation += size;

        // Bitmap
        size = MAX_BITMAP_WIDTH * MAX_BITMAP_HEIGHT * gColorDepth *
               sizeof(BitmapBuffer) / occupancyParameters.x;
        LOG_INFO(3, "d_bitmap: " << size << " bytes");
        checkCudaErrors(cudaMalloc((void**)&d_bitmap[device], size));
        totalMemoryAllocation += size;

        // Primitive IDs
        size = MAX_BITMAP_WIDTH * MAX_BITMAP_HEIGHT *
               sizeof(PrimitiveXYIdBuffer) / occupancyParameters.x;
        LOG_INFO(3, "d_primitivesXYIds: " << size << " bytes");
        checkCudaErrors(cudaMalloc((void**)&d_primitivesXYIds[device], size));
        totalMemoryAllocation += size;

        LOG_INFO(1, " - Total variable GPU memory allocated on device "
                        << device << ": " << totalMemoryAllocation << " bytes");
    }
}

/*
________________________________________________________________________________

GPU initialization
________________________________________________________________________________
*/
extern "C" void initialize_scene(int2 occupancyParameters, SceneInfo sceneInfo,
                                 int nbPrimitives, int nbLamps, int nbMaterials
#ifdef USE_MANAGED_MEMORY
                                 ,
                                 BoundingBox*& boundingBoxes,
                                 Primitive*& primitives
#endif
)
{
    // Multi GPU initialization
    int nbGPUs;
    checkCudaErrors(cudaGetDeviceCount(&nbGPUs));
    if (nbGPUs > MAX_GPU_COUNT)
        nbGPUs = MAX_GPU_COUNT;

    if (occupancyParameters.x > nbGPUs)
    {
        LOG_INFO(1, "You asked for " << occupancyParameters.x
                                     << " CUDA-capable devices, but only "
                                     << nbGPUs << " are available");
        occupancyParameters.x = nbGPUs;
    }
    else
        LOG_INFO(3, "CUDA-capable device count: " << occupancyParameters.x);

    for (int device(0); device < occupancyParameters.x; ++device)
    {
        size_t totalMemoryAllocation(0);
        checkCudaErrors(cudaSetDevice(device));
        for (int stream(0); stream < occupancyParameters.y; ++stream)
            checkCudaErrors(cudaStreamCreate(&d_streams[device][stream]));
        LOG_INFO(3, "Created " << occupancyParameters.y << " streams on device "
                               << device);

        // Bounding boxes
        int size(NB_MAX_BOXES * sizeof(BoundingBox));
        LOG_INFO(3, "d_boundingBoxes: " << size << " bytes");
#ifdef USE_MANAGED_MEMORY
        checkCudaErrors(
            cudaMallocManaged(&boundingBoxes, size, cudaMemAttachHost));
#else
        checkCudaErrors(cudaMalloc((void**)&d_boundingBoxes[device], size));
#endif
        totalMemoryAllocation += size;

        // Primitives
        size = NB_MAX_PRIMITIVES * sizeof(Primitive);
        LOG_INFO(3, "d_primitives: " << size << " bytes");
#ifdef USE_MANAGED_MEMORY
        checkCudaErrors(
            cudaMallocManaged(&primitives, size, cudaMemAttachHost));
#else
        checkCudaErrors(cudaMalloc((void**)&d_primitives[device], size));
#endif
        totalMemoryAllocation += size;

        // Lamps
        size = NB_MAX_LAMPS * sizeof(Lamp);
        checkCudaErrors(cudaMalloc((void**)&d_lamps[device], size));
        LOG_INFO(3, "d_lamps: " << size << " bytes");
        totalMemoryAllocation += size;

        // Materials
        size = NB_MAX_MATERIALS * sizeof(Material);
        checkCudaErrors(cudaMalloc((void**)&d_materials[device], size));
        LOG_INFO(3, "d_materials: " << size << " bytes");
        totalMemoryAllocation += size;

        // Light information
        size = NB_MAX_LIGHTINFORMATIONS * sizeof(LightInformation);
        checkCudaErrors(cudaMalloc((void**)&d_lightInformation[device], size));
        LOG_INFO(3, "d_lightInformation: " << size << " bytes");
        totalMemoryAllocation += size;

        d_textures[device] = 0;
        LOG_INFO(3, "Total constant GPU memory allocated on device "
                        << device << ": " << totalMemoryAllocation << " bytes");
    }

    LOG_INFO(3, "GPU: SceneInfo         : " << sizeof(SceneInfo));
    LOG_INFO(3, "GPU: Ray               : " << sizeof(Ray));
    LOG_INFO(3, "GPU: PrimitiveType     : " << sizeof(PrimitiveType));
    LOG_INFO(3, "GPU: Material          : " << sizeof(Material));
    LOG_INFO(3, "GPU: BoundingBox       : " << sizeof(BoundingBox));
    LOG_INFO(3, "GPU: Primitive         : " << sizeof(Primitive));
    LOG_INFO(3, "GPU: PostProcessingType: " << sizeof(PostProcessingType));
    LOG_INFO(3, "GPU: PostProcessingInfo: " << sizeof(PostProcessingInfo));
    LOG_INFO(3, "Textures " << NB_MAX_TEXTURES);
}

/*
________________________________________________________________________________

GPU finalization
________________________________________________________________________________
*/
extern "C" void finalize_scene(int2 occupancyParameters
#ifdef USE_MANAGED_MEMORY
                               ,
                               BoundingBox* boundingBoxes, Primitive* primitives
#endif
)
{
    LOG_INFO(3, "Releasing device resources");
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));
#ifdef USE_MANAGED_MEMORY
        FREECUDARESOURCE(boundingBoxes);
        FREECUDARESOURCE(primitives);
#else
        FREECUDARESOURCE(d_boundingBoxes[device]);
        FREECUDARESOURCE(d_primitives[device]);
#endif
        FREECUDARESOURCE(d_lamps[device]);
        FREECUDARESOURCE(d_materials[device]);
        FREECUDARESOURCE(d_textures[device]);
        FREECUDARESOURCE(d_lightInformation[device]);
        FREECUDARESOURCE(d_randoms[device]);
        FREECUDARESOURCE(d_postProcessingBuffer[device]);
        FREECUDARESOURCE(d_bitmap[device]);
        FREECUDARESOURCE(d_primitivesXYIds[device]);
        for (int stream(0); stream < occupancyParameters.y; ++stream)
        {
            checkCudaErrors(cudaStreamDestroy(d_streams[device][stream]));
            d_streams[device][stream] = 0;
        }
        checkCudaErrors(cudaDeviceReset());
    }
}

/*
________________________________________________________________________________

CPU -> GPU data transfers
________________________________________________________________________________
*/
extern "C" void h2d_scene(int2 occupancyParameters, BoundingBox* boundingBoxes,
                          int nbActiveBoxes, Primitive* primitives,
                          int nbPrimitives, Lamp* lamps, int nbLamps)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));
#ifndef USE_MANAGED_MEMORY
        checkCudaErrors(cudaMemcpyAsync(d_boundingBoxes[device], boundingBoxes,
                                        nbActiveBoxes * sizeof(BoundingBox),
                                        cudaMemcpyHostToDevice,
                                        d_streams[device][0]));
        checkCudaErrors(cudaMemcpyAsync(d_primitives[device], primitives,
                                        nbPrimitives * sizeof(Primitive),
                                        cudaMemcpyHostToDevice,
                                        d_streams[device][0]));
#endif
        checkCudaErrors(
            cudaMemcpyAsync(d_lamps[device], lamps, nbLamps * sizeof(Lamp),
                            cudaMemcpyHostToDevice, d_streams[device][0]));
    }
}

extern "C" void h2d_materials(int2 occupancyParameters, Material* materials,
                              int nbActiveMaterials)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));
        checkCudaErrors(cudaMemcpyAsync(d_materials[device], materials,
                                        nbActiveMaterials * sizeof(Material),
                                        cudaMemcpyHostToDevice,
                                        d_streams[device][0]));
    }
}

extern "C" void h2d_randoms(int2 occupancyParameters, float* randoms)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));
        checkCudaErrors(cudaMemcpyAsync(d_randoms[device], randoms,
                                        MAX_BITMAP_WIDTH * MAX_BITMAP_HEIGHT *
                                            sizeof(float),
                                        cudaMemcpyHostToDevice,
                                        d_streams[device][0]));
    }
}

extern "C" void h2d_textures(int2 occupancyParameters, int activeTextures,
                             TextureInfo* textureInfos)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));
        int totalSize(0);
        for (int i(0); i < activeTextures; ++i)
            if (textureInfos[i].buffer)
            {
                LOG_INFO(3, "Texture [" << i << "] memory allocated="
                                        << textureInfos[i].size.x *
                                               textureInfos[i].size.y *
                                               textureInfos[i].size.z
                                        << " bytes");
                totalSize += textureInfos[i].size.x * textureInfos[i].size.y *
                             textureInfos[i].size.z;
            }

        FREECUDARESOURCE(d_textures[device]);
        if (totalSize > 0)
        {
            totalSize *= sizeof(BitmapBuffer);
            LOG_INFO(3, "Total GPU texture memory to allocate: " << totalSize
                                                                 << " bytes");
            checkCudaErrors(cudaMalloc((void**)&d_textures[device], totalSize));

            for (int i(0); i < activeTextures; ++i)
                if (textureInfos[i].buffer != 0)
                {
                    LOG_INFO(3, "Texture ["
                                    << i
                                    << "] transfered=" << textureInfos[i].size.x
                                    << "," << textureInfos[i].size.y << ","
                                    << textureInfos[i].size.z
                                    << ", offset=" << textureInfos[i].offset);
                    int textureSize = textureInfos[i].size.x *
                                      textureInfos[i].size.y *
                                      textureInfos[i].size.z;
                    checkCudaErrors(cudaMemcpyAsync(
                        d_textures[device] + textureInfos[i].offset,
                        textureInfos[i].buffer,
                        textureSize * sizeof(BitmapBuffer),
                        cudaMemcpyHostToDevice, d_streams[device][0]));
                }
        }
    }
}

extern "C" void h2d_lightInformation(int2 occupancyParameters,
                                     LightInformation* lightInformation,
                                     int lightInformationSize)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));
        checkCudaErrors(
            cudaMemcpyAsync(d_lightInformation[device], lightInformation,
                            lightInformationSize * sizeof(LightInformation),
                            cudaMemcpyHostToDevice, d_streams[device][0]));
    }
}

#ifdef USE_KINECT
extern "C" void h2d_kinect(int2 occupancyParameters, BitmapBuffer* kinectVideo,
                           BitmapBuffer* kinectDepth)
{
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(
            cudaMemcpyAsync(d_textures[device], kinectVideo,
                            KINECT_COLOR_SIZE * sizeof(BitmapBuffer),
                            cudaMemcpyHostToDevice, d_streams[device][0]));
        checkCudaErrors(
            cudaMemcpyAsync(d_textures[device] + KINECT_COLOR_SIZE, kinectDepth,
                            KINECT_DEPTH_SIZE * sizeof(BitmapBuffer),
                            cudaMemcpyHostToDevice, d_streams[device][0]));
    }
}
#endif // USE_KINECT

/*
________________________________________________________________________________

GPU -> CPU data transfers
________________________________________________________________________________
*/
extern "C" void d2h_bitmap(int2 occupancyParameters, SceneInfo sceneInfo,
                           BitmapBuffer* bitmap,
                           PrimitiveXYIdBuffer* primitivesXYIds)
{
    int offsetBitmap = sceneInfo.size.x * sceneInfo.size.y * gColorDepth *
                       sizeof(BitmapBuffer) / occupancyParameters.x;
    int offsetXYIds = sceneInfo.size.x * sceneInfo.size.y *
                      sizeof(PrimitiveXYIdBuffer) / occupancyParameters.x;
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));

        // Synchronize stream
        for (int stream(0); stream < occupancyParameters.y; ++stream)
        {
            LOG_INFO(3, "Synchronizing stream "
                            << stream << "/" << occupancyParameters.y
                            << " on device " << device << "/"
                            << occupancyParameters.x);
            checkCudaErrors(cudaStreamSynchronize(d_streams[device][stream]));
        }

        // Copy results back to host
        LOG_INFO(3, "Copy results back to host: "
                        << device * offsetBitmap << "/" << offsetBitmap << ", "
                        << device * offsetXYIds << "/" << offsetXYIds);
        checkCudaErrors(cudaMemcpyAsync(bitmap + device * offsetBitmap,
                                        d_bitmap[device], offsetBitmap,
                                        cudaMemcpyDeviceToHost));
        checkCudaErrors(cudaMemcpyAsync(primitivesXYIds + device * offsetXYIds,
                                        d_primitivesXYIds[device], offsetXYIds,
                                        cudaMemcpyDeviceToHost));
    }
}

/*
________________________________________________________________________________

Kernel launcher
________________________________________________________________________________
*/
extern "C" void cudaRender(int2 occupancyParameters, int4 blockSize,
                           SceneInfo sceneInfo, int4 objects,
                           PostProcessingInfo postProcessingInfo, vec3f origin,
                           vec3f direction, vec4f angles
#ifdef USE_MANAGED_MEMORY
                           ,
                           BoundingBox* boundingBoxes, Primitive* primitives
#endif
)
{
    LOG_INFO(3, "CPU PostProcessingBuffer: " << sizeof(PostProcessingBuffer));
    LOG_INFO(3, "CPU PrimitiveXYIdBuffer : " << sizeof(PrimitiveXYIdBuffer));
    LOG_INFO(3, "CPU BoundingBox         : " << sizeof(BoundingBox));
    LOG_INFO(3, "CPU Primitive           : " << sizeof(Primitive));
    LOG_INFO(3, "CPU Material            : " << sizeof(Material));

    int2 size;
    size.x = static_cast<int>(sceneInfo.size.x);
    size.y = static_cast<int>(sceneInfo.size.y) /
             (occupancyParameters.x * occupancyParameters.y);

    dim3 grid;
    grid.x = (size.x + blockSize.x - 1) / blockSize.x;
    grid.y = (size.y + blockSize.y - 1) / blockSize.y;
    grid.z = 1;

    dim3 blocks;
    blocks.x = blockSize.x;
    blocks.y = blockSize.y;
    blocks.z = 1;

    LOG_INFO(3, "Running rendering kernel...");
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));

        for (int stream(0); stream < occupancyParameters.y; ++stream)
        {
            switch (sceneInfo.cameraType)
            {
            case ctAnaglyph:
            {
                k_anaglyphRenderer<<<grid, blocks, 0,
                                     d_streams[device][stream]>>>(
                    occupancyParameters,
#ifndef USE_MANAGED_MEMORY
                    d_boundingBoxes[device],
#else
                    boundingBoxes,
#endif
                    objects.x,
#ifndef USE_MANAGED_MEMORY
                    d_primitives[device],
#else
                    primitives,
#endif
                    objects.y, d_lightInformation[device], objects.w, objects.z,
                    d_materials[device], d_textures[device], d_randoms[device],
                    origin, direction, angles, sceneInfo, postProcessingInfo,
                    d_postProcessingBuffer[device], d_primitivesXYIds[device]);
                break;
            }
            case ctVR:
            {
                k_3DVisionRenderer<<<grid, blocks, 0,
                                     d_streams[device][stream]>>>(
                    occupancyParameters,
#ifndef USE_MANAGED_MEMORY
                    d_boundingBoxes[device],
#else
                    boundingBoxes,
#endif
                    objects.x,
#ifndef USE_MANAGED_MEMORY
                    d_primitives[device],
#else
                    primitives,
#endif
                    objects.y, d_lightInformation[device], objects.w, objects.z,
                    d_materials[device], d_textures[device], d_randoms[device],
                    origin, direction, angles, sceneInfo, postProcessingInfo,
                    d_postProcessingBuffer[device], d_primitivesXYIds[device]);
                break;
            }
            case ctPanoramic:
            {
                k_fishEyeRenderer<<<grid, blocks, 0,
                                    d_streams[device][stream]>>>(
                    occupancyParameters, device * stream * size.y,
#ifndef USE_MANAGED_MEMORY
                    d_boundingBoxes[device],
#else
                    boundingBoxes,
#endif
                    objects.x,
#ifndef USE_MANAGED_MEMORY
                    d_primitives[device],
#else
                    primitives,
#endif
                    objects.y, d_lightInformation[device], objects.w, objects.z,
                    d_materials[device], d_textures[device], d_randoms[device],
                    origin, direction, angles, sceneInfo, postProcessingInfo,
                    d_postProcessingBuffer[device], d_primitivesXYIds[device]);
                break;
            }
            case ctVolumeRendering:
            {
                k_volumeRenderer<<<grid, blocks, 0,
                                   d_streams[device][stream]>>>(
                    occupancyParameters,
                    device * (size.y / occupancyParameters.x), stream * size.y,
#ifndef USE_MANAGED_MEMORY
                    d_boundingBoxes[device],
#else
                    boundingBoxes,
#endif
                    objects.x,
#ifndef USE_MANAGED_MEMORY
                    d_primitives[device],
#else
                    primitives,
#endif
                    objects.y, d_lightInformation[device], objects.w, objects.z,
                    d_materials[device], d_textures[device], d_randoms[device],
                    origin, direction, angles, sceneInfo, postProcessingInfo,
                    d_postProcessingBuffer[device], d_primitivesXYIds[device]);
                break;
            }
            default:
            {
                k_standardRenderer<<<grid, blocks, 0,
                                     d_streams[device][stream]>>>(
                    occupancyParameters,
                    device * (size.y / occupancyParameters.x), stream * size.y,
#ifndef USE_MANAGED_MEMORY
                    d_boundingBoxes[device],
#else
                    boundingBoxes,
#endif
                    objects.x,
#ifndef USE_MANAGED_MEMORY
                    d_primitives[device],
#else
                    primitives,
#endif
                    objects.y, d_lightInformation[device], objects.w, objects.z,
                    d_materials[device], d_textures[device], d_randoms[device],
                    origin, direction, angles, sceneInfo, postProcessingInfo,
                    d_postProcessingBuffer[device], d_primitivesXYIds[device]);
                break;
            }
            }
            cudaError_t status = cudaGetLastError();
            if (status != cudaSuccess)
            {
                LOG_ERROR(
                    "**********************************************************"
                    "**********************");
                LOG_ERROR("Error code : [" << status << "] "
                                           << cudaGetErrorString(status));
                LOG_ERROR("Device     : " << device);
                LOG_ERROR("Stream     : " << stream);
                LOG_ERROR("Image size : " << size.x << ", " << size.y);
                LOG_ERROR("Grid size  : " << grid.x << ", " << grid.y << ", "
                                          << grid.z);
                LOG_ERROR("Block size : " << blocks.x << ", " << blocks.y
                                          << ", " << blocks.z);
                LOG_ERROR("Boxes      : " << objects.x);
                LOG_ERROR("Primitives : " << objects.y);
                LOG_ERROR("Lamps      : " << objects.z);
                LOG_ERROR(
                    "**********************************************************"
                    "**********************");
            }
        }
        // checkCudaErrors(cudaThreadSynchronize());
    }
    LOG_INFO(3, "Rendering kernel done!");

    // --------------------------------------------------------------------------------
    // Post processing on device 0, stream 0
    // --------------------------------------------------------------------------------
    size.x = static_cast<int>(sceneInfo.size.x);
    size.y = static_cast<int>(sceneInfo.size.y) / occupancyParameters.x;

    grid.x = (size.x + blockSize.x - 1) / blockSize.x;
    grid.y = (size.y + blockSize.y - 1) / blockSize.y;
    grid.z = 1;

    blocks.x = blockSize.x;
    blocks.y = blockSize.y;
    blocks.z = blockSize.z;

    LOG_INFO(3, "Running post-processing kernel...");
    for (int device(0); device < occupancyParameters.x; ++device)
    {
        checkCudaErrors(cudaSetDevice(device));

        switch (postProcessingInfo.type)
        {
        case ppe_depthOfField:
            k_depthOfField<<<grid, blocks, 0, d_streams[device][0]>>>(
                occupancyParameters, sceneInfo, postProcessingInfo,
                d_postProcessingBuffer[device], d_randoms[device],
                d_bitmap[device]);
            break;
        case ppe_ambientOcclusion:
            k_ambiantOcclusion<<<grid, blocks, 0, d_streams[device][0]>>>(
                occupancyParameters, sceneInfo, postProcessingInfo,
                d_postProcessingBuffer[device], d_randoms[device],
                d_bitmap[device]);
            break;
        case ppe_radiosity:
            k_radiosity<<<grid, blocks, 0, d_streams[device][0]>>>(
                occupancyParameters, sceneInfo, postProcessingInfo,
                d_primitivesXYIds[device], d_postProcessingBuffer[device],
                d_randoms[device], d_bitmap[device]);
            break;
        case ppe_filter:
            k_filter<<<grid, blocks, 0, d_streams[device][0]>>>(
                occupancyParameters, sceneInfo, postProcessingInfo,
                d_postProcessingBuffer[device], d_bitmap[device]);
            break;
        case ppe_cartoon:
            k_cartoon<<<grid, blocks, 0, d_streams[device][0]>>>(
                occupancyParameters, sceneInfo, postProcessingInfo,
                d_postProcessingBuffer[device], d_bitmap[device]);
            break;
        default:
            k_default<<<grid, blocks, 0, d_streams[device][0]>>>(
                occupancyParameters, sceneInfo, postProcessingInfo,
                d_postProcessingBuffer[device], d_bitmap[device]);
            break;
        }

        cudaError_t status = cudaGetLastError();
        if (status != cudaSuccess)
        {
            LOG_ERROR(
                "**************************************************************"
                "******************");
            LOG_ERROR("Error code : [" << status << "] "
                                       << cudaGetErrorString(status));
            LOG_ERROR("Device     : " << device);
            LOG_ERROR("Stream     : " << 0);
            LOG_ERROR("Image size : " << size.x << ", " << size.y);
            LOG_ERROR("Grid size  : " << grid.x << ", " << grid.y << ", "
                                      << grid.z);
            LOG_ERROR("Block size : " << blocks.x << ", " << blocks.y << ", "
                                      << blocks.z);
            LOG_ERROR("Boxes      : " << objects.x);
            LOG_ERROR("Primitives : " << objects.y);
            LOG_ERROR("Lamps      : " << objects.z);
            LOG_ERROR(
                "**************************************************************"
                "******************");
        }
    }
    LOG_INFO(3, "Post-processing kernel done!");
}
