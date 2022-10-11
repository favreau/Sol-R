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

#ifndef SOLR_H
#define SOLR_H

#include "consts.h"
#include "solr_defines.h"
#include <map>
#include <vector>

namespace solr
{
#ifdef USE_OPENCL
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#ifdef __APPLE__
#include </Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.13.sdk/System/Library/Frameworks/OpenCL.framework/Headers/opencl.h>
#else // __APPLE__
#include <CL/cl.h>
#endif // __APPLE__

typedef cl_float vec1f;
typedef cl_float2 vec2f;
typedef cl_float4 vec3f;
typedef cl_float4 vec4f;
typedef cl_int vec1i;
typedef cl_int2 vec2i;
typedef cl_int3 vec3i;
typedef cl_int4 vec4i;
typedef cl_int4 PrimitiveXYIdBuffer;

inline vec2i make_vec2i(const int x = 0, const int y = 0)
{
    return {{x, y}};
}
inline vec3i make_vec3i(const int x = 0, const int y = 0, const int z = 0)
{
    return {{x, y, z}};
}
inline vec4i make_vec4i(const int x = 0, const int y = 0, const int z = 0,
                        const int w = 0)
{
    return {{x, y, z, w}};
}
inline vec2f make_vec2f(const float x = 0.f, const float y = 0.f)
{
    return {{x, y}};
}
inline vec3f make_vec3f(const float x = 0.f, const float y = 0.f,
                        const float z = 0.f)
{
    return {{x, y, z}};
}
inline vec4f make_vec4f(const float x = 0.f, const float y = 0.f,
                        const float z = 0.f, const float w = 0.f)
{
    return {{x, y, z, w}};
}

#define __ALIGN16__
#else
#include <vector_types.h>

typedef float vec1f;
typedef float2 vec2f;
typedef float3 vec3f;
typedef float4 vec4f;
typedef int vec1i;
typedef int2 vec2i;
typedef int3 vec3i;
typedef int4 vec4i;
typedef int4 PrimitiveXYIdBuffer;

inline vec2i make_vec2i(const int x = 0, const int y = 0)
{
    return {x, y};
}
inline vec3i make_vec3i(const int x = 0, const int y = 0, const int z = 0)
{
    return {x, y, z};
}
inline vec4i make_vec4i(const int x = 0, const int y = 0, const int z = 0,
                        const int w = 0)
{
    return {x, y, z, w};
}
inline vec2f make_vec2f(const float x = 0.f, const float y = 0.f)
{
    return {x, y};
}
inline vec3f make_vec3f(const float x = 0.f, const float y = 0.f,
                        const float z = 0.f)
{
    return {x, y, z};
}
inline vec4f make_vec4f(const float x = 0.f, const float y = 0.f,
                        const float z = 0.f, const float w = 0.f)
{
    return {x, y, z, w};
}

#define __ALIGN16__ __align__(16)
#endif

// Vectors
typedef std::vector<vec2f> vec2fs;
typedef std::vector<vec3f> vec3fs;
typedef std::vector<vec4f> vec4fs;
typedef std::vector<vec2i> vec2is;
typedef std::vector<vec3i> vec3is;
typedef std::vector<vec4i> vec4is;

#define _CRT_SECURE_NO_WARNINGS
#define __INLINE__ inline

typedef unsigned char BitmapBuffer;
typedef float RandomBuffer;
typedef int Lamp;

struct PostProcessingBuffer
{
    vec4f colorInfo;
    vec4f sceneInfo;
};

enum CameraType
{
    ctPerspective = 0,
    ctOrthographic = 1,
    ctAnaglyph = 2,
    ctVR = 3,
    ctPanoramic = 4,
    ctAntialiazed = 5,
    ctVolumeRendering = 6
};

enum FrameBufferType
{
    ftRGB = 0, // RGB 24bit
    ftBGR = 1  // BGR 24bit
};

enum AdvancedIllumination
{
    aiNone = 0,
    aiBasic = 1,
    aiFull = 2,
    aiRandomIllumination = 3
};

enum GraphicsLevel
{
    glNoShading = 0,
    glPhong = 1,
    glPhongAndBlinn = 2,
    glReflectionsAndRefractions = 3,
    glFull = 4
};

enum AtmosphericEffect
{
    aeNone = 0,
    aeFog = 1
};

// Scene information
struct __ALIGN16__ SceneInfo
{
    vec2i size;            // Image size
    CameraType cameraType; // Camera type( Perspective, Orthographic, Anaglyph,
                           // VR, Panoramic, Antialiazed, Volume rendering )
    GraphicsLevel graphicsLevel; // Graphics level( No Shading=0, Lambert=1,
                                 // Specular=2, textured=3, Reflections and
                                 // Refractions=4, Shadows=5)
    vec1i nbRayIterations; // Maximum number of ray iterations for current frame
    vec1f transparentColor; // Value above which r+g+b color is considered as
                            // transparent
    vec1f viewDistance;     // Maximum viewing distance
    vec1f shadowIntensity;  // Shadow intensity( off=0, pitch black=1)
    vec1f eyeSeparation;    // Distance between both eyes (3D stereo)
    vec1i renderBoxes;      // Activate bounding box rendering
    vec1i pathTracingIteration;      // Current iteration for current frame
    vec1i maxPathTracingIterations;  // Maximum number of iterations for current
                                     // frame
    FrameBufferType frameBufferType; // Frame buffer type( RGB or BGR )
    vec1i timestamp;                 // Timestamp
    AtmosphericEffect atmosphericEffect;       // Atmospheric effects
    vec1i doubleSidedTriangles;                // Use double-sided triangles
    vec1i extendedGeometry;                    // Use extended geometry
    AdvancedIllumination advancedIllumination; // Advanced features (Global
                                               // illumination, random
                                               // lightning, etc)
    vec1i draftMode;          // Draft mode when camera in motion
    vec1i skyboxRadius;       // Skybox sphere radius
    vec1i skyboxMaterialId;   // Skybox material Id
    vec1i gradientBackground; // Gradient background
    vec1f geometryEpsilon;    // Geometry epsilon
    vec1f rayEpsilon;         // Ray epsilon
    vec4f backgroundColor;    // Background color
};

// Ray structure
struct __ALIGN16__ Ray
{
    vec3f origin;        // Origin of the ray
    vec3f direction;     // Direction of the ray
    vec3f inv_direction; // Inverted direction( Used for optimal Ray-Box
                         // intersection )
    vec4i signs;         // Signs ( Used for optimal Ray-Box intersection )
};

// Light Information Structure used for global illumination
// When iterating on frames, lights sources are randomly picked from an array of
// that very structure, in order to simulate global illumination
struct __ALIGN16__ LightInformation
{
    vec1i primitiveId; // ID of the emitting primitive
    vec1i materialId;  // Material ID of the emitting primitive
    vec3f location;    // Position in space
    vec4f color;       // Light
};

// Primitive types
enum PrimitiveType
{
    ptSphere = 0,
    ptCylinder = 1,
    ptTriangle = 2,
    ptCheckboard = 3,
    ptCamera = 4,
    ptXYPlane = 5,
    ptYZPlane = 6,
    ptXZPlane = 7,
    ptMagicCarpet = 8,
    ptEnvironment = 9,
    ptEllipsoid = 10,
    ptQuad = 11,
    ptCone = 12
};

// Material structure
struct __ALIGN16__ Material
{
    vec4f innerIllumination; // x: Inner illumination
                             // y: Diffusion strength
                             // z: <not used>
                             // w: Illuminance
    vec4f color;             // Color( R,G,B )
    vec4f specular;          // x: Value
                             // y: Power
                             // z: <not used>
                             // w: <not used>
    vec1f reflection; // Reflection rate( No reflection=0 -> Full reflection=1 )
    vec1f refraction; // Refraction index( ex: glass=1.33 )
    vec1f transparency; // Transparency rate( Opaque=0 -> Full transparency=1 )
    vec1f opacity;      // Opacity strength
    vec4i attributes; // x: Fast transparency( off=0, on=1 ). Fast transparency
    // produces no shadows and drops intersections if rays intersects primitive
    // with the same material ID y: Procedural textures( off=0, on=1 ) z:
    // Wireframe( off=0, on=1 ). Wire frame produces no shading w: Wireframe
    // Width
    vec4i textureMapping;        // x: U padding
                                 // y: V padding
                                 // z: Texture ID (Deprecated)
                                 // w: Texture color depth
    vec4i textureOffset;         // x: Offset in the diffuse map
                                 // y: Offset in the normal map
                                 // z: Offset in the bump map
                                 // w: Offset in the specular map
    vec4i textureIds;            // x: Diffuse map
                                 // y: Normal map
                                 // z: Bump map
                                 // w: Specular map
    vec4i advancedTextureOffset; // x: Reflection map
                                 // y: Transparency map
                                 // z: Ambiant Occulusion
                                 // w: not used
    vec4i advancedTextureIds;    // x: Reflection map
                                 // y: Transparency map
                                 // z: Ambiant Occulusion
                                 // w: not used
    vec2f mappingOffset; // Texture mapping offsets based on sceneInfo.timestamp
};

// Bounding Box Structure
struct __ALIGN16__ BoundingBox
{
    vec3f parameters[2];   // Bottom-Left and Top-Right corners
    vec1i nbPrimitives;    // Number of primitives in the box
    vec1i startIndex;      // Index of the first primitive in the box
    vec2i indexForNextBox; // If no intersection, how many of the following
                           // boxes can be skipped?
};
typedef std::map<size_t, BoundingBox> BoundingBoxes;

// Primitive Structure
struct __ALIGN16__ Primitive
{
    // Vertices
    vec3f p0;
    vec3f p1;
    vec3f p2;
    // Normals
    vec3f n0;
    vec3f n1;
    vec3f n2;
    // Size( x,y,z )
    vec3f size;
    // Type( See PrimitiveType )
    vec1i type;
    // Index
    vec1i index;
    // Material ID
    vec1i materialId;
    // Texture coordinates
    vec2f vt0;
    vec2f vt1;
    vec2f vt2;
};
typedef std::map<size_t, Primitive> Primitives;

enum TextureType
{
    tex_diffuse = 0,
    tex_bump,
    tex_normal,
    tex_ambient_occlusion,
    tex_reflective,
    tex_specular,
    tex_transparent
};

// Texture information structure
struct __ALIGN16__ TextureInfo
{
    unsigned char *buffer; // Pointer to the texture
    vec1i offset; // Offset of the texture in the global texture buffer (the one
                  // that will be transfered to the GPU)
    vec3i size;   // Size of the texture
    TextureType type; // Texture type (diffuse, normal, bump, etc.)
};

// Post processing types
// Effects are based on the PostProcessingBuffer
enum PostProcessingType
{
    ppe_none,             // No effect
    ppe_depthOfField,     // Depth of field
    ppe_ambientOcclusion, // Ambient occlusion
    ppe_radiosity,        // Radiosity
    ppe_filter,           // Various Filters
    ppe_cartoon           // Cartoon
};

// Post processing information
struct __ALIGN16__ PostProcessingInfo
{
    vec1i type;   // Type( See PostProcessingType enum )
    vec1f param1; // Parameter role depends on post processing type
    vec1f param2; // Parameter role depends on post processing type
    vec1i param3; // Parameter role depends on post processing type
};

// Host data structures
struct CPUPrimitive
{
    bool belongsToModel;
    bool movable;
    vec3f p0;
    vec3f p1;
    vec3f p2;
    vec3f n0;
    vec3f n1;
    vec3f n2;
    vec3f size;
    int type;
    int materialId;
    vec2f vt0; // Texture coordinates
    vec2f vt1;
    vec2f vt2;
    vec3f speed0;
    vec3f speed1;
    vec3f speed2;
};

struct CPUBoundingBox
{
    vec3f parameters[2];
    vec3f center;
    std::vector<long> primitives;
    long indexForNextBox;
};

typedef std::map<unsigned int, CPUBoundingBox> BoxContainer;
typedef std::map<unsigned int, CPUPrimitive> PrimitiveContainer;
typedef std::map<unsigned int, Lamp> LampContainer;

// Forward declarations
class GPUKernel;
#ifdef USE_CUDA
class CudaKernel;
#else
#ifdef USE_OPENCL
class OpenCLKernel;
#else
#ifdef USE_CPU
class CPUKernel;
#endif // USE_CPU
#endif // USE_OPENCL
#endif // USE_CUDA

} // namespace solr

#endif // SOLR_H
