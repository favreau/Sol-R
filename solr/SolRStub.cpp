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

#pragma warning(disable : 4996)

#include "SolRStub.h"
#include "Logging.h"
#include "engines/GPUKernel.h"
#include "io/FileMarshaller.h"
#include "io/OBJReader.h"
#include "io/PDBReader.h"
#include "solr.h"

#include <fstream>

#ifdef USE_OPENCL
#include <engines/opencl/OpenCLKernel.h>
#endif

using namespace solr;

SceneInfo gSceneInfoStub;
PostProcessingInfo gPostProcessingInfoStub;

// --------------------------------------------------------------------------------
// Implementation
// --------------------------------------------------------------------------------
int SolR_SetSceneInfo1()
{
    return 0;
}

extern "C" SOLR_API int SolR_SetSceneInfo(
    int width, int height, int graphicsLevel, int nbRayIterations,
    double transparentColor, double viewDistance, double shadowIntensity,
    double eyeSeparation, double bgColorR, double bgColorG, double bgColorB,
    double bgColorA, int renderBoxes, int pathTracingIteration,
    int maxPathTracingIterations, int frameBufferType, int timestamp,
    int atmosphericEffect, int cameraType, int doubleSidedTriangles,
    int extendedGeometry, int advancedIllumination, int skyboxSize,
    int skyboxMaterialId, double geometryEpsilon, double rayEpsilon)
{
    LOG_INFO(3, "SolR_SetSceneInfo ("
                    << width << "," << height << "," << graphicsLevel << ","
                    << nbRayIterations << "," << transparentColor << ","
                    << viewDistance << "," << shadowIntensity << ","
                    << eyeSeparation << "," << bgColorR << "," << bgColorG
                    << "," << bgColorB << "," << bgColorA << "," << cameraType
                    << "," << renderBoxes << "," << pathTracingIteration << ","
                    << maxPathTracingIterations << "," << frameBufferType << ","
                    << timestamp << "," << atmosphericEffect << ","
                    << cameraType << "," << geometryEpsilon << ","
                    << rayEpsilon);

    gSceneInfoStub.size.x = width;
    gSceneInfoStub.size.y = height;
    gSceneInfoStub.graphicsLevel = static_cast<GraphicsLevel>(graphicsLevel);
    gSceneInfoStub.nbRayIterations = nbRayIterations;
    gSceneInfoStub.transparentColor = static_cast<float>(transparentColor);
    gSceneInfoStub.viewDistance = static_cast<float>(viewDistance);
    gSceneInfoStub.shadowIntensity = static_cast<float>(shadowIntensity);
    gSceneInfoStub.eyeSeparation = static_cast<float>(eyeSeparation);
    gSceneInfoStub.backgroundColor.x = static_cast<float>(bgColorR);
    gSceneInfoStub.backgroundColor.y = static_cast<float>(bgColorG);
    gSceneInfoStub.backgroundColor.z = static_cast<float>(bgColorB);
    gSceneInfoStub.backgroundColor.w = static_cast<float>(bgColorA);
    gSceneInfoStub.renderBoxes = static_cast<int>(renderBoxes);
    gSceneInfoStub.pathTracingIteration = pathTracingIteration;
    gSceneInfoStub.maxPathTracingIterations = maxPathTracingIterations;
    gSceneInfoStub.frameBufferType =
        static_cast<FrameBufferType>(frameBufferType);
    gSceneInfoStub.timestamp = timestamp;
    gSceneInfoStub.atmosphericEffect =
        static_cast<AtmosphericEffect>(atmosphericEffect);
    gSceneInfoStub.cameraType = static_cast<CameraType>(cameraType);
    gSceneInfoStub.doubleSidedTriangles = doubleSidedTriangles;
    gSceneInfoStub.extendedGeometry = extendedGeometry;
    gSceneInfoStub.advancedIllumination =
        static_cast<AdvancedIllumination>(advancedIllumination);

    gSceneInfoStub.skyboxRadius = skyboxSize;
    gSceneInfoStub.skyboxMaterialId = skyboxMaterialId;
    gSceneInfoStub.geometryEpsilon = geometryEpsilon;
    gSceneInfoStub.rayEpsilon = rayEpsilon;
    return 0;
}

int SolR_SetPostProcessingInfo(int type, double param1, double param2,
                               int param3)
{
    LOG_INFO(3, "SolR_SetPostProcessingInfo");
    gPostProcessingInfoStub.type = type;
    gPostProcessingInfoStub.param1 = static_cast<float>(param1);
    gPostProcessingInfoStub.param2 = static_cast<float>(param2);
    gPostProcessingInfoStub.param3 = param3;
    return 0;
}

int SolR_SetDraftMode(int draft)
{
    gSceneInfoStub.draftMode = draft;
    return 0;
}

int SolR_InitializeKernel(bool activeLogging, int platform, int device)
{
    LOG_INFO(3, "SolR_InitializeKernel");
    solr::GPUKernel *Kernel = solr::SingletonKernel::kernel();
    if (Kernel)
    {
        gSceneInfoStub.pathTracingIteration = 0;
        Kernel->setSceneInfo(gSceneInfoStub);
        Kernel->initBuffers();
        Kernel->setFrame(0);
        return 0;
    }
    return -1;
}

// --------------------------------------------------------------------------------
int SolR_FinalizeKernel()
{
    LOG_INFO(3, "SolR_FinalizeKernel");
    return 0;
}

// --------------------------------------------------------------------------------
int SolR_ResetKernel()
{
    LOG_INFO(3, "SolR_ResetKernel");
    solr::SingletonKernel::kernel()->resetAll();
    return 0;
}

// --------------------------------------------------------------------------------
void SolR_SetCamera(double eye_x, double eye_y, double eye_z, double dir_x,
                    double dir_y, double dir_z, double angle_x, double angle_y,
                    double angle_z)
{
    LOG_INFO(3, "SolR_SetCamera");
    const vec3f eye =
        make_vec3f(static_cast<float>(eye_x), static_cast<float>(eye_y),
                   static_cast<float>(eye_z));
    const vec3f dir =
        make_vec3f(static_cast<float>(dir_x), static_cast<float>(dir_y),
                   static_cast<float>(dir_z));
    const vec4f angles =
        make_vec4f(static_cast<float>(angle_x), static_cast<float>(angle_y),
                   static_cast<float>(angle_z), 6400.f);
    solr::SingletonKernel::kernel()->setCamera(eye, dir, angles);
}

// --------------------------------------------------------------------------------
int SolR_RunKernel(double timer, BitmapBuffer *image)
{
    LOG_INFO(3, "SolR_RunKernel");
    solr::SingletonKernel::kernel()->setSceneInfo(gSceneInfoStub);
    solr::SingletonKernel::kernel()->setPostProcessingInfo(
        gPostProcessingInfoStub);
    solr::SingletonKernel::kernel()->render_begin(static_cast<float>(timer));
    solr::SingletonKernel::kernel()->render_end();
    memcpy(image, solr::SingletonKernel::kernel()->getBitmap(),
           gSceneInfoStub.size.x * gSceneInfoStub.size.y * gColorDepth);
    return 0;
}

// --------------------------------------------------------------------------------
int SolR_AddPrimitive(int type, int movable)
{
    LOG_INFO(3, "SolR_AddPrimitive");
    int id = solr::SingletonKernel::kernel()->addPrimitive(
        static_cast<PrimitiveType>(type));
    solr::SingletonKernel::kernel()->setPrimitiveIsMovable(id, (movable == 1));
    return id;
}

// --------------------------------------------------------------------------------
int SolR_SetPrimitive(int index, double p0_x, double p0_y, double p0_z,
                      double p1_x, double p1_y, double p1_z, double p2_x,
                      double p2_y, double p2_z, double size_x, double size_y,
                      double size_z, int materialId)
{
    LOG_INFO(3, "SolR_SetPrimitive");
    solr::SingletonKernel::kernel()->setPrimitive(
        index, static_cast<float>(p0_x), static_cast<float>(p0_y),
        static_cast<float>(p0_z), static_cast<float>(p1_x),
        static_cast<float>(p1_y), static_cast<float>(p1_z),
        static_cast<float>(p2_x), static_cast<float>(p2_y),
        static_cast<float>(p2_z), static_cast<float>(size_x),
        static_cast<float>(size_y), static_cast<float>(size_z), materialId);
    return 0;
}

int SolR_GetPrimitive(int index, double &p0_x, double &p0_y, double &p0_z,
                      double &p1_x, double &p1_y, double &p1_z, double &p2_x,
                      double &p2_y, double &p2_z, double &size_x,
                      double &size_y, double &size_z, int &materialId)
{
    LOG_INFO(3, "SolR_GetPrimitive");
    solr::CPUPrimitive *primitive =
        solr::SingletonKernel::kernel()->getPrimitive(index);
    if (primitive != NULL)
    {
        p0_x = primitive->p0.x;
        p0_y = primitive->p0.y;
        p0_z = primitive->p0.z;
        p1_x = primitive->p1.x;
        p1_y = primitive->p1.y;
        p1_z = primitive->p1.z;
        p2_x = primitive->p2.x;
        p2_y = primitive->p2.y;
        p2_z = primitive->p2.z;
        size_x = primitive->size.x;
        size_y = primitive->size.y;
        size_z = primitive->size.z;
        materialId = primitive->materialId;
        return 0;
    }
    return -1;
}

int SolR_GetPrimitiveAt(int x, int y)
{
    LOG_INFO(3, "SolR_GetPrimitiveAt");
    return solr::SingletonKernel::kernel()->getPrimitiveAt(x, y);
}

int SolR_GetPrimitiveCenter(int index, double &x, double &y, double &z)
{
    LOG_INFO(3, "SolR_GetPrimitiveCenter");
    vec4f center = solr::SingletonKernel::kernel()->getPrimitiveCenter(index);
    x = static_cast<double>(center.x);
    y = static_cast<double>(center.y);
    z = static_cast<double>(center.z);
    return 0;
}

int SolR_RotatePrimitive(int index, double rx, double ry, double rz, double ax,
                         double ay, double az)
{
    LOG_INFO(3, "SolR_RotatePrimitive");
    //    vec4f rotationCenter = {{static_cast<float>(rx),
    //    static_cast<float>(ry), static_cast<float>(rz), 0.f}}; vec4f angles =
    //    {{static_cast<float>(ax), static_cast<float>(ay),
    //    static_cast<float>(az), 0.f}};
    //
    //    solr::SingletonKernel::kernel()->rotatePrimitive( index, boxId,
    //    rotationCenter, angles ); // TODO!!
    return 0;
}

int SolR_RotatePrimitives(int fromBoxId, int toBoxId, double rx, double ry,
                          double rz, double ax, double ay, double az)
{
    LOG_INFO(3, "SolR_RotatePrimitives");
    try
    {
        vec3f rotationCenter =
            make_vec3f(static_cast<float>(rx), static_cast<float>(ry),
                       static_cast<float>(rz));
        vec4f angles =
            make_vec4f(static_cast<float>(ax), static_cast<float>(ay),
                       static_cast<float>(az));

        solr::SingletonKernel::kernel()->rotatePrimitives(rotationCenter,
                                                          angles);
        solr::SingletonKernel::kernel()->compactBoxes(false);
    }
    catch (...)
    {
    }
    return 0;
}

int SolR_SetPrimitiveMaterial(int index, int materialId)
{
    LOG_INFO(3, "SolR_SetPrimitiveMaterial");
    solr::SingletonKernel::kernel()->setPrimitiveMaterial(index, materialId);
    return 0;
}

int SolR_GetPrimitiveMaterial(int index)
{
    LOG_INFO(3, "SolR_GetPrimitiveMaterial");
    return solr::SingletonKernel::kernel()->getPrimitiveMaterial(index);
}

int SolR_SetPrimitiveNormals(int index, double n0_x, double n0_y, double n0_z,
                             double n1_x, double n1_y, double n1_z, double n2_x,
                             double n2_y, double n2_z)
{
    LOG_INFO(3, "SolR_SetPrimitiveNormals");
    vec3f n0 = make_vec3f(static_cast<float>(n0_x), static_cast<float>(n0_y),
                          static_cast<float>(n0_z));
    vec3f n1 = make_vec3f(static_cast<float>(n1_x), static_cast<float>(n1_y),
                          static_cast<float>(n1_z));
    vec3f n2 = make_vec3f(static_cast<float>(n2_x), static_cast<float>(n2_y),
                          static_cast<float>(n2_z));
    solr::SingletonKernel::kernel()->setPrimitiveNormals(index, n0, n1, n2);
    return 0;
}

int SolR_SetPrimitiveTextureCoordinates(int index, double t0_x, double t0_y,
                                        double t1_x, double t1_y, double t2_x,
                                        double t2_y)
{
    LOG_INFO(3, "SolR_SetPrimitiveTextureCoordinates");
    const vec2f t0 =
        make_vec2f(static_cast<float>(t0_x), static_cast<float>(t0_y));
    const vec2f t1 =
        make_vec2f(static_cast<float>(t1_x), static_cast<float>(t1_y));
    const vec2f t2 =
        make_vec2f(static_cast<float>(t2_x), static_cast<float>(t2_y));
    solr::SingletonKernel::kernel()->setPrimitiveTextureCoordinates(index, t0,
                                                                    t1, t2);
    return 0;
}

// --------------------------------------------------------------------------------
int SolR_UpdateSkeletons(int index, double p0_x, double p0_y, double p0_z,
                         double size, double radius, int materialId,
                         double head_radius, int head_materialId,
                         double hands_radius, int hands_materialId,
                         double feet_radius, int feet_materialId)
{
    LOG_INFO(3, "SolR_UpdateSkeletons");
#if USE_KINECT
    vec3f position = {static_cast<float>(p0_x), static_cast<float>(p0_y),
                      static_cast<float>(p0_z)};
    return solr::SingletonKernel::kernel()->updateSkeletons(
        index, position, static_cast<float>(size), static_cast<float>(radius),
        materialId, static_cast<float>(head_radius), head_materialId,
        static_cast<float>(hands_radius), hands_materialId,
        static_cast<float>(feet_radius), feet_materialId);
#else
    return 0;
#endif // USE_KINECT
}

// --------------------------------------------------------------------------------
int SolR_LoadTextureFromFile(int index, char *filename)
{
    LOG_INFO(3, "SolR_LoadTextureFromFile");
    return solr::SingletonKernel::kernel()->loadTextureFromFile(index,
                                                                filename);
}

// --------------------------------------------------------------------------------
int SolR_SetTexture(int index, HANDLE texture)
{
    LOG_INFO(3, "SolR_SetTexture");
    /* TODO
 CPUTextureInformation texInfo;
 solr::SingletonKernel::kernel()->setTexture(
    index,
    static_cast<char*>(texture) );
 */
    return 0;
}

int SolR_GetTextureSize(int index, int &width, int &height, int &depth)
{
    LOG_INFO(3, "SolR_GetTextureSize");
    if (index < static_cast<int>(
                    solr::SingletonKernel::kernel()->getNbActiveTextures()))
    {
        TextureInfo texInfo;
        memset(&texInfo, 0, sizeof(TextureInfo));
        solr::SingletonKernel::kernel()->getTexture(index, texInfo);
        if (texInfo.buffer)
        {
            width = texInfo.size.x;
            height = texInfo.size.y;
            depth = texInfo.size.z;
        }
        return 0;
    }
    return 1;
}

int SolR_GetTexture(int index, BitmapBuffer *image)
{
    LOG_INFO(3, "SolR_GetTexture");
    if (index < static_cast<int>(
                    solr::SingletonKernel::kernel()->getNbActiveTextures()))
    {
        TextureInfo texInfo;
        memset(&texInfo, 0, sizeof(TextureInfo));
        solr::SingletonKernel::kernel()->getTexture(index, texInfo);
        if (texInfo.buffer)
        {
            int len(texInfo.size.x * texInfo.size.y * texInfo.size.z);
            for (int i(0); i < len; i += texInfo.size.z)
            {
                image[i] = texInfo.buffer[i + 2];
                image[i + 1] = texInfo.buffer[i + 1];
                image[i + 2] = texInfo.buffer[i];
            }
            // memcpy(image,texInfo.buffer,texInfo.size.x*texInfo.size.y*texInfo.size.z);
        }
        return 0;
    }
    return 1;
}

int SolR_GetNbTextures(int &nbTextures)
{
    LOG_INFO(3, "SolR_GetNbTextures");
    return solr::SingletonKernel::kernel()->getNbActiveTextures();
}

// ---------- Materials ----------
int SolR_AddMaterial()
{
    LOG_INFO(3, "SolR_AddMaterial");
    return solr::SingletonKernel::kernel()->addMaterial();
}

// --------------------------------------------------------------------------------
int SolR_SetMaterial(int index, double color_r, double color_g, double color_b,
                     double gloss, double reflection, double refraction,
                     int procedural, int wireframe, int wireframeDepth,
                     double transparency, double opacity, int diffuseTextureId,
                     int normalTextureId, int bumpTextureId,
                     int specularTextureId, int reflectionTextureId,
                     int transparencyTextureId, int ambientOcclusionTextureId,
                     double specValue, double specPower, double specCoef,
                     double innerIllumination, double illuminationDiffusion,
                     double illuminationPropagation, int fastTransparency)
{
    LOG_INFO(3, "SolR_SetMaterial ("
                    << index << "," << static_cast<float>(color_r) << ","
                    << static_cast<float>(color_g) << ","
                    << static_cast<float>(color_b) << ","
                    << static_cast<float>(gloss) << ","
                    << static_cast<float>(reflection) << ","
                    << static_cast<float>(refraction) << "," << procedural
                    << "," << wireframe << ","
                    << static_cast<int>(wireframeDepth) << ","
                    << static_cast<float>(transparency) << ","
                    << static_cast<float>(opacity) << ","
                    << static_cast<int>(diffuseTextureId) << ","
                    << static_cast<int>(normalTextureId) << ","
                    << static_cast<int>(bumpTextureId) << ","
                    << static_cast<int>(specularTextureId) << ","
                    << static_cast<int>(reflectionTextureId) << ","
                    << static_cast<int>(transparencyTextureId) << ","
                    << static_cast<int>(ambientOcclusionTextureId) << ","
                    << static_cast<float>(specValue) << ","
                    << static_cast<float>(specPower) << ","
                    << static_cast<float>(specCoef) << ","
                    << static_cast<float>(innerIllumination) << ","
                    << static_cast<float>(illuminationDiffusion) << ","
                    << static_cast<float>(illuminationPropagation) << ","
                    << fastTransparency << ")");

    solr::SingletonKernel::kernel()->setMaterial(
        index, static_cast<float>(color_r), static_cast<float>(color_g),
        static_cast<float>(color_b), static_cast<float>(gloss),
        static_cast<float>(reflection), static_cast<float>(refraction),
        (procedural == 1), (wireframe == 1), static_cast<int>(wireframeDepth),
        static_cast<float>(transparency), static_cast<float>(opacity),
        static_cast<int>(diffuseTextureId), static_cast<int>(normalTextureId),
        static_cast<int>(bumpTextureId), static_cast<int>(specularTextureId),
        static_cast<int>(reflectionTextureId),
        static_cast<int>(transparencyTextureId),
        static_cast<int>(ambientOcclusionTextureId),
        static_cast<float>(specValue), static_cast<float>(specPower),
        static_cast<float>(specCoef), static_cast<float>(innerIllumination),
        static_cast<float>(illuminationDiffusion),
        static_cast<float>(illuminationPropagation), (fastTransparency == 1));
    return 0;
}

// --------------------------------------------------------------------------------
int SolR_GetMaterial(
    int in_index, double &out_color_r, double &out_color_g, double &out_color_b,
    double &out_gloss, double &out_reflection, double &out_refraction,
    int &out_procedural, int &out_wireframe, int &out_wireframeDepth,
    double &out_transparency, double &out_opacity, int &out_diffuseTextureId,
    int &out_normalTextureId, int &out_bumpTextureId,
    int &out_specularTextureId, int &out_reflectionTextureId,
    int &out_transparencyTextureId, int &out_ambientOcclusionTextureId,
    double &out_specValue, double &out_specPower, double &out_specCoef,
    double &out_innerIllumination, double &out_illuminationDiffusion,
    double &out_illuminationPropagation, int &out_fastTransparency)
{
    LOG_INFO(3, "SolR_GetMaterial");
    float color_r, color_g, color_b, gloss, reflection, refraction,
        transparency, opacity;
    float specValue, specPower, specCoef, innerIllumination,
        illuminationDiffusion, illuminationPropagation;
    bool procedural;
    bool wireframe;
    int wireframeDepth;
    int diffuseTextureId, normalTextureId, bumpTextureId, specularTextureId,
        reflectionTextureId, transparencyTextureId, ambientOcclusionTextureId;
    bool fastTransparency;

    int returnValue = solr::SingletonKernel::kernel()->getMaterialAttributes(
        in_index, color_r, color_g, color_b, gloss, reflection, refraction,
        procedural, wireframe, wireframeDepth, transparency, opacity,
        diffuseTextureId, normalTextureId, bumpTextureId, specularTextureId,
        reflectionTextureId, transparencyTextureId, ambientOcclusionTextureId,
        specValue, specPower, specCoef, innerIllumination,
        illuminationDiffusion, illuminationPropagation, fastTransparency);

    out_color_r = static_cast<double>(color_r);
    out_color_g = static_cast<double>(color_g);
    out_color_b = static_cast<double>(color_b);
    out_gloss = static_cast<double>(gloss);
    out_reflection = static_cast<double>(reflection);
    out_refraction = static_cast<double>(refraction);
    out_transparency = static_cast<double>(transparency);
    out_opacity = static_cast<double>(opacity);
    out_diffuseTextureId = static_cast<int>(diffuseTextureId);
    out_normalTextureId = static_cast<int>(normalTextureId);
    out_bumpTextureId = static_cast<int>(bumpTextureId);
    out_specularTextureId = static_cast<int>(specularTextureId);
    out_reflectionTextureId = static_cast<int>(reflectionTextureId);
    out_transparencyTextureId = static_cast<int>(transparencyTextureId);
    out_ambientOcclusionTextureId = static_cast<int>(ambientOcclusionTextureId);
    out_procedural = procedural ? 1 : 0;
    out_wireframe = wireframe ? 1 : 0;
    out_wireframeDepth = wireframeDepth;
    out_specValue = static_cast<double>(specValue);
    out_specPower = static_cast<double>(specPower);
    out_specCoef = static_cast<double>(specCoef);
    out_innerIllumination = static_cast<double>(innerIllumination);
    out_illuminationDiffusion = static_cast<double>(illuminationDiffusion);
    out_illuminationPropagation = static_cast<double>(illuminationPropagation);
    out_fastTransparency = fastTransparency ? 1 : 0;
    return returnValue;
}

// --------------------------------------------------------------------------------
int SolR_LoadMolecule(char *filename, int geometryType, double defaultAtomSize,
                      double defaultStickSize, int atomMaterialType,
                      double scale)
{
    LOG_INFO(3, "SolR_LoadMolecule");
    // PDB
    solr::PDBReader prbReader;
    const float s(static_cast<float>(scale));
    const vec4f objectScale = make_vec4f(s, s, s);
    prbReader.loadAtomsFromFile(filename, *solr::SingletonKernel::kernel(),
                                static_cast<solr::GeometryType>(geometryType),
                                static_cast<float>(defaultAtomSize),
                                static_cast<float>(defaultStickSize),
                                atomMaterialType, objectScale);
    return solr::SingletonKernel::kernel()->getNbActivePrimitives();
}

// --------------------------------------------------------------------------------
int SolR_LoadOBJModel(char *filename, int materialId, int autoScale,
                      double scale, int autoCenter, double &height)
{
    LOG_INFO(3, "SolR_LoadOBJModel");
    vec4f center = make_vec4f(0.f, 0.f, 0.f);
    // PDB
    solr::OBJReader objReader;
    float s(static_cast<float>(scale));
    vec4f objectScale = make_vec4f(s, s, s);
    solr::CPUBoundingBox aabb;
    solr::CPUBoundingBox inAABB;
    vec4f minPos =
        objReader.loadModelFromFile(filename, *solr::SingletonKernel::kernel(),
                                    center, (autoScale == 1), objectScale, true,
                                    materialId, false, (autoCenter == 1), aabb,
                                    false, inAABB);

    height = -minPos.y / 2.f;
    return solr::SingletonKernel::kernel()->getNbActivePrimitives();
}

// --------------------------------------------------------------------------------
int SolR_SaveToFile(char *filename)
{
    LOG_INFO(3, "SolR_SaveToFile");
    solr::FileMarshaller fm;
    fm.saveToFile(*solr::SingletonKernel::kernel(), filename);
    return solr::SingletonKernel::kernel()->getNbActivePrimitives();
}

// --------------------------------------------------------------------------------
int SolR_LoadFromFile(char *filename, double scale)
{
    LOG_INFO(3, "SolR_LoadFromFile");
    vec4f center = make_vec4f(0.f, 0.f, 0.f);
    solr::FileMarshaller fm;
    fm.loadFromFile(*solr::SingletonKernel::kernel(), filename, center,
                    static_cast<float>(scale));
    return solr::SingletonKernel::kernel()->getNbActivePrimitives();
}

// --------------------------------------------------------------------------------
int SolR_CompactBoxes(bool update)
{
    LOG_INFO(3, "SolR_CompactBoxes");
    return solr::SingletonKernel::kernel()->compactBoxes(update);
}

// --------------------------------------------------------------------------------
int SolR_GetLight(int index)
{
    LOG_INFO(3, "SolR_GetLight");
    return solr::SingletonKernel::kernel()->getLight(index);
}

int SolR_GenerateScreenshot(char *filename, int width, int height, int quality)
{
    solr::SingletonKernel::kernel()->generateScreenshot(filename, width, height,
                                                        quality);
    return 0;
}

#ifdef USE_OPENCL
int SolR_GetOpenCLPlaformCount()
{
    solr::SingletonKernel::kernel()->queryDevice();
    return dynamic_cast<solr::OpenCLKernel *>(solr::SingletonKernel::kernel())
        ->getNumPlatforms();
}

int SolR_GetOpenCLPlatformDescription(int platform, char *value,
                                      int valueLength)
{
    std::string description =
        dynamic_cast<solr::OpenCLKernel *>(solr::SingletonKernel::kernel())
            ->getPlatformDescription(platform);
    strncpy(value, description.c_str(), valueLength);
    return 0;
}

int SolR_GetOpenCLDeviceCount(int platform)
{
    return dynamic_cast<solr::OpenCLKernel *>(solr::SingletonKernel::kernel())
        ->getNumDevices(platform);
}

int SolR_GetOpenCLDeviceDescription(int platform, int device, char *value,
                                    int valueLength)
{
    std::string description =
        dynamic_cast<solr::OpenCLKernel *>(solr::SingletonKernel::kernel())
            ->getDeviceDescription(platform, device);
    strncpy(value, description.c_str(), valueLength);
    return 0;
}

int SolR_PopulateOpenCLInformation()
{
    dynamic_cast<solr::OpenCLKernel *>(solr::SingletonKernel::kernel())
        ->queryDevice();
    return 0;
}

int SolR_RecompileKernels(char *filename)
{
    solr::SingletonKernel::kernel()->setKernelFilename(filename);
    solr::SingletonKernel::kernel()->recompileKernels();
    return 0;
}
#else
int SolR_GetOpenCLPlaformCount()
{
    return 0;
}

int SolR_GetOpenCLPlatformDescription(int platform, char *value,
                                      int valueLength)
{
    std::string description = "This library was not compiled for OpenCL";
    strncpy(value, description.c_str(), valueLength);
    return 0;
}

int SolR_GetOpenCLDeviceCount(int platform)
{
    return 0;
}

int SolR_GetOpenCLDeviceDescription(int platform, int device, char *value,
                                    int valueLength)
{
    std::string description = "This library was not compiled for OpenCL";
    strncpy(value, description.c_str(), valueLength);
    return 0;
}

int SolR_PopulateOpenCLInformation()
{
    return 0;
}

int SolR_RecompileKernels(char *)
{
    return 0;
}
#endif // USE_OPENCL
