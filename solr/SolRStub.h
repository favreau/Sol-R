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

// --------------------------------------------------------------------
// IMPORTANT!!! C++ exceptions must be disabled in the compiler options
// --------------------------------------------------------------------

#pragma once

#ifdef WIN32
#include <windows.h>
#else
typedef char *HANDLE;
#endif

#include "api.h"
#include "engines/GPUKernel.h"

// ---------- OpenCL ----------
extern "C" SOLR_API int SolR_PopulateOpenCLInformation();
extern "C" SOLR_API int SolR_GetOpenCLPlaformCount();
extern "C" SOLR_API int SolR_GetOpenCLPlatformDescription(int platform, char *value, int valueLength);
extern "C" SOLR_API int SolR_GetOpenCLDeviceCount(int platform);
extern "C" SOLR_API int SolR_GetOpenCLDeviceDescription(int platform, int device, char *value, int valueLength);
extern "C" SOLR_API int SolR_RecompileKernels(char *filename);

// ---------- Scene ----------
extern "C" SOLR_API int SolR_SetSceneInfo(int width, int height, int graphicsLevel, int nbRayIterations,
                                          double transparentColor, double viewDistance, double shadowIntensity,
                                          double eyeSeparation, double bgColorR, double bgColorG, double bgColorB,
                                          double bgColorA, int renderBoxes, int pathTracingIteration,
                                          int maxPathTracingIterations, int frameBufferType, int timestamp,
                                          int atmosphericEffect, int cameraType, int doubleSidedTriangles,
                                          int extendedGeometry, int advancedIllumination, int skyboxSize,
                                          int skyboxMaterialId, double geometryEpsilon, double rayEpsilon);

extern "C" SOLR_API int SolR_SetPostProcessingInfo(int type, double param1, double param2, int param3);

extern "C" SOLR_API int SolR_SetDraftMode(int draft);

extern "C" SOLR_API int SolR_InitializeKernel(bool activeLogging, int platform, int device);
extern "C" SOLR_API int SolR_FinalizeKernel();
extern "C" SOLR_API int SolR_ResetKernel();

extern "C" SOLR_API int SolR_GenerateScreenshot(char *filename, int width, int height, int quality);

// ---------- Camera ----------
extern "C" SOLR_API void SolR_SetCamera(double eye_x, double eye_y, double eye_z, double dir_x, double dir_y,
                                        double dir_z, double angle_x, double angle_y, double angle_z);

// ---------- Rendering ----------
extern "C" SOLR_API int SolR_RunKernel(double timer, solr::BitmapBuffer *image);

// ---------- Primitives ----------
extern "C" SOLR_API int SolR_AddPrimitive(int type, int movable);

extern "C" SOLR_API int SolR_SetPrimitive(int index, double p0_x, double p0_y, double p0_z, double p1_x, double p1_y,
                                          double p1_z, double p2_x, double p2_y, double p2_z, double size_x,
                                          double size_y, double size_z, int materialId);

extern "C" SOLR_API int SolR_GetPrimitive(int index, double &p0_x, double &p0_y, double &p0_z, double &p1_x,
                                          double &p1_y, double &p1_z, double &p2_x, double &p2_y, double &p2_z,
                                          double &size_x, double &size_y, double &size_z, int &materialId);

extern "C" SOLR_API int SolR_GetPrimitiveAt(int x, int y);

extern "C" SOLR_API int SolR_GetPrimitiveCenter(int index, double &x, double &y, double &z);

extern "C" SOLR_API int SolR_RotatePrimitive(int index, double rx, double ry, double rz, double ax, double ay,
                                             double az);

extern "C" SOLR_API int SolR_RotatePrimitives(int fromBoxId, int toBoxId, double rx, double ry, double rz, double ax,
                                              double ay, double az);

extern "C" SOLR_API int SolR_SetPrimitiveMaterial(int index, int materialId);

extern "C" SOLR_API int SolR_GetPrimitiveMaterial(int index);

extern "C" SOLR_API int SolR_SetPrimitiveNormals(int index, double n0_x, double n0_y, double n0_z, double n1_x,
                                                 double n1_y, double n1_z, double n2_x, double n2_y, double n2_z);

extern "C" SOLR_API int SolR_SetPrimitiveTextureCoordinates(int index, double t0_x, double t0_y, double t1_x,
                                                            double t1_y, double t2_x, double t2_y);

// ---------- Materials ----------
extern "C" SOLR_API int SolR_AddMaterial();
extern "C" SOLR_API int SolR_SetMaterial(int index, double color_r, double color_g, double color_b, double gloss,
                                         double reflection, double refraction, int procedural, int wireframe,
                                         int wireframeDepth, double transparency, double opacity, int diffuseTextureId,
                                         int normalTextureId, int bumpTextureId, int specularTextureId,
                                         int reflectionTextureId, int transparencyTextureId,
                                         int ambientOcclusionTextureId, double specValue, double specPower,
                                         double specCoef, double innerIllumination, double illuminationDiffusion,
                                         double illuminationPropagation, int fastTransparency);

extern "C" SOLR_API int SolR_GetMaterial(int index, double &color_r, double &color_g, double &color_b, double &gloss,
                                         double &reflection, double &refraction, int &procedural, int &wireframe,
                                         int &wireframeDepth, double &transparency, double &opacity,
                                         int &diffuseTextureId, int &bumpTextureId, int &normalTextureId,
                                         int &specularTextureId, int &reflectionTextureId, int &transparencyTextureId,
                                         int &ambientOcclusionTextureId, double &specValue, double &specPower,
                                         double &specCoef, double &innerIllumination, double &illuminationDiffusion,
                                         double &illuminationPropagation, int &fastTransparency);

// Boxes
extern "C" SOLR_API int SolR_CompactBoxes(bool update);

// ---------- Lights ----------
extern "C" SOLR_API int SolR_GetLight(int index);

// ---------- Textures ----------
extern "C" SOLR_API int SolR_LoadTextureFromFile(int index, char *filename);
extern "C" SOLR_API int SolR_SetTexture(int index, HANDLE texture);
extern "C" SOLR_API int SolR_GetTextureSize(int index, int &width, int &height, int &depth);
extern "C" SOLR_API int SolR_GetTexture(int index, solr::BitmapBuffer *image);
extern "C" SOLR_API int SolR_GetNbTextures(int &nbTextures);

// ---------- Molecules ----------
extern "C" SOLR_API int SolR_LoadMolecule(char *filename, int geometryType, double defaultAtomSize,
                                          double defaultStickSize, int atomMaterialType, double scale);

// ---------- OBJ models ----------
extern "C" SOLR_API int SolR_LoadOBJModel(char *filename, int materialId, int autoScale, double scale, int autoCenter,
                                          double &height);

// ---------- File marshaller ----------
extern "C" SOLR_API int SolR_SaveToFile(char *filename);
extern "C" SOLR_API int SolR_LoadFromFile(char *filename, double scale);

#ifdef USE_KINECT
// ---------- Kinect ----------
extern "C" SOLR_API int SolR_UpdateSkeletons(int index, double center_x, double center_y, double center_z, double size,
                                             double radius, int materialId, double head_radius, int head_materialId,
                                             double hands_radius, int hands_materialId, double feet_radius,
                                             int feet_materialId);
#endif // USE_KINECT
