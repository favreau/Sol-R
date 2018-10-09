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

#pragma once

#include <solr.h>

using namespace solr;

extern "C" void initialize_scene(vec2i occupancyParameters, SceneInfo sceneInfo, int nbPrimitives, int nbLamps,
                                 int nbMaterials
#ifdef USE_MANAGED_MEMORY
                                 ,
                                 BoundingBox *&boundingBoxes, Primitive *&primitives
#endif
);

extern "C" void finalize_scene(vec2i occupancyParameters
#ifdef USE_MANAGED_MEMORY
                               ,
                               BoundingBox *boundingBoxes, Primitive *primitives
#endif
);

extern "C" void reshape_scene(vec2i occupancyParameters, SceneInfo sceneInfo);

extern "C" void h2d_scene(vec2i occupancyParameters, BoundingBox *boundingBoxes, int nbActiveBoxes,
                          Primitive *primitives, int nbPrimitives, Lamp *lamps, int nbLamps);

extern "C" void h2d_materials(vec2i occupancyParameters, Material *materials, int nbActiveMaterials);

extern "C" void h2d_randoms(vec2i occupancyParameters, float *randoms);

extern "C" void h2d_textures(vec2i occupancyParameters, int activeTextures, TextureInfo *textureInfos);

extern "C" void h2d_lightInformation(vec2i occupancyParameters, LightInformation *lightInformation,
                                     int lightInformationSize);

extern "C" void d2h_bitmap(vec2i occupancyParameters, SceneInfo sceneInfo, BitmapBuffer *bitmap,
                           PrimitiveXYIdBuffer *primitivesXYIds);

extern "C" void cudaRender(vec2i occupancyParameters, vec4i blockSize, SceneInfo sceneInfo, vec4i objects,
                           PostProcessingInfo PostProcessingInfo, vec3f origin, vec3f direction, vec4f angles
#ifdef USE_MANAGED_MEMORY
                           ,
                           BoundingBox *boundingBoxes, Primitive *primitives
#endif
);

#ifdef USE_KINECT
extern "C" void h2d_kinect(vec2i occupancyParameters, BitmapBuffer *video, BitmapBuffer *depth);
#endif // USE_KINECT
