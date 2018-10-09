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

// CUDA Managed Memory
#undef USE_MANAGED_MEMORY

// Consts
#define MAX_GPU_COUNT 32
#define MAX_STREAM_COUNT 32
#define NB_MAX_ITERATIONS 10

const unsigned int BOUNDING_BOXES_TREE_DEPTH = 64;
const unsigned int NB_MAX_BOXES = 2500000;
const unsigned int NB_MAX_PRIMITIVES = 2500000;
const unsigned int NB_MAX_LAMPS = 512;
const unsigned int NB_MAX_MATERIALS = 65506 + 30; // Last 30 materials are reserved
const unsigned int NB_MAX_TEXTURES = 512;
const unsigned int NB_MAX_FRAMES = 512;
const unsigned int NB_MAX_LIGHTINFORMATIONS = 512;
const unsigned int MAX_BITMAP_WIDTH = 1920;
const unsigned int MAX_BITMAP_HEIGHT = 1080;
const unsigned int MAX_BITMAP_SIZE = MAX_BITMAP_WIDTH * MAX_BITMAP_HEIGHT;

// Constants
const int MATERIAL_NONE = -1;
const int TEXTURE_NONE = -1;
const int TEXTURE_MANDELBROT = -2;
const int TEXTURE_JULIA = -3;
const int gColorDepth = 3;

// Globals
#define PI 3.14159265358979323846f
#define MATERIAL_DEFAULT_EMMISION_STRENGTH 0.6f
#define SKYBOX_LUNINANCE_STRENGTH 0.2f
#define EXTENDED_GEOMETRY // Includes spheres, cylinders, etc
#define NATURAL_DEPTHOFFIELD

#undef VOLUME_RENDERING_NORMALS

// Where random materials start
const unsigned int RANDOM_MATERIALS_OFFSET = 1000;

// Skybox matrerials
const unsigned int SKYBOX_FRONT_MATERIAL = NB_MAX_MATERIALS - 2;
const unsigned int SKYBOX_RIGHT_MATERIAL = NB_MAX_MATERIALS - 3;
const unsigned int SKYBOX_BACK_MATERIAL = NB_MAX_MATERIALS - 4;
const unsigned int SKYBOX_LEFT_MATERIAL = NB_MAX_MATERIALS - 5;
const unsigned int SKYBOX_TOP_MATERIAL = NB_MAX_MATERIALS - 6;
const unsigned int SKYBOX_BOTTOM_MATERIAL = NB_MAX_MATERIALS - 7;

// Ground material
const unsigned int SKYBOX_GROUND_MATERIAL = NB_MAX_MATERIALS - 8;

// Cornell Box material
const unsigned int CORNELLBOX_FRONT_MATERIAL = NB_MAX_MATERIALS - 9;
const unsigned int CORNELLBOX_RIGHT_MATERIAL = NB_MAX_MATERIALS - 10;
const unsigned int CORNELLBOX_BACK_MATERIAL = NB_MAX_MATERIALS - 11;
const unsigned int CORNELLBOX_LEFT_MATERIAL = NB_MAX_MATERIALS - 12;
const unsigned int CORNELLBOX_TOP_MATERIAL = NB_MAX_MATERIALS - 13;
const unsigned int CORNELLBOX_BOTTOM_MATERIAL = NB_MAX_MATERIALS - 14;
const unsigned int CORNELLBOX_GROUND_MATERIAL = NB_MAX_MATERIALS - 15;

// Fractals
const unsigned int MANDELBROT_MATERIAL = NB_MAX_MATERIALS - 16;
const unsigned int JULIA_MATERIAL = NB_MAX_MATERIALS - 17;

// Basic reflection materials
const unsigned int BASIC_REFLECTION_MATERIAL_001 = NB_MAX_MATERIALS - 18;
const unsigned int BASIC_REFLECTION_MATERIAL_002 = NB_MAX_MATERIALS - 19;
const unsigned int BASIC_REFLECTION_MATERIAL_003 = NB_MAX_MATERIALS - 20;
const unsigned int BASIC_REFLECTION_MATERIAL_004 = NB_MAX_MATERIALS - 21;
const unsigned int BASIC_REFLECTION_MATERIAL_005 = NB_MAX_MATERIALS - 22;
const unsigned int BASIC_REFLECTION_MATERIAL_006 = NB_MAX_MATERIALS - 23;

// Light source materials
const unsigned int LIGHT_MATERIAL_001 = NB_MAX_MATERIALS - 24;
const unsigned int LIGHT_MATERIAL_002 = NB_MAX_MATERIALS - 25;
const unsigned int LIGHT_MATERIAL_003 = NB_MAX_MATERIALS - 26;
const unsigned int LIGHT_MATERIAL_004 = NB_MAX_MATERIALS - 27;
const unsigned int LIGHT_MATERIAL_005 = NB_MAX_MATERIALS - 28;
const unsigned int LIGHT_MATERIAL_006 = NB_MAX_MATERIALS - 29;
const unsigned int LIGHT_MATERIAL_007 = NB_MAX_MATERIALS - 30;
const unsigned int LIGHT_MATERIAL_008 = NB_MAX_MATERIALS - 31;
const unsigned int LIGHT_MATERIAL_009 = NB_MAX_MATERIALS - 32;
const unsigned int LIGHT_MATERIAL_010 = NB_MAX_MATERIALS - 33;
const unsigned int DEFAULT_LIGHT_MATERIAL = NB_MAX_MATERIALS - 34;

// Basic color materials
const unsigned int WHITE_MATERIAL = NB_MAX_MATERIALS - 35;
const unsigned int RED_MATERIAL = NB_MAX_MATERIALS - 36;
const unsigned int GREEN_MATERIAL = NB_MAX_MATERIALS - 37;
const unsigned int BLUE_MATERIAL = NB_MAX_MATERIALS - 38;
const unsigned int YELLOW_MATERIAL = NB_MAX_MATERIALS - 39;
const unsigned int PURPLE_MATERIAL = NB_MAX_MATERIALS - 40;

// Ground material
const unsigned int SKYBOX_SPHERE_MATERIAL = NB_MAX_MATERIALS - 41;

// Kinect
const unsigned int KINECT_COLOR_MATERIAL = NB_MAX_MATERIALS - 42;
const unsigned int KINECT_DEPTH_MATERIAL = NB_MAX_MATERIALS - 43;

// Metal
const unsigned int METAL_MATERIAL = NB_MAX_MATERIALS - 44;

const unsigned int KINECT_COLOR_TEXTURE = 0;
const unsigned int KINECT_DEPTH_TEXTURE = 1;

const unsigned int KINECT_COLOR_WIDTH = 640;
const unsigned int KINECT_COLOR_HEIGHT = 480;
const unsigned int KINECT_COLOR_DEPTH = 4;
const unsigned int KINECT_COLOR_SIZE = KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * KINECT_COLOR_DEPTH;

const unsigned int KINECT_DEPTH_WIDTH = 320;
const unsigned int KINECT_DEPTH_HEIGHT = 240;
const unsigned int KINECT_DEPTH_DEPTH = 2;
const unsigned int KINECT_DEPTH_SIZE = KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT * KINECT_DEPTH_DEPTH;
