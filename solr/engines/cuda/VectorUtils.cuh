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

#pragma once

// Project
#include "../../solr.h"

// Cuda
#include "helper_cuda.h"
#include "helper_math.h"

using namespace solr;

// _____________________________________________________________________________________________________________________
__device__ __INLINE__ void saturateVector(float4 &v)
{
    v.x = (v.x < 0.f) ? 0.f : v.x;
    v.y = (v.y < 0.f) ? 0.f : v.y;
    v.z = (v.z < 0.f) ? 0.f : v.z;
    v.w = (v.w < 0.f) ? 0.f : v.w;

    v.x = (v.x > 1.f) ? 1.f : v.x;
    v.y = (v.y > 1.f) ? 1.f : v.y;
    v.z = (v.z > 1.f) ? 1.f : v.z;
    v.w = (v.w > 1.f) ? 1.f : v.w;
}

// _____________________________________________________________________________________________________________________
__device__ __INLINE__ vec3f crossProduct(const vec3f &b, const vec3f &c)
{
    vec3f a;
    a.x = b.y * c.z - b.z * c.y;
    a.y = b.z * c.x - b.x * c.z;
    a.z = b.x * c.y - b.y * c.x;
    return a;
}

/*
________________________________________________________________________________________________________________________
incident  : le vecteur normal inverse a la direction d'incidence de la source
lumineuse normal    : la normale a l'interface orientee dans le materiau ou se
propage le rayon incident reflected : le vecteur normal reflechi
________________________________________________________________________________________________________________________
*/
__device__ __INLINE__ void vectorReflection(vec3f &r, const vec3f &i,
                                            const vec3f &n)
{
    r = i - 2.f * dot(i, n) * n;
}

/*
________________________________________________________________________________________________________________________
incident: le vecteur norm? inverse ? la direction d?incidence de la source
lumineuse n1      : index of refraction of original medium n2      : index of
refraction of new medium
________________________________________________________________________________________________________________________
*/
__device__ __INLINE__ void vectorRefraction(vec3f &refracted,
                                            const vec3f incident,
                                            const float n1, const vec3f normal,
                                            const float n2)
{
    refracted = incident;
    if (n2 != 0.f)
    {
        float eta = n1 / n2;
        float c1 = -dot(incident, normal);
        float cs2 = 1.f - eta * eta * (1.f - c1 * c1);
        if (cs2 >= 0.f)
        {
            refracted = eta * incident + (eta * c1 - sqrt(cs2)) * normal;
        }
    }
}

/*
________________________________________________________________________________________________________________________
*/
__device__ __INLINE__ vec3f project(const vec3f &A, const vec3f &B)
{
    return B * (dot(A, B) / dot(B, B));
}

/*
________________________________________________________________________________________________________________________
__v : Vector to rotate
__c : Center of rotations
__a : Angles
________________________________________________________________________________________________________________________
*/
__device__ __INLINE__ void vectorRotation(vec3f &v, const vec3f &rotationCenter,
                                          const vec4f &angles)
{
    vec4f cosAngles, sinAngles;

    cosAngles.x = cosf(angles.x);
    cosAngles.y = cosf(angles.y);
    cosAngles.z = cosf(angles.z);

    sinAngles.x = sinf(angles.x);
    sinAngles.y = sinf(angles.y);
    sinAngles.z = sinf(angles.z);

    // Rotate Center
    vec4f vector;
    vector.x = v.x - rotationCenter.x;
    vector.y = v.y - rotationCenter.y;
    vector.z = v.z - rotationCenter.z;
    vec4f result = vector;

    /* X axis */
    result.y = vector.y * cosAngles.x - vector.z * sinAngles.x;
    result.z = vector.y * sinAngles.x + vector.z * cosAngles.x;
    vector = result;
    result = vector;

    /* Y axis */
    result.z = vector.z * cosAngles.y - vector.x * sinAngles.y;
    result.x = vector.z * sinAngles.y + vector.x * cosAngles.y;
    vector = result;
    result = vector;

    /* Z axis */
    result.x = vector.x * cosAngles.z - vector.y * sinAngles.z;
    result.y = vector.x * sinAngles.z + vector.y * cosAngles.z;

    v.x = result.x + rotationCenter.x;
    v.y = result.y + rotationCenter.y;
    v.z = result.z + rotationCenter.z;
}
