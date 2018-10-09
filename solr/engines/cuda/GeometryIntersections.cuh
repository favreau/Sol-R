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
//#define VOLUME_RENDERING_NORMALS

// Project
#include "../../solr.h"
#include "VectorUtils.cuh"
#include "GeometryShaders.cuh"
#include "TextureMapping.cuh"

/*
________________________________________________________________________________

Compute ray attributes
________________________________________________________________________________
*/
__device__ __INLINE__ void computeRayAttributes(Ray &ray)
{
    ray.inv_direction.x = ray.direction.x != 0.f ? 1.f / ray.direction.x : 1.f;
    ray.inv_direction.y = ray.direction.y != 0.f ? 1.f / ray.direction.y : 1.f;
    ray.inv_direction.z = ray.direction.z != 0.f ? 1.f / ray.direction.z : 1.f;
    ray.signs.x = (ray.inv_direction.x < 0);
    ray.signs.y = (ray.inv_direction.y < 0);
    ray.signs.z = (ray.inv_direction.z < 0);
}

/*
________________________________________________________________________________

Box intersection
________________________________________________________________________________
*/
__device__ __INLINE__ bool boxIntersection(const BoundingBox &box, const Ray &ray, const float &t0, const float &t1)
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    tmin = (box.parameters[ray.signs.x].x - ray.origin.x) * ray.inv_direction.x;
    tmax = (box.parameters[1 - ray.signs.x].x - ray.origin.x) * ray.inv_direction.x;
    tymin = (box.parameters[ray.signs.y].y - ray.origin.y) * ray.inv_direction.y;
    tymax = (box.parameters[1 - ray.signs.y].y - ray.origin.y) * ray.inv_direction.y;

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    tzmin = (box.parameters[ray.signs.z].z - ray.origin.z) * ray.inv_direction.z;
    tzmax = (box.parameters[1 - ray.signs.z].z - ray.origin.z) * ray.inv_direction.z;

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    return ((tmin < t1) && (tmax > t0));
}

/*
________________________________________________________________________________

Skybox mapping
________________________________________________________________________________
*/
__device__ __INLINE__ vec4f skyboxMapping(const SceneInfo &sceneInfo, Material *materials, BitmapBuffer *textures,
                                          const Ray &ray)
{
    Material &material = materials[sceneInfo.skyboxMaterialId];
    vec4f result = material.color;
    // solve the equation sphere-ray to find the intersections
    vec3f dir = normalize(ray.direction - ray.origin);

    float a = 2.f * dot(dir, dir);
    float b = 2.f * dot(ray.origin, dir);
    float c = dot(ray.origin, ray.origin) - (sceneInfo.skyboxRadius * sceneInfo.skyboxRadius);
    float d = b * b - 2.f * a * c;

    if (d <= 0.f || a == 0.f)
        return result;
    float r = sqrt(d);
    float t1 = (-b - r) / a;
    float t2 = (-b + r) / a;

    if (t1 <= sceneInfo.geometryEpsilon && t2 <= sceneInfo.geometryEpsilon)
        return result; // both intersections are behind the ray origin

    float t = 0.f;
    if (t1 <= sceneInfo.geometryEpsilon)
        t = t2;
    else if (t2 <= sceneInfo.geometryEpsilon)
        t = t1;
    else
        t = (t1 < t2) ? t1 : t2;

    if (t < sceneInfo.geometryEpsilon)
        return result; // Too close to intersection
    vec3f intersection = normalize(ray.origin + t * dir);

    // Intersection found, now get skybox color
    float U = ((atan2(intersection.x, intersection.z) / PI) + 1.f) * .5f;
    float V = (asin(intersection.y) / PI) + .5f;

    int u = int(material.textureMapping.x * U);
    int v = int(material.textureMapping.y * V);

    if (material.textureMapping.x != 0)
        u %= material.textureMapping.x;
    if (material.textureMapping.y != 0)
        v %= material.textureMapping.y;

    if (u >= 0 && u < material.textureMapping.x && v >= 0 && v < material.textureMapping.y)
    {
        int A = (v * material.textureMapping.x + u) * material.textureMapping.w;
        int B = material.textureMapping.x * material.textureMapping.y * material.textureMapping.w;
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
    }

    return result;
}

/*
________________________________________________________________________________

Ellipsoid intersection
________________________________________________________________________________
*/
__device__ __INLINE__ bool ellipsoidIntersection(const SceneInfo &sceneInfo, const Primitive &ellipsoid,
                                                 Material *materials, const Ray &ray, vec3f &intersection,
                                                 vec3f &normal, float &shadowIntensity)
{
    // Shadow intensity
    shadowIntensity = 1.f;

    // solve the equation sphere-ray to find the intersections
    vec3f O_C = ray.origin - ellipsoid.p0;
    vec3f dir = normalize(ray.direction);

    float a = ((dir.x * dir.x) / (ellipsoid.size.x * ellipsoid.size.x)) +
              ((dir.y * dir.y) / (ellipsoid.size.y * ellipsoid.size.y)) +
              ((dir.z * dir.z) / (ellipsoid.size.z * ellipsoid.size.z));
    float b = ((2.f * O_C.x * dir.x) / (ellipsoid.size.x * ellipsoid.size.x)) +
              ((2.f * O_C.y * dir.y) / (ellipsoid.size.y * ellipsoid.size.y)) +
              ((2.f * O_C.z * dir.z) / (ellipsoid.size.z * ellipsoid.size.z));
    float c = ((O_C.x * O_C.x) / (ellipsoid.size.x * ellipsoid.size.x)) +
              ((O_C.y * O_C.y) / (ellipsoid.size.y * ellipsoid.size.y)) +
              ((O_C.z * O_C.z) / (ellipsoid.size.z * ellipsoid.size.z)) - 1.f;

    float d = ((b * b) - (4.f * a * c));
    if (d < 0.f || a == 0.f || b == 0.f || c == 0.f)
    {
        return false;
    }
    d = sqrt(d);

    float t1 = (-b + d) / (2.f * a);
    float t2 = (-b - d) / (2.f * a);

    if (t1 <= sceneInfo.geometryEpsilon && t2 <= sceneInfo.geometryEpsilon)
        return false; // both intersections are behind the ray origin

    float t = 0.f;
    if (t1 <= sceneInfo.geometryEpsilon)
        t = t2;
    else if (t2 <= sceneInfo.geometryEpsilon)
        t = t1;
    else
        t = (t1 < t2) ? t1 : t2;

    if (t < sceneInfo.geometryEpsilon)
        return false; // Too close to intersection
    intersection = ray.origin + t * dir;

    normal = intersection - ellipsoid.p0;
    normal.x = 2.f * normal.x / (ellipsoid.size.x * ellipsoid.size.x);
    normal.y = 2.f * normal.y / (ellipsoid.size.y * ellipsoid.size.y);
    normal.z = 2.f * normal.z / (ellipsoid.size.z * ellipsoid.size.z);

    normal = normalize(normal);
    return true;
}

/*
________________________________________________________________________________

Sphere intersection
________________________________________________________________________________
*/
__device__ __INLINE__ bool sphereIntersection(const SceneInfo &sceneInfo, const Primitive &sphere, Material *materials,
                                              const Ray &ray, vec3f &intersection, vec3f &normal,
                                              float &shadowIntensity)
{
    bool back = false;
    // solve the equation sphere-ray to find the intersections
    vec3f O_C = ray.origin - sphere.p0;
    vec3f dir = normalize(ray.direction);

    float a = 2.f * dot(dir, dir);
    float b = 2.f * dot(O_C, dir);
    float c = dot(O_C, O_C) - (sphere.size.x * sphere.size.x);
    float d = b * b - 2.f * a * c;

    if (d <= 0.f || a == 0.f)
        return false;
    float r = sqrt(d);
    float t1 = (-b - r) / a;
    float t2 = (-b + r) / a;

    if (t1 <= sceneInfo.geometryEpsilon && t2 <= sceneInfo.geometryEpsilon)
        return false; // both intersections are behind the ray origin

    float t = 0.f;
    if (t1 <= sceneInfo.geometryEpsilon)
    {
        t = t2;
        back = true;
    }
    else if (t2 <= sceneInfo.geometryEpsilon)
        t = t1;
    else
        t = (t1 < t2) ? t1 : t2;

    if (t < sceneInfo.geometryEpsilon)
        return false; // Too close to intersection
    intersection = ray.origin + t * dir;

    // TO REMOVE - For Charts only
    // if( intersection.y < sphere.p0.y ) return false;

    if (materials[sphere.materialId].attributes.y == 0)
    {
        // Compute normal vector
        normal = intersection - sphere.p0;
    }
    else
    {
        // Procedural texture
        vec3f newCenter;
        newCenter.x = sphere.p0.x + 0.008f * sphere.size.x * cos(sceneInfo.timestamp + intersection.x);
        newCenter.y = sphere.p0.y + 0.008f * sphere.size.y * sin(sceneInfo.timestamp + intersection.y);
        newCenter.z = sphere.p0.z + 0.008f * sphere.size.z * sin(cos(sceneInfo.timestamp + intersection.z));
        normal = intersection - newCenter;
    }
    normal = normalize(normal);
    if (back)
        normal *= -1.f;

    // Shadow management
    r = dot(dir, normal);
    shadowIntensity = (materials[sphere.materialId].transparency != 0.f) ? (1.f - fabs(r)) : 1.f;

    return true;
}

/*
________________________________________________________________________________

Cylinder intersection
ref: http://courses.cms.caltech.edu/cs11/material/advcpp/lab7/index.html
________________________________________________________________________________
*/
__device__ __INLINE__ bool cylinderIntersection(const SceneInfo &sceneInfo, const Primitive &cylinder,
                                                Material *materials, const Ray &ray, vec3f &intersection, vec3f &normal,
                                                float &shadowIntensity)
{
    vec3f O_C = ray.origin - cylinder.p0;
    vec3f dir = ray.direction;
    vec3f n = crossProduct(dir, cylinder.n1);
    float ln = length(n);

    // Parallel? (?)
    if ((ln < sceneInfo.geometryEpsilon) && (ln > -sceneInfo.geometryEpsilon))
        return false;

    n = normalize(n);

    float d = fabs(dot(O_C, n));
    if (d > cylinder.size.y)
        return false;

    vec3f O = crossProduct(O_C, cylinder.n1);
    float t = -dot(O, n) / ln;
    if (t < 0.f)
        return false;

    O = normalize(crossProduct(n, cylinder.n1));
    float s = fabs(sqrtf(cylinder.size.x * cylinder.size.x - d * d) / dot(dir, O));

    float t1 = t - s;
    float t2 = t + s;

    // Calculate intersection point
    intersection = ray.origin + t1 * dir;
    vec3f HB1 = intersection - cylinder.p0;
    vec3f HB2 = intersection - cylinder.p1;
    float scale1 = dot(HB1, cylinder.n1);
    float scale2 = dot(HB2, cylinder.n1);
    // Cylinder length
    if (scale1 < sceneInfo.geometryEpsilon || scale2 > sceneInfo.geometryEpsilon)
    {
        intersection = ray.origin + t2 * dir;
        HB1 = intersection - cylinder.p0;
        HB2 = intersection - cylinder.p1;
        scale1 = dot(HB1, cylinder.n1);
        scale2 = dot(HB2, cylinder.n1);
        // Cylinder length
        if (scale1 < sceneInfo.geometryEpsilon || scale2 > sceneInfo.geometryEpsilon)
            return false;
    }

    vec3f V = intersection - cylinder.p2;
    normal = V - project(V, cylinder.n1);
    normal = normalize(normal);

    // Shadow management
    shadowIntensity = 1.f;
    return true;
}

/*
________________________________________________________________________________

Cone intersection
ref: http://courses.cms.caltech.edu/cs11/material/advcpp/lab7/index.html
________________________________________________________________________________
*/
__device__ __INLINE__ bool coneIntersection(const SceneInfo &sceneInfo, const Primitive &cone, Material *materials,
                                            const Ray &ray, vec3f &intersection, vec3f &normal, float &shadowIntensity)
{
    vec3f O_C = ray.origin - cone.p0;
    vec3f dir = ray.direction;
    vec3f n = crossProduct(dir, cone.n1);

    float ln = length(n);

    // Parallel? (?)
    if ((ln < sceneInfo.geometryEpsilon) && (ln > -sceneInfo.geometryEpsilon))
        return false;

    n = normalize(n);

    float d = fabs(dot(O_C, n));
    if (d > cone.size.y)
        return false;

    vec3f O = crossProduct(O_C, cone.n1);
    float t = -dot(O, n) / ln;
    if (t < 0.f)
        return false;

    O = normalize(crossProduct(n, cone.n1));
    float s = fabs(sqrtf(cone.size.x * cone.size.x - d * d) / dot(dir, O));

    float t1 = t - s;
    float t2 = t + s;

    // Calculate intersection point
    intersection = ray.origin + t1 * dir;
    vec3f HB1 = intersection - cone.p0;
    vec3f HB2 = intersection - cone.p1;
    float scale1 = dot(HB1, cone.n1);
    float scale2 = dot(HB2, cone.n1);
    // Cylinder length
    if (scale1 < sceneInfo.geometryEpsilon || scale2 > sceneInfo.geometryEpsilon)
    {
        intersection = ray.origin + t2 * dir;
        HB1 = intersection - cone.p0;
        HB2 = intersection - cone.p1;
        scale1 = dot(HB1, cone.n1);
        scale2 = dot(HB2, cone.n1);
        // Cylinder length
        if (scale1 < sceneInfo.geometryEpsilon || scale2 > sceneInfo.geometryEpsilon)
            return false;
    }

    vec3f V = intersection - cone.p2;
    normal = V - project(V, cone.n1);
    normal = normalize(normal);

    // Shadow management
    dir = normalize(dir);
    float r = dot(dir, normal);
    shadowIntensity = 1.f;
    return true;
}

/*
________________________________________________________________________________

Checkboard intersection
________________________________________________________________________________
*/
__device__ __INLINE__ bool planeIntersection(const SceneInfo &sceneInfo, const Primitive &primitive,
                                             Material *materials, BitmapBuffer *textures, const Ray &ray,
                                             vec3f &intersection, vec3f &normal, float &shadowIntensity, bool reverse)
{
    bool collision = false;

    float reverted = reverse ? -1.f : 1.f;
    normal = primitive.n0;
    switch (primitive.type)
    {
    case ptMagicCarpet:
    case ptCheckboard:
    {
        intersection.y = primitive.p0.y;
        float y = ray.origin.y - primitive.p0.y;
        if (reverted * ray.direction.y < 0.f && reverted * ray.origin.y > reverted * primitive.p0.y)
        {
            intersection.x = ray.origin.x + y * ray.direction.x / -ray.direction.y;
            intersection.z = ray.origin.z + y * ray.direction.z / -ray.direction.y;
            collision = fabs(intersection.x - primitive.p0.x) < primitive.size.x &&
                        fabs(intersection.z - primitive.p0.z) < primitive.size.z;
        }
        break;
    }
    case ptXZPlane:
    {
        float y = ray.origin.y - primitive.p0.y;
        if (reverted * ray.direction.y < 0.f && reverted * ray.origin.y > reverted * primitive.p0.y)
        {
            intersection.x = ray.origin.x + y * ray.direction.x / -ray.direction.y;
            intersection.y = primitive.p0.y;
            intersection.z = ray.origin.z + y * ray.direction.z / -ray.direction.y;
            collision = fabs(intersection.x - primitive.p0.x) < primitive.size.x &&
                        fabs(intersection.z - primitive.p0.z) < primitive.size.z;
            if (materials[primitive.materialId].attributes.z == 2) // Wireframe
                collision &= wireFrameMapping(intersection.x, intersection.z,
                                              materials[primitive.materialId].attributes.w, primitive);
        }
        if (!collision && reverted * ray.direction.y > 0.f && reverted * ray.origin.y < reverted * primitive.p0.y)
        {
            normal = -normal;
            intersection.x = ray.origin.x + y * ray.direction.x / -ray.direction.y;
            intersection.y = primitive.p0.y;
            intersection.z = ray.origin.z + y * ray.direction.z / -ray.direction.y;
            collision = fabs(intersection.x - primitive.p0.x) < primitive.size.x &&
                        fabs(intersection.z - primitive.p0.z) < primitive.size.z;
            if (materials[primitive.materialId].attributes.z == 2) // Wireframe
                collision &= wireFrameMapping(intersection.x, intersection.z,
                                              materials[primitive.materialId].attributes.w, primitive);
        }
        break;
    }
    case ptYZPlane:
    {
        float x = ray.origin.x - primitive.p0.x;
        if (reverted * ray.direction.x < 0.f && reverted * ray.origin.x > reverted * primitive.p0.x)
        {
            intersection.x = primitive.p0.x;
            intersection.y = ray.origin.y + x * ray.direction.y / -ray.direction.x;
            intersection.z = ray.origin.z + x * ray.direction.z / -ray.direction.x;
            collision = fabs(intersection.y - primitive.p0.y) < primitive.size.y &&
                        fabs(intersection.z - primitive.p0.z) < primitive.size.z;
            if (materials[primitive.materialId].innerIllumination.x != 0.f)
            {
                // Chessboard like Lights
                collision &= int(fabs(intersection.z)) % 4000 < 2000 && int(fabs(intersection.y)) % 4000 < 2000;
            }
            if (materials[primitive.materialId].attributes.z == 2) // Wireframe
                collision &= wireFrameMapping(intersection.y, intersection.z,
                                              materials[primitive.materialId].attributes.w, primitive);
        }
        if (!collision && reverted * ray.direction.x > 0.f && reverted * ray.origin.x < reverted * primitive.p0.x)
        {
            normal = -normal;
            intersection.x = primitive.p0.x;
            intersection.y = ray.origin.y + x * ray.direction.y / -ray.direction.x;
            intersection.z = ray.origin.z + x * ray.direction.z / -ray.direction.x;
            collision = fabs(intersection.y - primitive.p0.y) < primitive.size.y &&
                        fabs(intersection.z - primitive.p0.z) < primitive.size.z;
            if (materials[primitive.materialId].innerIllumination.x != 0.f)
            {
                // Chessboard like Lights
                collision &= int(fabs(intersection.z)) % 4000 < 2000 && int(fabs(intersection.y)) % 4000 < 2000;
            }
            if (materials[primitive.materialId].attributes.z == 2) // Wireframe
                collision &= wireFrameMapping(intersection.y, intersection.z,
                                              materials[primitive.materialId].attributes.w, primitive);
        }
        break;
    }
    case ptXYPlane:
    case ptCamera:
    {
        float z = ray.origin.z - primitive.p0.z;
        if (reverted * ray.direction.z < 0.f && reverted * ray.origin.z > reverted * primitive.p0.z)
        {
            intersection.z = primitive.p0.z;
            intersection.x = ray.origin.x + z * ray.direction.x / -ray.direction.z;
            intersection.y = ray.origin.y + z * ray.direction.y / -ray.direction.z;
            collision = fabs(intersection.x - primitive.p0.x) < primitive.size.x &&
                        fabs(intersection.y - primitive.p0.y) < primitive.size.y;
            if (materials[primitive.materialId].attributes.z == 2) // Wireframe
                collision &= wireFrameMapping(intersection.x, intersection.y,
                                              materials[primitive.materialId].attributes.w, primitive);
        }
        if (!collision && reverted * ray.direction.z > 0.f && reverted * ray.origin.z < reverted * primitive.p0.z)
        {
            normal = -normal;
            intersection.z = primitive.p0.z;
            intersection.x = ray.origin.x + z * ray.direction.x / -ray.direction.z;
            intersection.y = ray.origin.y + z * ray.direction.y / -ray.direction.z;
            collision = fabs(intersection.x - primitive.p0.x) < primitive.size.x &&
                        fabs(intersection.y - primitive.p0.y) < primitive.size.y;
            if (materials[primitive.materialId].attributes.z == 2) // Wireframe
                collision &= wireFrameMapping(intersection.x, intersection.y,
                                              materials[primitive.materialId].attributes.w, primitive);
        }
        break;
    }
    }

    if (collision)
    {
        // Shadow intensity
        shadowIntensity = 1.f; // sceneInfo.shadowIntensity*(1.f-materials[primitive.materialId].transparency);

        vec4f color = materials[primitive.materialId].color;
        if (primitive.type == ptCamera || materials[primitive.materialId].textureIds.x != TEXTURE_NONE)
        {
            vec4f specular = {0.f, 0.f, 0.f}; // TODO?
            vec4f attributes;
            vec4f advancedAttributes;
            color = cubeMapping(sceneInfo, primitive, materials, textures, intersection, normal, specular, attributes,
                                advancedAttributes);
            shadowIntensity = color.w;
        }

        if ((color.x + color.y + color.z) / 3.f >= sceneInfo.transparentColor)
        {
            collision = false;
        }
    }
    return collision;
}

/*
______80__________________________________________________________________________

Triangle intersection
________________________________________________________________________________
*/
__device__ __INLINE__ bool triangleIntersection(const SceneInfo &sceneInfo, const Primitive &triangle, const Ray &ray,
                                                vec3f &intersection, vec3f &normal, vec3f &areas,
                                                float &shadowIntensity, const bool &processingShadows)
{
    // Reject rays using the barycentric coordinates of
    // the intersection point with respect to T.
    vec3f E01 = triangle.p1 - triangle.p0;
    vec3f E03 = triangle.p2 - triangle.p0;
    vec3f P = crossProduct(ray.direction, E03);
    float det = dot(E01, P);

    if (fabs(det) < sceneInfo.geometryEpsilon)
        return false;

    vec3f T = ray.origin - triangle.p0;
    float a = dot(T, P) / det;
    if (a < 0.f || a > 1.f)
        return false;

    vec3f Q = crossProduct(T, E01);
    float b = dot(ray.direction, Q) / det;
    if (b < 0.f || b > 1.f)
        return false;

    // Reject rays using the barycentric coordinates of
    // the intersection point with respect to T′.
    if ((a + b) > 1.f)
    {
        vec3f E23 = triangle.p0 - triangle.p1;
        vec3f E21 = triangle.p1 - triangle.p1;
        vec3f P_ = crossProduct(ray.direction, E21);
        float det_ = dot(E23, P_);
        if (fabs(det_) < sceneInfo.geometryEpsilon)
            return false;
        vec3f T_ = ray.origin - triangle.p2;
        float a_ = dot(T_, P_) / det_;
        if (a_ < 0.f)
            return false;
        vec3f Q_ = crossProduct(T_, E23);
        float b_ = dot(ray.direction, Q_) / det_;
        if (b_ < 0.f)
            return false;
    }

    // Compute the ray parameter of the intersection
    // point.
    float t = dot(E03, Q) / det;
    if (t < 0)
        return false;

    // Intersection
    intersection = ray.origin + t * ray.direction;

    // Normal
    vec3f v0 = (triangle.p0 - intersection);
    vec3f v1 = (triangle.p1 - intersection);
    vec3f v2 = (triangle.p2 - intersection);
    areas.x = 0.5f * length(crossProduct(v1, v2));
    areas.y = 0.5f * length(crossProduct(v0, v2));
    areas.z = 0.5f * length(crossProduct(v0, v1));
    normal = normalize((triangle.n0 * areas.x + triangle.n1 * areas.y + triangle.n2 * areas.z) /
                       (areas.x + areas.y + areas.z));

    if (sceneInfo.doubleSidedTriangles)
    {
        // Double Sided triangles
        // Reject triangles with normal opposite to ray.
        vec3f N = normalize(ray.direction);
        if (processingShadows)
            if (dot(N, normal) <= 0.f)
                return false;
            else if (dot(N, normal) >= 0.f)
                return false;
    }

    vec3f dir = normalize(ray.direction);
    float r = dot(dir, normal);

    if (r > 0.f)
        normal *= -1.f;

    // Shadow management
    shadowIntensity = 1.f;
    return true;
}

/*
________________________________________________________________________________

Intersections with primitives
________________________________________________________________________________
*/
__device__ __INLINE__ bool intersectionWithPrimitives(
    const SceneInfo &sceneInfo, const PostProcessingInfo &postProcessingInfo, BoundingBox *boundingBoxes,
    const int &nbActiveBoxes, Primitive *primitives, const int &nbActivePrimitives, Material *materials,
    BitmapBuffer *textures, const Ray &ray, const int &iteration, int &closestPrimitive, vec3f &closestIntersection,
    vec3f &closestNormal, vec3f &closestAreas, vec4f &closestColor, vec4f &colorBox, const int currentmaterialId)
{
    bool intersections = false;
    float minDistance = (iteration < 2) ? sceneInfo.viewDistance : sceneInfo.viewDistance / (iteration + 1);

    Ray r;
    r.origin = ray.origin;
    r.direction = ray.direction - ray.origin;
    computeRayAttributes(r);

    vec3f intersection = {0.f, 0.f, 0.f};
    vec3f normal = {0.f, 0.f, 0.f};
    bool i = false;
    float shadowIntensity = 0.f;

    int cptBoxes = 0;
    while (cptBoxes < nbActiveBoxes)
    {
        BoundingBox &box = boundingBoxes[cptBoxes];
        if (boxIntersection(box, r, 0.f, minDistance))
        {
            // Intersection with Box
            if (sceneInfo.renderBoxes != 0) // Box 0 is for light emitting objects
            {
                colorBox += materials[box.startIndex % NB_MAX_MATERIALS].color / 50.f;
            }
            else
            {
                // Intersection with primitive within boxes
                for (int cptPrimitives = 0; cptPrimitives < box.nbPrimitives; ++cptPrimitives)
                {
                    Primitive &primitive = primitives[box.startIndex + cptPrimitives];
                    Material &material = materials[primitive.materialId];
                    if (material.attributes.x == 0 ||
                        (material.attributes.x == 1 && currentmaterialId != primitive.materialId)) // !!!! TEST SHALL BE
                                                                                                   // REMOVED TO
                                                                                                   // INCREASE
                                                                                                   // TRANSPARENCY
                                                                                                   // QUALITY !!!
                    {
                        vec3f areas = {0.f, 0.f, 0.f};
                        if (sceneInfo.extendedGeometry)
                        {
                            i = false;
                            switch (primitive.type)
                            {
                            case ptEnvironment:
                            case ptSphere:
                                i = sphereIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                       shadowIntensity);
                                break;
                            case ptCylinder:
                                i = cylinderIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                         shadowIntensity);
                                break;
                            case ptCone:
                                i = coneIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                     shadowIntensity);
                                break;
                            case ptEllipsoid:
                                i = ellipsoidIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                          shadowIntensity);
                                break;
                            case ptTriangle:
                                i = triangleIntersection(sceneInfo, primitive, r, intersection, normal, areas,
                                                         shadowIntensity, false);
                                break;
                            default:
                                i = planeIntersection(sceneInfo, primitive, materials, textures, r, intersection,
                                                      normal, shadowIntensity, false);
                            }
                        }
                        else
                        {
                            i = triangleIntersection(sceneInfo, primitive, r, intersection, normal, areas,
                                                     shadowIntensity, false);
                        }

                        float distance = length(intersection - r.origin);

                        if (i && distance > sceneInfo.geometryEpsilon && distance < minDistance)
                        {
                            // Only keep intersection with the closest object
                            minDistance = distance;
                            closestPrimitive = box.startIndex + cptPrimitives;
                            closestIntersection = intersection;
                            closestNormal = normal;
                            closestAreas = areas;
                            intersections = true;
                        }
                    }
                }
            }
            ++cptBoxes;
        }
        else
        {
            cptBoxes += box.indexForNextBox.x;
        }
    }
    return intersections;
}

/*
________________________________________________________________________________

Shadows computation
We do not consider the object from which the ray is launched...
This object cannot shadow itself !

We now have to find the intersection between the considered object and the ray
which origin is the considered 3D vec4f and which direction is defined by the
light source center.
.
. * Lamp                     Ray = Origin -> Light Source Center
.  \
.   \##
.   #### object
.    ##
.      \
.       \  Origin
.--------O-------
.
@return 1.f when pixel is in the shades

________________________________________________________________________________
*/
__device__ __INLINE__ float processShadows(const SceneInfo &sceneInfo, BoundingBox *boudingBoxes,
                                           const int &nbActiveBoxes, Primitive *primitives, Material *materials,
                                           BitmapBuffer *textures, const int &nbPrimitives, const vec3f &lampCenter,
                                           const vec3f &origin, const int &lightId, const int &iteration, vec4f &color,
                                           const int &objectId)
{
    float result = 0.f;
    int cptBoxes = 0;
    color.x = 0.f;
    color.y = 0.f;
    color.z = 0.f;
    Ray r;
    r.direction = lampCenter - origin;
    r.origin = origin + normalize(r.direction) * sceneInfo.rayEpsilon;
    computeRayAttributes(r);
    const float minDistance = (iteration < 2) ? sceneInfo.viewDistance : sceneInfo.viewDistance / (iteration + 1);

    while (result < (sceneInfo.shadowIntensity) && cptBoxes < nbActiveBoxes)
    {
        BoundingBox &box = boudingBoxes[cptBoxes];
        if (boxIntersection(box, r, 0.f, minDistance))
        {
            int cptPrimitives = 0;
            while (result < sceneInfo.shadowIntensity && cptPrimitives < box.nbPrimitives)
            {
                vec3f intersection = {0.f, 0.f, 0.f};
                vec3f normal = {0.f, 0.f, 0.f};
                vec3f areas = {0.f, 0.f, 0.f};
                float shadowIntensity = 0.f;

                Primitive &primitive = primitives[box.startIndex + cptPrimitives];
                if (primitive.index != lightId && primitive.index != objectId &&
                    materials[primitive.materialId].attributes.x == 0)
                {
                    bool hit = false;
                    if (sceneInfo.extendedGeometry)
                    {
                        switch (primitive.type)
                        {
                        case ptSphere:
                            hit = sphereIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                     shadowIntensity);
                            break;
                        case ptEllipsoid:
                            hit = ellipsoidIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                        shadowIntensity);
                            break;
                        case ptCylinder:
                            hit = cylinderIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                       shadowIntensity);
                            break;
                        case ptCone:
                            hit = coneIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                   shadowIntensity);
                            break;
                        case ptTriangle:
                            hit = triangleIntersection(sceneInfo, primitive, r, intersection, normal, areas,
                                                       shadowIntensity, true);
                            break;
                        case ptCamera:
                            hit = false;
                            break;
                        default:
                            hit = planeIntersection(sceneInfo, primitive, materials, textures, r, intersection, normal,
                                                    shadowIntensity, false);
                            break;
                        }
                    }
                    else
                    {
                        hit = triangleIntersection(sceneInfo, primitive, r, intersection, normal, areas,
                                                   shadowIntensity, true);
                    }

                    if (hit)
                    {
                        vec3f O_I = intersection - r.origin;
                        vec3f O_L = r.direction;
                        float l = length(O_I);
                        if (l > sceneInfo.geometryEpsilon && l < length(O_L))
                        {
                            float ratio = shadowIntensity * sceneInfo.shadowIntensity;
                            if (materials[primitive.materialId].transparency != 0.f)
                            {
                                // Shadow color
                                O_L = normalize(O_L);
                                float a = fabs(dot(O_L, normal));
                                float r = (materials[primitive.materialId].transparency == 0.f)
                                              ? 1.f
                                              : (1.f - materials[primitive.materialId].transparency);
                                ratio *= r * a;
                                color.x += ratio * (0.3f - 0.3f * materials[primitive.materialId].color.x);
                                color.y += ratio * (0.3f - 0.3f * materials[primitive.materialId].color.y);
                                color.z += ratio * (0.3f - 0.3f * materials[primitive.materialId].color.z);
                            }
                            result += ratio;
                        }
                    }
                }
                ++cptPrimitives;
            }
            ++cptBoxes;
        }
        else
        {
            cptBoxes += box.indexForNextBox.x;
        }
    }
    result = max(0.f, min(result, sceneInfo.shadowIntensity));
    return result;
}

/*
________________________________________________________________________________

Primitive shader
________________________________________________________________________________
*/
__device__ __INLINE__ vec4f primitiveShader(
    const int &index, const SceneInfo &sceneInfo, const PostProcessingInfo &postProcessingInfo,
    BoundingBox *boundingBoxes, const int &nbActiveBoxes, Primitive *primitives, const int &nbActivePrimitives,
    LightInformation *lightInformation, const int &lightInformationSize, const int &nbActiveLamps, Material *materials,
    BitmapBuffer *textures, RandomBuffer *randoms, const vec3f &origin, vec3f &normal, const int &objectId,
    vec3f &intersection, const vec3f &areas, vec4f &closestColor, const int &iteration, vec4f &refractionFromColor,
    float &shadowIntensity, vec4f &totalBlinn, vec4f &attributes)
{
    Primitive &primitive = primitives[objectId];
    Material &material = materials[primitive.materialId];
    vec4f lampsColor = {0.f, 0.f, 0.f, 0.f};

    // Lamp Impact
    shadowIntensity = 0.f;

    // Bump
    vec3f bumpNormal = {0.f, 0.f, 0.f};
    vec4f advancedAttributes = {0.f, 0.f, 0.f, 0.f};

    // Specular
    vec4f specular;
    specular.x = material.specular.x;
    specular.y = material.specular.y;
    specular.z = material.specular.z;

    // Intersection color
    vec4f intersectionColor = intersectionShader(sceneInfo, primitive, materials, textures, intersection, areas,
                                                 bumpNormal, specular, attributes, advancedAttributes);
    normal += bumpNormal;
    normal = normalize(normal);

    if (/*material.innerIllumination.x!=0.f || */ material.attributes.z == 1)
    {
        // Wireframe returns constant color
        return intersectionColor;
    }

    if (sceneInfo.graphicsLevel > glNoShading)
    {
        closestColor *= material.innerIllumination.x;
        for (int cpt = 0; cpt < lightInformationSize; ++cpt)
        {
            int cptLamp = (sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
                              ? (sceneInfo.pathTracingIteration % lightInformationSize)
                              : 0;
            if (lightInformation[cptLamp].primitiveId != primitive.index)
            {
                vec3f center;
                // randomize lamp center
                center = lightInformation[cptLamp].location;

                int t = (index + sceneInfo.timestamp) % (MAX_BITMAP_SIZE - 3);
                Material &m = materials[lightInformation[cptLamp].materialId];
                if (sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
                {
                    float a = m.innerIllumination.y * 10.f * sceneInfo.pathTracingIteration /
                              sceneInfo.maxPathTracingIterations;
                    center.x += randoms[t] * a;
                    center.y += randoms[t + 1] * a;
                    center.z += randoms[t + 2] * a;
                }

                vec3f lightRay = center - intersection;
                float lightRayLength = length(lightRay);
                if (lightRayLength < m.innerIllumination.z)
                {
                    vec4f shadowColor = {0.f, 0.f, 0.f, 0.f};
                    // --------------------------------------------------------------------------------
                    // Lambert
                    // --------------------------------------------------------------------------------
                    lightRay = normalize(lightRay);
                    float lambert = material.innerIllumination.x + dot(normal, lightRay);

                    if (lambert > 0.f && sceneInfo.graphicsLevel > 3 &&
                        iteration < 4 && // No need to process shadows after 4 generations
                                         // of rays... cannot be seen anyway.
                        material.innerIllumination.x == 0.f)
                    {
                        shadowIntensity =
                            processShadows(sceneInfo, boundingBoxes, nbActiveBoxes, primitives, materials, textures,
                                           nbActivePrimitives, center, intersection,
                                           lightInformation[cptLamp].primitiveId, iteration, shadowColor, objectId);
                    }

                    if (sceneInfo.graphicsLevel > glNoShading)
                    {
                        float photonEnergy = sqrt(lightRayLength / m.innerIllumination.z);
                        photonEnergy = (photonEnergy > 1.f) ? 1.f : photonEnergy;
                        photonEnergy = (photonEnergy < 0.f) ? 0.f : photonEnergy;

                        // Transparent materials are lighted on both sides but the amount
                        // of light received by the "dark side"
                        // depends on the transparency rate.
                        lambert *= (lambert < 0.f) ? -materials[primitive.materialId].transparency : 1.f;

                        if (lightInformation[cptLamp].materialId != MATERIAL_NONE)
                        {
                            Material &m = materials[lightInformation[cptLamp].materialId];
                            lambert *= m.innerIllumination.x; // Lamp illumination
                        }
                        else
                            lambert *= lightInformation[cptLamp].color.w;

                        if (material.innerIllumination.w != 0.f)
                            // Randomize lamp intensity depending on material noise, for
                            // more realistic rendering
                            lambert *= (1.f + randoms[t] * material.innerIllumination.w * 100.f);

                        lambert *= (1.f - shadowIntensity);
                        lambert += sceneInfo.backgroundColor.w;
                        lambert *= (1.f - photonEnergy);

                        // Lighted object, not in the shades
                        lampsColor += lambert * lightInformation[cptLamp].color - shadowColor;

                        if (sceneInfo.graphicsLevel > 1 && shadowIntensity < sceneInfo.shadowIntensity)
                        {
                            // --------------------------------------------------------------------------------
                            // Blinn - Phong
                            // --------------------------------------------------------------------------------
                            vec3f viewRay = normalize(intersection - origin);
                            vec3f blinnDir = lightRay - viewRay;
                            float temp = sqrt(dot(blinnDir, blinnDir));
                            if (temp != 0.f)
                            {
                                // Specular reflection
                                blinnDir = (1.f / temp) * blinnDir;
                                float blinnTerm = dot(blinnDir, normal);
                                blinnTerm = (blinnTerm < 0.f) ? 0.f : blinnTerm;

                                blinnTerm = specular.x * pow(blinnTerm, specular.y);
                                blinnTerm *= (1.f - photonEnergy);
                                totalBlinn +=
                                    lightInformation[cptLamp].color * lightInformation[cptLamp].color.w * blinnTerm;

                                // Get transparency from specular map
                                totalBlinn.w = specular.z;
                            }
                        }
                    }
                }
            }

            // Light impact on material
            closestColor += intersectionColor * lampsColor;

            // Ambient occlusion
            if (material.advancedTextureIds.z != TEXTURE_NONE)
            {
                closestColor *= advancedAttributes.x;
            }

            // Saturate color
            saturateVector(closestColor);

            refractionFromColor = intersectionColor; // Refraction depending on color;
            saturateVector(totalBlinn);
        }
    }
    else
    {
        closestColor = intersectionColor;
    }
    return closestColor;
}

/*
________________________________________________________________________________

Intersections with primitives
________________________________________________________________________________
*/
__device__ __INLINE__ vec4f intersectionsWithPrimitives(
    const int &index, const SceneInfo &sceneInfo, BoundingBox *boundingBoxes, const int &nbActiveBoxes,
    Primitive *primitives, const int &nbActivePrimitives, Material *materials, BitmapBuffer *textures,
    LightInformation *lightInformation, const int &lightInformationSize, const int &nbActiveLamps,
    RandomBuffer *randoms, const PostProcessingInfo &postProcessingInfo, const Ray &ray)
{
    Ray r;
    r.origin = ray.origin;
    r.direction = ray.direction - ray.origin;
    computeRayAttributes(r);

    vec3f intersection = ray.origin;
    vec3f normal = {0.f, 0.f, 0.f};
    bool i = false;
    float shadowIntensity = 0.f;

    const int MAXDEPTH = 10;
    vec4f colors[MAXDEPTH];
    for (int i(0); i < MAXDEPTH; ++i)
    {
        colors[i].x = 0.f;
        colors[i].y = 0.f;
        colors[i].z = 0.f;
        colors[i].w = sceneInfo.viewDistance;
    }
#ifdef VOLUME_RENDERING_NORMALS
    int normals[MAXDEPTH];
    memset(&normals[0], 0, sizeof(bool) * MAXDEPTH);
#endif // VOLUME_RENDERING_NORMALS

    int nbIntersections = 0;
    int cptBoxes = 0;
    while (cptBoxes < nbActiveBoxes)
    {
        BoundingBox &box = boundingBoxes[cptBoxes];
        if (boxIntersection(box, r, 0.f, sceneInfo.viewDistance))
        {
            // Intersection with primitive within boxes
            for (int cptPrimitives = 0; cptPrimitives < box.nbPrimitives; ++cptPrimitives)
            {
                i = false;
                Primitive &primitive = primitives[box.startIndex + cptPrimitives];
                Material &material = materials[primitive.materialId];
                vec3f areas = {0.f, 0.f, 0.f};
                if (sceneInfo.extendedGeometry)
                {
                    switch (primitive.type)
                    {
                    case ptEnvironment:
                    case ptSphere:
                        i = sphereIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                               shadowIntensity);
                        break;
                    case ptCylinder:
                        i = cylinderIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                 shadowIntensity);
                        break;
                    case ptCone:
                        i = coneIntersection(sceneInfo, primitive, materials, r, intersection, normal, shadowIntensity);
                        break;
                    case ptEllipsoid:
                        i = ellipsoidIntersection(sceneInfo, primitive, materials, r, intersection, normal,
                                                  shadowIntensity);
                        break;
                    case ptTriangle:
                        i = triangleIntersection(sceneInfo, primitive, r, intersection, normal, areas, shadowIntensity,
                                                 false);
                        break;
                    default:
                        i = planeIntersection(sceneInfo, primitive, materials, textures, r, intersection, normal,
                                              shadowIntensity, false);
                        break;
                    }
                }
                else
                {
                    i = triangleIntersection(sceneInfo, primitive, r, intersection, normal, areas, shadowIntensity,
                                             false);
                }
                if (i)
                {
                    float dist = length(intersection - r.origin);
                    if (dist > postProcessingInfo.param1)
                    {
                        ++nbIntersections;
                        vec4f color = material.color;
                        if (sceneInfo.graphicsLevel != glNoShading)
                        {
                            color *= (1.f - material.transparency);
                            vec4f attributes;
                            attributes.x = material.reflection;
                            attributes.y = material.transparency;
                            attributes.z = material.refraction;
                            attributes.w = material.opacity;
                            vec4f rBlinn = {0.f, 0.f, 0.f, 0.f};
                            vec4f refractionFromColor;
                            vec4f closestColor = material.color;
                            shadowIntensity = 0.f;
                            color =
                                primitiveShader(index, sceneInfo, postProcessingInfo, boundingBoxes, nbActiveBoxes,
                                                primitives, nbActivePrimitives, lightInformation, lightInformationSize,
                                                nbActiveLamps, materials, textures, randoms, r.origin, normal,
                                                box.startIndex + cptPrimitives, intersection, areas, closestColor, 0,
                                                refractionFromColor, shadowIntensity, rBlinn, attributes);
                        }
                        for (int i(0); i < MAXDEPTH; ++i)
                        {
                            if (dist < colors[i].w)
                            {
                                float a = dot(normalize(ray.direction - ray.origin), normal);
                                for (int j(MAXDEPTH - 1); j >= i; --j)
                                {
                                    colors[j + 1] = colors[j];
#ifdef VOLUME_RENDERING_NORMALS
                                    normals[j + 1] = normals[j];
#endif // VOLUME_RENDERING_NORMALS
                                }
                                colors[i].x = color.x * fabs(a);
                                colors[i].y = color.y * fabs(a);
                                colors[i].z = color.z * fabs(a);
                                colors[i].w = dist;
#ifdef VOLUME_RENDERING_NORMALS
                                normals[i] = (a < 0.f) ? -1 : 1;
#endif // VOLUME_RENDERING_NORMALS
                                break;
                            }
                        }
                    }
                }
            }
            ++cptBoxes;
        }
        else
        {
            cptBoxes += box.indexForNextBox.x;
        }
    }

    vec4f color = colors[0] * sceneInfo.backgroundColor.w;
    if (nbIntersections > 0)
    {
        int N = 0;
        float D = colors[0].w;
        const int precision = 500;
        const float step = sceneInfo.viewDistance / float(precision);
        float alpha = 1.f / postProcessingInfo.param2;
        int c = 0;
        for (int i(0); i < precision && c < MAXDEPTH - 1; ++i)
        {
            if (D > colors[c].w && N == 0)
            {
                color.x += colors[c].x * alpha;
                color.y += colors[c].y * alpha;
                color.z += colors[c].z * alpha;
            }
            D += step;
            if (D >= colors[c + 1].w)
            {
                ++c;
#ifdef VOLUME_RENDERING_NORMALS
                N += normals[c];
#endif // VOLUME_RENDERING_NORMALS
            }
        }
        color.w = 0.f;
        normalize(color);
        /*
        if(length(color)<sceneInfo.backgroundColor.w)
        {
        color.x=1.f;
        color.y=1.f;
        color.z=0.f;
        }
        */
    }
    color.w = colors[0].w;
    return color;
}
