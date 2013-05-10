/* 
* Copyright (C) 2011-2012 Cyrille Favreau <cyrille_favreau@hotmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Library General Public
* License as published by the Free Software Foundation; either
* version 2 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Library General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

/*
* Author: Cyrille Favreau <cyrille_favreau@hotmail.com>
*
*/

#pragma OPENCL EXTENSION cl_khr_fp64 : enable

// Globals
#define gNbIterations 20
#define EPSILON 1.f

// 3D vision type
enum VisionType
{
   vtStandard = 0,
   vtAnaglyph = 1,
   vt3DVision = 2
};

// Scene information
typedef struct 
{
   int     width;
   int     height;
   int     shadowsEnabled;
   int     nbRayIterations;
   float   transparentColor;
   float   viewDistance;
   float   shadowIntensity;
   float   width3DVision;
   float4  backgroundColor;
   int     supportFor3DVision;
   int     renderBoxes;
   int     pathTracingIteration;
   int     maxPathTracingIterations;
   int4    misc;
} SceneInfo; 

typedef struct 
{
   float4 origin;
   float4 direction;
   float4 inv_direction;
   int4   signs;
} Ray;

// Enums
enum PrimitiveType 
{
	ptSphere      = 0,
	ptCylinder    = 1,
	ptTriangle    = 2,
	ptCheckboard  = 3,
	ptCamera      = 4,
	ptXYPlane     = 5,
	ptYZPlane     = 6,
	ptXZPlane     = 7,
   ptMagicCarpet = 8,
   ptEnvironment = 9
};

// TODO! Data structure is too big!!!
typedef struct 
{
   float4 innerIllumination;
	float4 color;
   float4 specular;       // x: value, y: power, w: coef, z: inner illumination
   float4 reflection;     
	float4 refraction;
   float4 transparency;
	int4   textureInfo;
} Material;

typedef struct 
{
   float4 parameters[2];
   int   nbPrimitives;
   int   startIndex;
} BoundingBox;

typedef struct 
{
	float4 center;
	float4 otherCenter;
	float4 axis;
	float4 normal;
	float4 size;
	int    type;
	int    materialId;
	float2 materialInfo;
} Primitive;

// Post processing effect
enum PostProcessingType 
{
   ppe_none,
   ppe_depthOfField,
   ppe_ambientOcclusion,
   ppe_cartoon,
   ppe_antiAliasing
};

typedef struct
{
   int   type;
   float param1; // pointOfFocus;
   float param2; // strength;
   int   param3; // iterations;
} PostProcessingInfo;

__constant int NB_MAX_BOXES      = 18*18*18;
__constant int NB_MAX_PRIMITIVES = 100000;
__constant int NB_MAX_LAMPS      = 10;
__constant int NB_MAX_MATERIALS  = 100;
__constant int NB_MAX_TEXTURES   = 50;

// Constants
__constant int NO_MATERIAL = -1;
__constant int NO_TEXTURE  = -1;
__constant int gColorDepth = 3;

// Textures
__constant int gTextureWidth = 256;
__constant int gTextureHeight= 256;
__constant int gTextureDepth = 3;

// Kinect
/*
#define gKinectVideoWidth  640
#define gKinectVideoHeight 480
#define gKinectVideo       4

#define gKinectDepthWidth  320
#define gKinectDepthHeight 240
#define gKinectDepth       2

 __constant char*        d_kinectVideo;
 __constant char*        d_kinectDepth;
 */

// ________________________________________________________________________________
float vectorLength( float4 vector )
{
   return sqrt( vector.x*vector.x + vector.y*vector.y + vector.z*vector.z );
}

// ________________________________________________________________________________
inline void normalizeVector( float4* v )
{
   (*v) /= vectorLength(*v);
}

// ________________________________________________________________________________
void saturateVector( float4* v )
{
   v->x = (v->x<0.f) ? 0.f : v->x;
   v->y = (v->y<0.f) ? 0.f : v->y; 
   v->z = (v->z<0.f) ? 0.f : v->z;
   v->w = (v->w<0.f) ? 0.f : v->w;

   v->x = (v->x>1.f) ? 1.f : v->x;
   v->y = (v->y>1.f) ? 1.f : v->y; 
   v->z = (v->z>1.f) ? 1.f : v->z;
   v->w = (v->w>1.f) ? 1.f : v->w;
}

// ________________________________________________________________________________
#define dotProduct( v1, v2 )\
	( v1.x*v2.x + v1.y*v2.y + v1.z*v2.z)

// ________________________________________________________________________________
float4 crossProduct( float4* b, float4* c )
{
   float4 a;
   a.x = b->y*c->z - b->z*c->y;
   a.y = b->z*c->x - b->x*c->z;
   a.z = b->x*c->y - b->y*c->x;
   return a;
}

/*
________________________________________________________________________________
incident  : le vecteur normal inverse a la direction d'incidence de la source 
lumineuse
normal    : la normale a l'interface orientee dans le materiau ou se propage le 
rayon incident
reflected : le vecteur normal reflechi
________________________________________________________________________________
*/
#define vectorReflection( __r, __i, __n ) \
	__r = __i-2.f*dotProduct(__i,__n)*__n;
	
/*
________________________________________________________________________________
incident: le vecteur norm? inverse ? la direction d?incidence de la source 
lumineuse
n1      : index of refraction of original medium
n2      : index of refraction of new medium
________________________________________________________________________________
*/
void vectorRefraction( 
   float4*      refracted, 
   const float4 incident, 
   const float  n1, 
   const float4 normal, 
   const float  n2 )
{
   (*refracted) = incident;
   if(n1!=n2 && n2!=0.f) 
   {
      float r = n1/n2;
      float cosI = dotProduct( incident, normal );
      float cosT2 = 1.f - r*r*(1.f - cosI*cosI);
      (*refracted) = r*incident + (r*cosI-sqrt( fabs(cosT2) ))*normal;
   }
}

/*
________________________________________________________________________________
__v : Vector to rotate
__c : Center of rotations
__a : Angles
________________________________________________________________________________
*/
#define vectorRotation( __v, __c, __a ) \
{ \
	float4 __r = __v; \
	/* X axis */ \
	__r.y = __v.y*half_cos(angles.x) - __v.z*half_sin(__a.x); \
	__r.z = __v.y*half_sin(angles.x) + __v.z*half_cos(__a.x); \
	__v = __r; \
	__r = __v; \
	/* Y axis */ \
	__r.z = __v.z*half_cos(__a.y) - __v.x*half_sin(__a.y); \
	__r.x = __v.z*half_sin(__a.y) + __v.x*half_cos(__a.y); \
	__v = __r; \
}

/*
________________________________________________________________________________

Compute ray attributes
________________________________________________________________________________
*/
void computeRayAttributes(Ray* ray)
{
   ray->inv_direction.x = 1.f/ray->direction.x;
   ray->inv_direction.y = 1.f/ray->direction.y;
   ray->inv_direction.z = 1.f/ray->direction.z;
   ray->signs.x = (ray->inv_direction.x < 0);
   ray->signs.y = (ray->inv_direction.y < 0);
   ray->signs.z = (ray->inv_direction.z < 0);
}

/*
________________________________________________________________________________

Convert float4 into Delphi RGB color
________________________________________________________________________________
*/
void makeDelphiColor( 
   float4 color, 
   __global char*  bitmap, 
   int    index)
{
   int mdc_index = index*3; 
   color.x = (color.x>1.f) ? 1.f : color.x;
   color.y = (color.y>1.f) ? 1.f : color.y; 
   color.z = (color.z>1.f) ? 1.f : color.z;
   bitmap[mdc_index  ] = (char)(color.z*255.f);
   bitmap[mdc_index+1] = (char)(color.y*255.f);
   bitmap[mdc_index+2] = (char)(color.x*255.f);
}

/*
________________________________________________________________________________

Convert float4 into OpenGL RGB color
________________________________________________________________________________
*/
void makeColor( 
   int             type,
	float4         color, 
	__global char* bitmap, 
	int            index)
{
   int mdc_index = index*gColorDepth; 
   color.x = (color.x>1.f) ? 1.f : color.x;
   color.y = (color.y>1.f) ? 1.f : color.y; 
   color.z = (color.z>1.f) ? 1.f : color.z;

   bitmap[mdc_index+1] = (char)(color.y*255.f); // Green
   if( type == 0 )
   {
      // OpenGL
      bitmap[mdc_index  ] = (char)(color.x*255.f); // Red
      bitmap[mdc_index+2] = (char)(color.z*255.f); // Blue
   }
   else
   {
      // Delphi
      bitmap[mdc_index  ] = (char)(color.z*255.f);
      bitmap[mdc_index+2] = (char)(color.x*255.f);
   }
}

/*
________________________________________________________________________________

Sphere texture Mapping
________________________________________________________________________________
*/
 float4 sphereUVMapping( 
   Primitive* primitive,
   __global  Material*  materials,
   __global  char*      textures,
   float4    intersection,
   float     timer)
{
   float4 result = materials[primitive->materialId].color;
   float4 d = primitive->center-intersection;
   normalizeVector(&d);
   int u = primitive->size.x / primitive->materialInfo.x * (0.5f - atan2(d.z, d.x) / 2*M_PI);
   int v = primitive->size.y / primitive->materialInfo.y * (0.5f - 2.f*(asin(d.y) / 2*M_PI));

   u = u % gTextureWidth;
   v = v % gTextureHeight;
   if( u>=0 && u<gTextureWidth && v>=0 && v<gTextureHeight )
   {
      int index = (materials[primitive->materialId].textureInfo.y*gTextureWidth*gTextureHeight + v*gTextureWidth+u)*gTextureDepth;
      unsigned char r = textures[index  ];
      unsigned char g = textures[index+1];
      unsigned char b = textures[index+2];
      result.x = r/256.f;
      result.y = g/256.f;
      result.z = b/256.f;
   }
   return result; 
}

/*
________________________________________________________________________________

Cube texture mapping
________________________________________________________________________________
*/
 float4 cubeMapping( 
   Primitive* primitive, 
   __global Material*  materials,
   __global char*      textures,
   float4 intersection)
{
   float4 result = materials[primitive->materialId].color;
   int x = ((primitive->type == ptCheckboard) || (primitive->type == ptXZPlane) || (primitive->type == ptXYPlane))  ? 
      (intersection.x-primitive->center.x+primitive->size.x)*primitive->materialInfo.x :
      (intersection.z-primitive->center.z+primitive->size.x)*primitive->materialInfo.x;

   int y = ((primitive->type == ptCheckboard) || (primitive->type == ptXZPlane)) ? 
      (intersection.z+primitive->center.z+primitive->size.z)*primitive->materialInfo.y :
	  (intersection.y-primitive->center.y+primitive->size.z)*primitive->materialInfo.y;

   x = x%gTextureWidth;
   y = y%gTextureHeight;

   if( x>=0 && x<gTextureWidth && y>=0 && y<gTextureHeight )
   {
      int index = (materials[primitive->materialId].textureInfo.y*gTextureWidth*gTextureHeight + y*gTextureWidth+x)*gTextureDepth;
      unsigned char r = textures[index];
      unsigned char g = textures[index+1];
      unsigned char b = textures[index+2];
      result.x = r/256.f;
      result.y = g/256.f;
      result.z = b/256.f;
   }
   return result;
}

/*
________________________________________________________________________________

Box intersection
________________________________________________________________________________
*/
bool boxIntersection( 
   BoundingBox box, 
   Ray*        ray,
   float       t0,
   float       t1)
{
   float tmin, tmax, tymin, tymax, tzmin, tzmax;

   tmin = (box.parameters[ray->signs.x].x - ray->origin.x) * ray->inv_direction.x;
   tmax = (box.parameters[1-ray->signs.x].x - ray->origin.x) * ray->inv_direction.x;
   tymin = (box.parameters[ray->signs.y].y - ray->origin.y) * ray->inv_direction.y;
   tymax = (box.parameters[1-ray->signs.y].y - ray->origin.y) * ray->inv_direction.y;

   if ( (tmin > tymax) || (tymin > tmax) ) 
      return false;

   if (tymin > tmin) tmin = tymin;
   if (tymax < tmax) tmax = tymax;
   tzmin = (box.parameters[ray->signs.z].z - ray->origin.z) * ray->inv_direction.z;
   tzmax = (box.parameters[1-ray->signs.z].z - ray->origin.z) * ray->inv_direction.z;

   if ( (tmin > tzmax) || (tzmin > tmax) ) 
      return false;

   if (tzmin > tmin) tmin = tzmin;
   if (tzmax < tmax) tmax = tzmax;
   return ( (tmin < t1) && (tmax > t0) );
}

/*
________________________________________________________________________________

Sphere (*intersection)
________________________________________________________________________________
*/
bool sphereIntersection(
   SceneInfo* sceneInfo,
   Primitive* sphere, 
   __global Material*  materials, 
   __global char*      textures, 
   Ray*       ray, 
   float      timer,
   float4*    intersection,
   float4*    normal,
   float*     shadowIntensity,
   bool*      back
   ) 
{
   // solve the equation sphere-(*ray) to find the (*intersection)s
   float4 O_C = ray->origin - (*sphere).center;
   normalizeVector(&ray->direction);

   float a = 2.f*dotProduct(ray->direction,ray->direction);
   float b = 2.f*dotProduct(O_C,ray->direction);
   float c = dotProduct(O_C,O_C) - ((*sphere).size.x*(*sphere).size.x);
   float d = b*b-2.f*a*c;
   if( d>0.f && a != 0.f) 
   {
      float epsilon = 0.9f;

      float r = sqrt(d);
      float t1 = (-b-r)/a;
      float t2 = (-b+r)/a;

      if( t1<=epsilon && t2<=epsilon ) return false; // both (*intersection)s are behind the ray origin

      (*back) = (t1<=epsilon || t2<=epsilon); // If only one (*intersection) (t>0) then we are inside the sphere and the (*intersection) is at the (*back) of the sphere

      float t=0.f;
      if( t1<=epsilon ) 
         t = t2;
      else 
         if( t2<=epsilon )
            t = t1;
         else
            t=(t1<t2) ? t1 : t2;
      
      if( t<=epsilon ) return false; // Too close to (*intersection)

      (*intersection) = ray->origin+t*ray->direction; 
      
      // Compute (*normal) vector
      (*normal) = (*intersection)-(*sphere).center;
      (*normal).w = 0.f;
      (*normal) *= ((*back)) ? -1.f : 1.f;
      normalizeVector(normal);

      // Shadow intensity
      (*shadowIntensity) = (*sceneInfo).shadowIntensity*(1.f-materials[(*sphere).materialId].transparency.x);

#if 0
      if( materials[(*sphere).materialId].textureInfo.x == 1) 
      {
         // Procedural texture
         float4 newCenter;
         newCenter.x = (*sphere).center.x + 5.f*cos((*intersection).x);
         newCenter.y = (*sphere).center.y + 5.f*sin((*intersection).y);
         newCenter.z = (*sphere).center.z + 5.f*sin(cos((*intersection).z));
         (*normal)  = (*intersection) - newCenter;
      }
      // Power textures
      if (materials[(*sphere).materialId].textureInfo.y != NO_TEXTURE && materials[(*sphere).materialId].transparency.x != 0 ) 
      {
         float4 color = sphereUVMapping(sphere, materials, textures, (*intersection), timer );
         return ((color.x+color.y+color.z) >= (*sceneInfo).transparentColor ); 
      }
#endif // 0

      return true;
   }

#if EXTENDED_FEATURES
   // Soft Shadows
   if( result && computingShadows ) 
   {
      float4 O_R;
      O_R.x = ray->x-origin.x;
      O_R.y = ray->y-origin.y;
      O_R.z = ray->z-origin.z;

      normalizeVector(&O_R);
      (*shadowIntensity) = dotProduct(O_R, (*normal));
      (*shadowIntensity) = ((*shadowIntensity)>1.f) ? 1.f : (*shadowIntensity);
      (*shadowIntensity) = ((*shadowIntensity)<0.f) ? 0.f : (*shadowIntensity);
   } 
#endif // EXTENDED_FEATURES
   return false;
}

/*
________________________________________________________________________________

Cylinder (*intersection)
________________________________________________________________________________
*/
 bool cylinderIntersection( 
   SceneInfo* sceneInfo,
   Primitive* cylinder,
   __global Material* materials, 
   __global char*     textures,
   Ray*      ray,
   float     timer,
   float4*   intersection,
   float4*   normal,
   float*    shadowIntensity,
   bool*     back) 
{
   (*back) = false;
   float4 dir = ray->direction;
   //normalizeVector(&dir); // DO NOT NORMALIZE!!!
   float4 RC = ray->origin-cylinder->center;
   float4 n = crossProduct(&dir, &cylinder->axis);

   float ln = vectorLength(n);

   // Parallel? (?)
   if((ln<EPSILON)&&(ln>-EPSILON))
      return false;

   normalizeVector(&n);

   float d = fabs(dotProduct(RC,n));
   if (d>cylinder->center.w) return false;

   float4 O = crossProduct(&RC,&cylinder->axis);
   float t = -dotProduct(O, n)/ln;
   O = crossProduct(&n,&cylinder->axis);
   normalizeVector(&O);
   float s=fabs( sqrt(cylinder->center.w*cylinder->center.w-d*d) / dotProduct( dir,O ) );

   float in=t-s;
   float out=t+s;

   if (in<-EPSILON)
      if(out<-EPSILON)
         return false;
      else 
      {
         t=out;
         (*back) = true;
      }
   else 
      if(out<-EPSILON)
         t=in;
      else 
         if(in<out)
            t=in;
         else
         {
            t=out;
            (*back) = true;
         }

   if( t<0.f ) return false;

   // Calculate intersection point
   (*intersection) = ray->origin+t*dir;

   float4 HB = (*intersection)-cylinder->center;

   float scale = dotProduct(HB,cylinder->axis);

   // Cylinder length
   if( scale <= 0.f || vectorLength(HB)>cylinder->size.y ) return false;

   (*normal) = HB-cylinder->axis*scale;
   normal->w = 0.f;
   normalizeVector( normal );

   // Shadow intensity
   (*shadowIntensity) = sceneInfo->shadowIntensity*(1.f-materials[cylinder->materialId].transparency.x);
   return true;
}

/*
________________________________________________________________________________

Checkboard (*intersection)
________________________________________________________________________________
*/
 bool planeIntersection( 
   Primitive* primitive,
   __global Material* materials,
   __global char*     textures,
   Ray*      ray, 
   bool      reverse,
   float*    shadowIntensity,
   float4*   intersection,
   float4*   normal,
   float     transparentColor,
   float     timer)
{ 
   bool collision = false;

   float reverted = reverse ? -1.f : 1.f;
   switch( primitive->type ) 
   {
   case ptMagicCarpet:
   case ptCheckboard:
      {
         (*intersection).y = primitive->center.y;
         float y = ray->origin.y-primitive->center.y;
         if( reverted*ray->direction.y<0.f && reverted*ray->origin.y>reverted*primitive->center.y) 
         {
            (*normal).x =  0.f;
            (*normal).y =  1.f;
            (*normal).z =  0.f;
            (*intersection).x = ray->origin.x+y*ray->direction.x/-ray->direction.y;
            (*intersection).z = ray->origin.z+y*ray->direction.z/-ray->direction.y;
            collision = 
               fabs((*intersection).x - primitive->center.x) < primitive->size.x &&
               fabs((*intersection).z - primitive->center.z) < primitive->size.z;
         }
         break;
      }
   case ptXZPlane:
      {
         float y = ray->origin.y-primitive->center.y;
         if( reverted*ray->direction.y<0.f && reverted*ray->origin.y>reverted*primitive->center.y) 
         {
            (*normal).x =  0.f;
            (*normal).y =  1.f;
            (*normal).z =  0.f;
            (*intersection).x = ray->origin.x+y*ray->direction.x/-ray->direction.y;
            (*intersection).y = primitive->center.y;
            (*intersection).z = ray->origin.z+y*ray->direction.z/-ray->direction.y;
            collision = 
               fabs((*intersection).x - primitive->center.x) < primitive->size.x &&
               fabs((*intersection).z - primitive->center.z) < primitive->size.z;
         }
         if( !collision && reverted*ray->direction.y>0.f && reverted*ray->origin.y<reverted*primitive->center.y) 
         {
            (*normal).x =  0.f;
            (*normal).y = -1.f;
            (*normal).z =  0.f;
            (*intersection).x = ray->origin.x+y*ray->direction.x/-ray->direction.y;
            (*intersection).y = primitive->center.y;
            (*intersection).z = ray->origin.z+y*ray->direction.z/-ray->direction.y;
            collision = 
               fabs((*intersection).x - primitive->center.x) < primitive->size.x &&
               fabs((*intersection).z - primitive->center.z) < primitive->size.z;
         }
         break;
      }
   case ptYZPlane:
      {
         float x = ray->origin.x-primitive->center.x;
         if( reverted*ray->direction.x<0.f && reverted*ray->origin.x>reverted*primitive->center.x ) 
         {
            (*normal).x =  1.f;
            (*normal).y =  0.f;
            (*normal).z =  0.f;
            (*intersection).x = primitive->center.x;
            (*intersection).y = ray->origin.y+x*ray->direction.y/-ray->direction.x;
            (*intersection).z = ray->origin.z+x*ray->direction.z/-ray->direction.x;
            collision = 
               fabs((*intersection).y - primitive->center.y) < primitive->size.y &&
               fabs((*intersection).z - primitive->center.z) < primitive->size.z;
         }
         if( !collision && reverted*ray->direction.x>0.f && reverted*ray->origin.x<reverted*primitive->center.x ) 
         {
            (*normal).x = -1.f;
            (*normal).y =  0.f;
            (*normal).z =  0.f;
            (*intersection).x = primitive->center.x;
            (*intersection).y = ray->origin.y+x*ray->direction.y/-ray->direction.x;
            (*intersection).z = ray->origin.z+x*ray->direction.z/-ray->direction.x;
            collision = 
               fabs((*intersection).y - primitive->center.y) < primitive->size.y &&
               fabs((*intersection).z - primitive->center.z) < primitive->size.z;
         }
         break;
      }
   case ptXYPlane:
      {
         float z = ray->origin.z-primitive->center.z;
         if( reverted*ray->direction.z<0.f && reverted*ray->origin.z>reverted*primitive->center.z) 
         {
            (*normal).x =  0.f;
            (*normal).y =  0.f;
            (*normal).z =  1.f;
            (*intersection).z = primitive->center.z;
            (*intersection).x = ray->origin.x+z*ray->direction.x/-ray->direction.z;
            (*intersection).y = ray->origin.y+z*ray->direction.y/-ray->direction.z;
            collision = 
               fabs((*intersection).x - primitive->center.x) < primitive->size.x &&
               fabs((*intersection).y - primitive->center.y) < primitive->size.y;
         }
         if( !collision && reverted*ray->direction.z>0.f && reverted*ray->origin.z<reverted*primitive->center.z )
         {
            (*normal).x =  0.f;
            (*normal).y =  0.f;
            (*normal).z = -1.f;
            (*intersection).z = primitive->center.z;
            (*intersection).x = ray->origin.x+z*ray->direction.x/-ray->direction.z;
            (*intersection).y = ray->origin.y+z*ray->direction.y/-ray->direction.z;
            collision = 
               fabs((*intersection).x - primitive->center.x) < primitive->size.x &&
               fabs((*intersection).y - primitive->center.y) < primitive->size.y;
         }
         break;
      }
   case ptCamera:
      {
         if( reverted*ray->direction.z<0.f && reverted*ray->origin.z>reverted*primitive->center.z )
         {
            (*normal).x =  0.f;
            (*normal).y =  0.f;
            (*normal).z =  1.f;
            (*intersection).z = primitive->center.z;
            float z = ray->origin.z-primitive->center.z;
            (*intersection).x = ray->origin.x+z*ray->direction.x/-ray->direction.z;
            (*intersection).y = ray->origin.y+z*ray->direction.y/-ray->direction.z;
            collision =
               fabs((*intersection).x - primitive->center.x) < primitive->size.x &&
               fabs((*intersection).y - primitive->center.y) < primitive->size.y;
         }
         break;
      }
   }

   if( collision ) 
   {
      (*shadowIntensity) = 1.f;
      float4 color;
      color = materials[primitive->materialId].color;
      if(materials[primitive->materialId].textureInfo.y != NO_TEXTURE)
      {
         color = cubeMapping(primitive, materials, textures, (*intersection) );
      }

      if( materials[primitive->materialId].transparency.x != 0.f && ((color.x+color.y+color.z)/3.f) >= transparentColor ) 
      {
         collision = false;
      }
      else 
      {
         (*shadowIntensity) = ((color.x+color.y+color.z)/3.f*(1.f-materials[primitive->materialId].transparency.x));
      }
   }
   return collision;
}

/*
________________________________________________________________________________

(*intersection) Shader
________________________________________________________________________________
*/
 float4 intersectionShader( 
   SceneInfo* sceneInfo,
   Primitive* primitive, 
   __global Material*  materials,
   __global char*      textures,
   //__global char*      kinectVideo,
   float4     intersection,
   float      timer, 
   bool       back )
{
   float4 colorAtIntersection = materials[primitive->materialId].color;
   switch( primitive->type ) 
   {
   case ptEnvironment:
   case ptSphere:
      if(materials[primitive->materialId].textureInfo.y != NO_TEXTURE)
      {
         colorAtIntersection = sphereUVMapping(primitive, materials, textures, intersection, timer );
      }
      break;
   case ptCheckboard :
      {
         if( materials[primitive->materialId].textureInfo.y != NO_TEXTURE ) 
         {
            colorAtIntersection = cubeMapping( primitive, materials, textures, intersection );
         }
         else 
         {
            int x = (*sceneInfo).viewDistance + ((intersection.x - primitive->center.x)/primitive->center.w*primitive->materialInfo.x);
            int z = (*sceneInfo).viewDistance + ((intersection.z - primitive->center.z)/primitive->center.w*primitive->materialInfo.y);
            if(x%2==0) 
            {
               if (z%2==0) 
               {
                  colorAtIntersection.x = 1.f-colorAtIntersection.x;
                  colorAtIntersection.y = 1.f-colorAtIntersection.y;
                  colorAtIntersection.z = 1.f-colorAtIntersection.z;
               }
            }
            else 
            {
               if (z%2!=0) 
               {
                  colorAtIntersection.x = 1.f-colorAtIntersection.x;
                  colorAtIntersection.y = 1.f-colorAtIntersection.y;
                  colorAtIntersection.z = 1.f-colorAtIntersection.z;
               }
            }
         }
         break;
      }
   case ptCylinder:
      {
         if(materials[primitive->materialId].textureInfo.y != NO_TEXTURE)
         {
            colorAtIntersection = sphereUVMapping(primitive, materials, textures, intersection, timer );
         }
         break;
      }
   case ptXYPlane:
   case ptYZPlane:
   case ptXZPlane:
      {
         if( materials[primitive->materialId].textureInfo.y != NO_TEXTURE ) 
         {
            colorAtIntersection = cubeMapping( primitive, materials, textures, intersection );
         }
         break;
      }
/*
   case ptCamera:
      {
         int x = (intersection.x-primitive.center.x+primitive.size.x)*primitive.materialInfo.x;
         int y = gKinectVideoHeight - (intersection.y-primitive.center.y+primitive.size.y)*primitive.materialInfo.y;

         x = (x+gKinectVideoWidth)%gKinectVideoWidth;
         y = (y+gKinectVideoHeight)%gKinectVideoHeight;

         if( x>=0 && x<gKinectVideoWidth && y>=0 && y<gKinectVideoHeight ) 
         {
            int index = (y*gKinectVideoWidth+x)*gKinectVideo;
            unsigned char r = kinectVideo[index+2];
            unsigned char g = kinectVideo[index+1];
            unsigned char b = kinectVideo[index+0];
            colorAtIntersection.x = r/256.f;
            colorAtIntersection.y = g/256.f;
            colorAtIntersection.z = b/256.f;
         }
         break;
      }
*/
   }
   return colorAtIntersection;
}

/*
________________________________________________________________________________

Shadows computation
We do not consider the object from which the ray is launched...
This object cannot shadow itself !

We now have to find the (*intersection) between the considered object and the ray 
which origin is the considered 3D float4 and which direction is defined by the 
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
 float processShadows(
   SceneInfo* sceneInfo,
   __global BoundingBox* boudingBoxes, int nbActiveBoxes,
   __global int*       boxPrimitivesIndex,
   __global Primitive* primitives,
   __global Material*  materials,
   __global char*      textures,
   int        nbPrimitives, 
   float4     lampCenter, 
   float4     origin, 
   int        objectId,
   int        iteration,
   float      timer)
{
   float result = 0.f;
   int cptBoxes = 0;
   while( result<=sceneInfo->shadowIntensity && cptBoxes < nbActiveBoxes )
   {
      Ray r;
      r.origin    = origin;
      r.direction = lampCenter-origin;
      computeRayAttributes( &r );

      if(boxIntersection(boudingBoxes[cptBoxes], &r, 0.f, sceneInfo->viewDistance))
      {
         int cptPrimitives = 0;
         while( result<sceneInfo->shadowIntensity && cptPrimitives<boudingBoxes[cptBoxes].nbPrimitives)
         {
            float4 intersection = {0.f,0.f,0.f,0.f};
            float4 normal       = {0.f,0.f,0.f,0.f};
            float  shadowIntensity = 0.f;

            if( boxPrimitivesIndex[boudingBoxes[cptBoxes].startIndex+cptPrimitives] != objectId && dotProduct(lampCenter,r.direction) > 0.f )
            {
               Primitive primitive = primitives[boxPrimitivesIndex[boudingBoxes[cptBoxes].startIndex+cptPrimitives]]; // TODO?

               bool hit = false;
               bool back;
               switch(primitive.type)
               {
               case ptEnvironment :
                  break;
               case ptSphere      : 
                  hit = sphereIntersection( sceneInfo, &primitive, materials, textures, &r, timer, &intersection, &normal, &shadowIntensity, &back ); 
                  break;
               case ptCylinder: 
                  hit = cylinderIntersection( sceneInfo, &primitive, materials, textures, &r, timer, &intersection, &normal, &shadowIntensity, &back ); 
                  break;
               default:
                  //TODO: hit = planeIntersection( primitive, materials, textures, r, true, shadowIntensity, intersection, normal, sceneInfo.transparentColor, timer ); 
                  break;
               }
               result += hit ? (shadowIntensity-materials[primitive.materialId].innerIllumination.x) : 0.f;
            }
            cptPrimitives++;
         }
      }
      cptBoxes++;
   }
   return (result>1.f) ? 1.f : result;
}

/*
________________________________________________________________________________

Primitive shader
________________________________________________________________________________
*/
 float4 primitiveShader(
   SceneInfo* sceneInfo,
   __global BoundingBox* boundingBoxes, int nbActiveBoxes,
   __global int* boxPrimitivesIndex, __global Primitive* primitives, int nbActivePrimitives,
   __global int* lamps, int nbActiveLamps,
   __global Material* materials, __global char* textures,
   //char*      kinectVideo,
   __global float* randoms,
   const float4 origin,
   const float4 normal, 
   const int    objectId, 
   const float4 intersection, 
   const int    iteration,
   const float  timer,
   float4*      refractionFromColor,
   float*       shadowIntensity,
   float4*      totalBlinn)
{
   Primitive primitive = primitives[objectId];
   float4 color = materials[primitives[objectId].materialId].color;
   float4 lampsColor = { 0.f, 0.f, 0.f, 0.f };

   // Lamp Impact
   float lambert      = 0.f;
   float totalLambert = (*sceneInfo).backgroundColor.w; // Ambient light
   (*shadowIntensity) = 0.f;

   //TODO? Lamps have constant color?? if( materials[primitive.materialId].innerIllumination.x != 0.f ) return color;

   if( primitives[objectId].type == ptEnvironment )
   {
      // Final color
      color = intersectionShader( 
         sceneInfo, &primitive, materials, textures, 
         //kinectVideo, 
         intersection, timer, false );
   }
   else 
   {
      color *= materials[primitive.materialId].innerIllumination.x;
      for( int cptLamps=0; cptLamps<nbActiveLamps; cptLamps++ ) 
      {
         if(lamps[cptLamps] != objectId)
         {
            float4 center = primitives[lamps[cptLamps]].center;
            if( (*sceneInfo).pathTracingIteration > 0 )
            {
               int t = 3*(*sceneInfo).pathTracingIteration + (int)(10.f*timer)%100;
               // randomize lamp center
               center.x += primitives[lamps[cptLamps]].size.x*randoms[t  ]*(float)(*sceneInfo).pathTracingIteration/(float)(*sceneInfo).maxPathTracingIterations;
               center.y += primitives[lamps[cptLamps]].size.y*randoms[t+1]*(float)(*sceneInfo).pathTracingIteration/(float)(*sceneInfo).maxPathTracingIterations;
               center.z += primitives[lamps[cptLamps]].size.z*randoms[t+2]*(float)(*sceneInfo).pathTracingIteration/(float)(*sceneInfo).maxPathTracingIterations;
            }

            if( (*sceneInfo).shadowsEnabled ) 
            {
               (*shadowIntensity) = processShadows(
                  sceneInfo, boundingBoxes, nbActiveBoxes,
                  boxPrimitivesIndex, primitives, materials, textures, 
                  nbActivePrimitives, center, 
                  intersection, lamps[cptLamps], iteration, timer );
            }

            float4 lightRay = center - intersection;
            normalizeVector(&lightRay);
         
            // Lighted object, not in the shades
            Material material = materials[primitives[lamps[cptLamps]].materialId];
            lampsColor += material.color*material.innerIllumination.x;

            // --------------------------------------------------------------------------------
            // Lambert
            // --------------------------------------------------------------------------------
            lambert = dotProduct(lightRay, normal);
            lambert = (lambert<0.f) ? 0.f : lambert;
            lambert *= (materials[primitive.materialId].refraction.x == 0.f) ? material.innerIllumination.x : 1.f;
            lambert *= (1.f-(*shadowIntensity));
            totalLambert += lambert;

            if( (*shadowIntensity) < (*sceneInfo).shadowIntensity )
            {
               // --------------------------------------------------------------------------------
               // Blinn - Phong
               // --------------------------------------------------------------------------------
               float4 viewRay = intersection - origin;
               normalizeVector(&viewRay);

               float4 blinnDir = lightRay - viewRay;
               float temp = sqrt(dotProduct(blinnDir,blinnDir));
               if (temp != 0.f ) 
               {
                  // Specular reflection
                  blinnDir = (1.f / temp) * blinnDir;

                  float blinnTerm = dotProduct(blinnDir,normal);
                  blinnTerm = ( blinnTerm < 0.f) ? 0.f : blinnTerm;

                  blinnTerm = materials[primitive.materialId].specular.x * pow(blinnTerm,materials[primitive.materialId].specular.y);
                  (*totalBlinn) += material.color * material.innerIllumination.x * blinnTerm;
               }
            }
         }
      }
      // Final color
      float4 intersectionColor =
         intersectionShader( sceneInfo, &primitive, materials, textures,
         //kinectVideo, 
         intersection, timer, false );

      color += intersectionColor*totalLambert*lampsColor;
      saturateVector(&color);

      (*refractionFromColor) = intersectionColor; // Refraction depending on color;
      saturateVector(totalBlinn);
   }
   return color;
}

/*
________________________________________________________________________________

Intersections with primitives
________________________________________________________________________________
*/
 bool intersectionWithPrimitives(
   SceneInfo* sceneInfo,
   __global BoundingBox* boundingBoxes, int nbActiveBoxes,
   __global int* boxPrimitivesIndex, 
   __global Primitive* primitives, int nbActivePrimitives,
   __global Material* materials, 
   __global char* textures,
   Ray*    ray, 
   int     iteration,
   float   timer, 
   int*    closestPrimitive, 
   float4* closestIntersection,
   float4* closestNormal,
   float4* colorBox,
   bool*   back)
{
   bool intersections = false; 
   float minDistance  = (*sceneInfo).viewDistance;
   
   Ray r;
   r.origin    = ray->origin;
   r.direction = ray->direction-ray->origin;
   computeRayAttributes( &r );

   float4 intersection = {0.f,0.f,0.f,0.f};
   float4 normal       = {0.f,0.f,0.f,0.f};
   bool i = false;
   float shadowIntensity = 0.f;

   for( int cptBoxes = 0; cptBoxes<nbActiveBoxes; ++cptBoxes )
   {
      BoundingBox box = boundingBoxes[cptBoxes];
      if( box.nbPrimitives != 0 && boxIntersection(box, &r, 0.f, sceneInfo->viewDistance/iteration) )
      {
         // Intersection with Box
         if( sceneInfo->renderBoxes ) (*colorBox) += materials[cptBoxes%NB_MAX_MATERIALS].color / 10.f;

         // Intersection with primitive within boxes
         for( int cptPrimitives=0; cptPrimitives<box.nbPrimitives; ++cptPrimitives )
         { 
            Primitive primitive = primitives[boxPrimitivesIndex[box.startIndex+cptPrimitives]];

            i = false;
            switch( primitive.type )
            {
            case ptEnvironment :
            case ptSphere      :
               {
                  i = sphereIntersection( sceneInfo, &primitive, materials, textures, &r, timer, &intersection, &normal, &shadowIntensity, back ); 
                  break;
               }
            case ptCylinder: 
               {
                  i = cylinderIntersection( sceneInfo, &primitive, materials, textures, &r, timer, &intersection, &normal, &shadowIntensity, back ); 
                  break;
               }
            default: 
               {
                  i = planeIntersection( &primitive, materials, textures, &r, false, &shadowIntensity, &intersection, &normal, sceneInfo->transparentColor, timer); 
                  break;
               }
            }

            float distance = vectorLength( intersection - r.origin ); // <- Pb ici!!
            if( i && distance>EPSILON && distance<minDistance ) 
            {
               // Only keep intersection with the closest object
               minDistance            = distance;
               (*closestPrimitive)    = boxPrimitivesIndex[box.startIndex+cptPrimitives];
               (*closestIntersection) = intersection;
               (*closestNormal)       = normal;
               intersections          = true;
            }
         }
      }
   }
   return intersections;
}


/*
________________________________________________________________________________

Calculate the reflected vector                   
                                                  
                  ^ Normal to object surface (N)  
Reflection (O_R)  |                              
                \ |  Eye (O_E)                    
                 \| /                             
  ----------------O--------------- Object surface 
        closestIntersection                      
                                                   
============================================================================== 
colours                                                                                    
------------------------------------------------------------------------------ 
We now have to know the colour of this (*intersection)                                        
Color_from_object will compute the amount of light received by the
(*intersection) float4 and  will also compute the shadows. 
The resulted color is stored in result.                     
The first parameter is the closest object to the (*intersection) (following 
the ray). It can  be considered as a light source if its inner light rate 
is > 0.                            
________________________________________________________________________________
*/
 float4 launchRay( 
   __global BoundingBox* boundingBoxes, int nbActiveBoxes,
   __global int* boxPrimitivesIndex, 
   __global Primitive* primitives, int nbActivePrimitives,
   __global int* lamps, int nbActiveLamps,
   __global Material*  materials, 
   __global char* textures,
   //__global char*      kinectVideo, 
   __global float*     randoms,
   Ray*       ray, 
   float      timer, 
   SceneInfo* sceneInfo,
   float4*    intersection,
   float*     depthOfField,
   int*       primitiveXYId)
{
   float4 intersectionColor   = sceneInfo->backgroundColor;

   float4 closestIntersection = {0.f,0.f,0.f,0.f};
   float4 firstIntersection   = {0.f,0.f,0.f,0.f};
   float4 normal              = {0.f,0.f,0.f,0.f};
   int    closestPrimitive  = 0;
   bool   carryon           = true;
   Ray    rayOrigin         = *ray;
   float  initialRefraction = 1.f;
   int    iteration         = 0;
   float  previousWeight    = 1.f;
   float4 recursiveColor = { 0.f, 0.f, 0.f, 0.f };
   float4 recursiveBlinn = { 0.f, 0.f, 0.f, 0.f };

   (*primitiveXYId) = -1;

   // Variable declarations
   float  shadowIntensity = 0.f;
   float4 refractionFromColor;
   float4 reflectedTarget;
   float4 colorBox = { 0.f, 0.f, 0.f, 0.f };
   bool   back;

   while( iteration<sceneInfo->nbRayIterations && carryon ) 
   {
      // If no (*intersection) with lamps detected. Now compute (*intersection) with Primitives
      if( carryon ) 
      {
         carryon = intersectionWithPrimitives(
            sceneInfo,
            boundingBoxes, nbActiveBoxes,
            boxPrimitivesIndex, primitives, nbActivePrimitives,
            materials,
            textures,
            &rayOrigin,
            iteration, timer, 
            &closestPrimitive, &closestIntersection, 
            &normal, &colorBox, &back);
      }

      if( carryon ) 
      {
         if ( iteration==0 )
         {
            intersectionColor.x = 0.f;
            intersectionColor.y = 0.f;
            intersectionColor.z = 0.f;
            intersectionColor.w = 0.f;

            firstIntersection = closestIntersection;
            (*primitiveXYId) = closestPrimitive;
         }

         float4 rBlinn = {0.f,0.f,0.f,0.f};
         // Get object color
         recursiveColor = primitiveShader( 
            sceneInfo,
            boundingBoxes, nbActiveBoxes,
            boxPrimitivesIndex, primitives, nbActivePrimitives, lamps, nbActiveLamps, materials, textures, 
            //kinectVideo, 
            randoms,
            rayOrigin.origin, normal, closestPrimitive, closestIntersection, 
            iteration, timer, &refractionFromColor, &shadowIntensity, &rBlinn );
         recursiveBlinn += rBlinn;
         recursiveColor -= colorBox;

         if( shadowIntensity != 1.f ) // No reflection/refraction if in shades
         {
            // ----------
            // Refraction
            // ----------
            if( materials[primitives[closestPrimitive].materialId].transparency.x != 0.f ) 
            {
#if EXTENDED_FEATURES
               // Replace the normal using the (*intersection) color
               // r,g,b become x,y,z... What the fuck!!
               if( materials[primitives[closestPrimitive].materialId].textureId != NO_TEXTURE) 
               {
                  refractionFromColor -= 0.5f;
                  normal *= refractionFromColor;
               }
#endif // EXTENDED_FEATURES

               float4 O_E = rayOrigin.origin - closestIntersection;
               normalizeVector(&O_E);
               float refraction = back ? 1.f : materials[primitives[closestPrimitive].materialId].refraction.x;
               vectorRefraction( &rayOrigin.direction, O_E, refraction, normal, initialRefraction );
               reflectedTarget = closestIntersection - rayOrigin.direction;

               recursiveColor *= previousWeight*(1.f-materials[primitives[closestPrimitive].materialId].transparency.x);
               // Prepare next ray
               initialRefraction = refraction;
   			   previousWeight = previousWeight*materials[primitives[closestPrimitive].materialId].transparency.x;
            }
            else 
            {
               // ----------
               // Reflection
               // ----------
               if( materials[primitives[closestPrimitive].materialId].reflection.x != 0.f ) 
               {
                  float4 O_E = rayOrigin.origin - closestIntersection;
                  vectorReflection( rayOrigin.direction, O_E, normal );

                  reflectedTarget = closestIntersection - rayOrigin.direction;
                  recursiveColor *= previousWeight*(1.f-materials[primitives[closestPrimitive].materialId].reflection);
                  previousWeight = previousWeight*materials[primitives[closestPrimitive].materialId].reflection.x;
               }
               else 
               {
    				   recursiveColor *= previousWeight;
                  carryon = false;
               }         
            }
         }
         else 
         {
				recursiveColor *= previousWeight;
            carryon = false;
         }
            
         rayOrigin.origin    = closestIntersection; 
         rayOrigin.direction = reflectedTarget;
         intersectionColor  += recursiveColor;

         // Noise management
         if( sceneInfo->pathTracingIteration != 0 && materials[primitives[closestPrimitive].materialId].color.w != 0.f)
         {
            // Randomize view
            int rindex = 3.f*timer + sceneInfo->pathTracingIteration;
            rindex = rindex%(sceneInfo->width*sceneInfo->height);
            rayOrigin.direction.x += randoms[rindex  ]*materials[primitives[closestPrimitive].materialId].color.w;
            rayOrigin.direction.y += randoms[rindex+1]*materials[primitives[closestPrimitive].materialId].color.w;
            rayOrigin.direction.z += randoms[rindex+2]*materials[primitives[closestPrimitive].materialId].color.w;
         }

      }

      iteration++; 
   }

   intersectionColor += recursiveBlinn;

   saturateVector( &intersectionColor );
   (*intersection) = closestIntersection;

   float4 O_I = firstIntersection - ray->origin;
#if EXTENDED_FEATURES
   // --------------------------------------------------
   // Attenation effect (Fog)
   // --------------------------------------------------
   float len = 1.f-(vectorLength(O_I)/sceneInfo->viewDistance);
   len = (len>0.f) ? len : 0.f; 
   intersectionColor.x = intersectionColor.x * len;
   intersectionColor.y = intersectionColor.y * len;
   intersectionColor.z = intersectionColor.z * len;
#endif // 0

   // Depth of field
   //float4 FI_I = firstIntersection - ray->direction;
   (*depthOfField) = (vectorLength(O_I)-(*depthOfField))/sceneInfo->viewDistance;
   return intersectionColor;
}

/*
________________________________________________________________________________

Standard renderer
________________________________________________________________________________
*/
__kernel void k_standardRenderer(
   __global BoundingBox* BoundingBoxes, int nbActiveBoxes,
   __global int* boxPrimitivesIndex, 
   __global Primitive* primitives, int nbActivePrimitives,
   __global int* lamps, int nbActiveLamps,
   __global Material* materials,
   __global char* textures,
   //__global char* kinectVideo,
   __global float* randoms,
   Ray ray,
   float4 angles,
   SceneInfo sceneInfo,
   float timer,
   PostProcessingInfo postProcessingInfo,
   __global float4* postProcessingBuffer,
   __global int* primitiveXYIds )
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;

   float4 rotationCenter = {0.f,0.f,0.f,0.f};

   if( sceneInfo.pathTracingIteration == 0 )
   {
      postProcessingBuffer[index].x = 0.f;
      postProcessingBuffer[index].y = 0.f;
      postProcessingBuffer[index].z = 0.f;
      postProcessingBuffer[index].w = 0.f;
   }
   else
   {
      // Randomize view
      int rindex = index + 3.f*timer + sceneInfo.pathTracingIteration;
      rindex = rindex%(sceneInfo.width*sceneInfo.height);
      ray.direction.x += randoms[rindex  ]*postProcessingBuffer[index].w*postProcessingInfo.param2*(float)sceneInfo.pathTracingIteration/(float)sceneInfo.maxPathTracingIterations;
      ray.direction.y += randoms[rindex+1]*postProcessingBuffer[index].w*postProcessingInfo.param2*(float)sceneInfo.pathTracingIteration/(float)sceneInfo.maxPathTracingIterations;
      ray.direction.z += randoms[rindex+2]*postProcessingBuffer[index].w*postProcessingInfo.param2*(float)sceneInfo.pathTracingIteration/(float)sceneInfo.maxPathTracingIterations;
   }

   float dof = postProcessingInfo.param1;
   float4 intersection;

#if 0 // Isometric 3D
   ray.direction.x = ray.direction.x - (ray.origin.z*0.01f)*(float)(x - (sceneInfo.width/2));
   ray.direction.y = ray.direction.y + (ray.origin.z*0.01f)*(float)(y - (sceneInfo.height/2));
   ray.origin.x = ray.direction.x;
   ray.origin.y = ray.direction.y;
#else
   ray.direction.x = ray.direction.x - 8.f*(float)(x - (sceneInfo.width/2));
   ray.direction.y = ray.direction.y + 8.f*(float)(y - (sceneInfo.height/2));
#endif // 0

   vectorRotation( ray.origin, rotationCenter, angles );
   vectorRotation( ray.direction, rotationCenter, angles );

   int primitiveXYId;
   float4 color = launchRay(
      BoundingBoxes, nbActiveBoxes,
      boxPrimitivesIndex, primitives, nbActivePrimitives,
      lamps, nbActiveLamps,
      materials, textures, 
      //kinectVideo, 
      randoms,
      &ray, timer, 
      &sceneInfo,
      &intersection,
      &dof,
      &primitiveXYId);

   primitiveXYIds[index] = primitiveXYId;
   
   postProcessingBuffer[index].x += color.x;
   postProcessingBuffer[index].y += color.y;
   postProcessingBuffer[index].z += color.z;
   if( sceneInfo.pathTracingIteration == 0 ) postProcessingBuffer[index].w = dof;
}

/*
________________________________________________________________________________

Anaglyph Renderer
________________________________________________________________________________
*/
__kernel void k_anaglyphRenderer(
   __global BoundingBox* BoundingBoxes, int nbActiveBoxes,
   __global int* boxPrimitivesIndex, 
   __global Primitive* primitives, int nbActivePrimitives,
   __global int* lamps, int nbActiveLamps,
   __global Material* materials,
   __global char* textures,
   //__global char* kinectVideo,
   __global float* randoms,
   Ray ray, 
   float4 angles,
   SceneInfo sceneInfo,
   float timer,
   PostProcessingInfo postProcessingInfo,
   __global float4* postProcessingBuffer,
   __global int* primitiveXYIds)
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;

   float4 rotationCenter = {0.f,0.f,0.f,0.f};

   if( sceneInfo.pathTracingIteration == 0 )
   {
      postProcessingBuffer[index].x = 0.f;
      postProcessingBuffer[index].y = 0.f;
      postProcessingBuffer[index].z = 0.f;
      postProcessingBuffer[index].w = 0.f;
   }

   float dof = postProcessingInfo.param1;
   float4 intersection;
   Ray eyeRay;

   // Left eye
   eyeRay.origin.x = ray.origin.x + sceneInfo.width3DVision;
   eyeRay.origin.y = ray.origin.y;
   eyeRay.origin.z = ray.origin.z;

   eyeRay.direction.x = ray.direction.x - 8.f*(float)(x - (sceneInfo.width/2));
   eyeRay.direction.y = ray.direction.y + 8.f*(float)(y - (sceneInfo.height/2));
   eyeRay.direction.z = ray.direction.z;

   vectorRotation( eyeRay.origin, rotationCenter, angles );
   vectorRotation( eyeRay.direction, rotationCenter, angles );

   int primitiveXYId;
   float4 colorLeft = launchRay(
      BoundingBoxes, nbActiveBoxes,
      boxPrimitivesIndex, primitives, nbActivePrimitives,
      lamps, nbActiveLamps,
      materials, textures, 
      //kinectVideo, 
      randoms,
      &eyeRay, timer, 
      &sceneInfo,
      &intersection,
      &dof,
      &primitiveXYId);
   primitiveXYIds[index] = primitiveXYId;

   // Right eye
   eyeRay.origin.x = ray.origin.x - sceneInfo.width3DVision;
   eyeRay.origin.y = ray.origin.y;
   eyeRay.origin.z = ray.origin.z;

   eyeRay.direction.x = ray.direction.x - 8.f*(float)(x - (sceneInfo.width/2));
   eyeRay.direction.y = ray.direction.y + 8.f*(float)(y - (sceneInfo.height/2));
   eyeRay.direction.z = ray.direction.z;

   vectorRotation( eyeRay.origin, rotationCenter, angles );
   vectorRotation( eyeRay.direction, rotationCenter, angles );

   float4 colorRight = launchRay(
      BoundingBoxes, nbActiveBoxes,
      boxPrimitivesIndex, primitives, nbActivePrimitives,
      lamps, nbActiveLamps,
      materials, textures, 
      //kinectVideo, 
      randoms,
      &eyeRay, timer, 
      &sceneInfo,
      &intersection,
      &dof,
      &primitiveXYId);
   primitiveXYIds[index] = primitiveXYId;

   float r1 = colorLeft.x*0.299f + colorLeft.y*0.587f + colorLeft.z*0.114f;
   float b1 = 0.f;
   float g1 = 0.f;

   float r2 = 0.f;
   float g2 = colorRight.y;
   float b2 = colorRight.z;

   postProcessingBuffer[index].x += r1+r2;
   postProcessingBuffer[index].y += g1+g2;
   postProcessingBuffer[index].z += b1+b2;
   if( sceneInfo.pathTracingIteration == 0 ) postProcessingBuffer[index].w = dof;
}

/*
________________________________________________________________________________

3D Vision Renderer
________________________________________________________________________________
*/
__kernel void k_3DVisionRenderer(
   __global BoundingBox* BoundingBoxes, int nbActiveBoxes,
   __global int* boxPrimitivesIndex, 
   __global Primitive*   primitives,    int nbActivePrimitives,
   __global int* lamps, int nbActiveLamps,
   __global Material*    materials,
   __global char*        textures,
   //__global char*        kinectVideo,
   __global float*    randoms,
   Ray                ray,
   float4             angles,
   SceneInfo          sceneInfo,
   float              timer,
   PostProcessingInfo postProcessingInfo,
   __global float4*   postProcessingBuffer,
   __global int* primitiveXYIds)
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;

   float4 rotationCenter = {0.f,0.f,0.f,0.f};

   if( sceneInfo.pathTracingIteration == 0 )
   {
      postProcessingBuffer[index].x = 0.f;
      postProcessingBuffer[index].y = 0.f;
      postProcessingBuffer[index].z = 0.f;
      postProcessingBuffer[index].w = 0.f;
   }

   float dof = postProcessingInfo.param1;
   float4 intersection;
   int halfWidth  = sceneInfo.width/2;

   Ray eyeRay;
   if( x<halfWidth ) 
   {
      // Left eye
      eyeRay.origin.x = ray.origin.x + sceneInfo.width3DVision;
      eyeRay.origin.y = ray.origin.y;
      eyeRay.origin.z = ray.origin.z;

      eyeRay.direction.x = ray.direction.x - 8.f*(float)(x - (sceneInfo.width/2) + halfWidth/2 );
      eyeRay.direction.y = ray.direction.y + 8.f*(float)(y - (sceneInfo.height/2));
      eyeRay.direction.z = ray.direction.z;
   }
   else
   {
      // Right eye
      eyeRay.origin.x = ray.origin.x - sceneInfo.width3DVision;
      eyeRay.origin.y = ray.origin.y;
      eyeRay.origin.z = ray.origin.z;

      eyeRay.direction.x = ray.direction.x - 8.f*(float)(x - (sceneInfo.width/2) - halfWidth/2);
      eyeRay.direction.y = ray.direction.y + 8.f*(float)(y - (sceneInfo.height/2));
      eyeRay.direction.z = ray.direction.z;
   }
      
   vectorRotation( eyeRay.origin, rotationCenter, angles );
   vectorRotation( eyeRay.direction, rotationCenter, angles );

   int primitiveXYId;
   float4 color = launchRay(
      BoundingBoxes, nbActiveBoxes,
      boxPrimitivesIndex, primitives, nbActivePrimitives,
      lamps, nbActiveLamps,
      materials, textures, 
      //kinectVideo, 
      randoms,
      &eyeRay, timer, 
      &sceneInfo,
      &intersection,
      &dof,
      &primitiveXYId);
   primitiveXYIds[index] = primitiveXYId;

   postProcessingBuffer[index].x += color.x;
   postProcessingBuffer[index].y += color.y;
   postProcessingBuffer[index].z += color.z;
   if( sceneInfo.pathTracingIteration == 0 ) postProcessingBuffer[index].w = dof;
}

/*
________________________________________________________________________________

Post Processing Effect: Depth of field
________________________________________________________________________________
*/
__kernel void k_depthOfField(
   SceneInfo          sceneInfo,
   PostProcessingInfo postProcessingInfo,
   __global float4*   postProcessingBuffer,
   __global float*    randoms,
   __global char*     bitmap) 
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;
   float  depth = postProcessingInfo.param2*postProcessingBuffer[index].w;
   int    wh = sceneInfo.width*sceneInfo.height;

   float4 localColor;
   localColor.x = 0.f;
   localColor.y = 0.f;
   localColor.z = 0.f;

   for( int i=0; i<postProcessingInfo.param3; ++i )
   {
      int ix = i%wh;
      int iy = (i+sceneInfo.width)%wh;
      int xx = x+depth*randoms[ix]*0.1f;
      int yy = y+depth*randoms[iy]*0.1f;
      if( xx>=0 && xx<sceneInfo.width && yy>=0 && yy<sceneInfo.height )
      {
         int localIndex = yy*sceneInfo.width+xx;
         if( localIndex>=0 && localIndex<wh )
         {
            localColor += postProcessingBuffer[localIndex];
         }
      }
      else
      {
         localColor += postProcessingBuffer[index];
      }
   }
   localColor /= postProcessingInfo.param3;
   localColor /= (sceneInfo.pathTracingIteration+1);
   localColor.w = 1.f;

   makeColor( sceneInfo.misc.x, localColor, bitmap, index ); 
}

/*
________________________________________________________________________________

Post Processing Effect: Ambiant Occlusion
________________________________________________________________________________
*/
__kernel void k_ambiantOcclusion(
   SceneInfo          sceneInfo,
   PostProcessingInfo postProcessingInfo,
   __global float4*   postProcessingBuffer,
   __global float*    randoms,
   __global char*     bitmap) 
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;
   float occ = 0.f;
   float4 localColor = postProcessingBuffer[index];
   float  depth = localColor.w;

   const int step = 3;
   for( int X=-step; X<step; ++X )
   {
      for( int Y=-step; Y<step; ++Y )
      {
         int xx = x+X;
         int yy = y+Y;
         if( xx>=0 && xx<sceneInfo.width && yy>=0 && yy<sceneInfo.height )
         {
            int localIndex = yy*sceneInfo.width+xx;
            if( postProcessingBuffer[localIndex].w >= depth )
            {
               occ += 1.f;
            }
         }
      }
   }
   occ /= (float)((2*step)*(2*step));
   occ += 0.5f; // Ambient light
   localColor.x *= occ;
   localColor.y *= occ;
   localColor.z *= occ;
   localColor /= (sceneInfo.pathTracingIteration+1);
   saturateVector( &localColor );
   localColor.w = 1.f;

   makeColor( sceneInfo.misc.x, localColor, bitmap, index ); 
}

/*
________________________________________________________________________________

Post Processing Effect: Cartoon
________________________________________________________________________________
*/
__kernel void k_cartoon(
   SceneInfo          sceneInfo,
   PostProcessingInfo postProcessingInfo,
   __global float4*   postProcessingBuffer,
   __global float*    randoms,
   __global char*     bitmap) 
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;
   float4 localColor = postProcessingBuffer[index];

   int r = localColor.x*255/postProcessingInfo.param3;
   int g = localColor.y*255/postProcessingInfo.param3;
   int b = localColor.z*255/postProcessingInfo.param3;

   localColor.x = (float)(r*postProcessingInfo.param3/255.f);
   localColor.y = (float)(g*postProcessingInfo.param3/255.f);
   localColor.z = (float)(b*postProcessingInfo.param3/255.f);
   localColor /= (sceneInfo.pathTracingIteration+1);

   localColor.w = 1.f;
   makeColor( sceneInfo.misc.x, localColor, bitmap, index ); 
}

/*
________________________________________________________________________________

Post Processing Effect: Ambiant Occlusion
________________________________________________________________________________
*/
__kernel void k_antiAliasing(
   SceneInfo          sceneInfo,
   PostProcessingInfo PostProcessingInfo,
   __global float4*   postProcessingBuffer,
   __global float*    randoms,
   __global char*     bitmap) 
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;
   float4 localColor = {0.f,0.f,0.f,0.f};

   for( int X=-1; X<=1; X+=2 )
   {
      for( int Y=-1; Y<=1; Y+=2 )
      {
         int xx = x+X;
         int yy = y+Y;
         if( xx>=0 && xx<sceneInfo.width && yy>=0 && yy<sceneInfo.height )
         {
            int localIndex = yy*sceneInfo.width+xx;
            localColor += 0.2f*postProcessingBuffer[localIndex];
         }
      }
   }
   localColor /= 4.f;
   localColor += postProcessingBuffer[index];
   localColor /= (sceneInfo.pathTracingIteration+1);
   saturateVector( &localColor );
   localColor.w = 1.f;
   
   makeColor( sceneInfo.misc.x, localColor, bitmap, index ); 
}

/*
________________________________________________________________________________

Post Processing Effect: Default
________________________________________________________________________________
*/
__kernel void k_default(
   SceneInfo          sceneInfo,
   PostProcessingInfo PostProcessingInfo,
   __global float4*   postProcessingBuffer,
   __global char*     bitmap) 
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   int index = y*sceneInfo.width+x;

   float4 localColor = postProcessingBuffer[index]/(sceneInfo.pathTracingIteration+1);
   makeColor( sceneInfo.misc.x, localColor, bitmap, index ); 
}
