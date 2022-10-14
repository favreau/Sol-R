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

// IMPORTANT
#ifdef MSVC
#define ALIGNMENT \
    __attribute__((packed)) // This is apparently required on Windows...
#else
#define ALIGNMENT
#endif

// Typedefs
typedef int4 PrimitiveXYIdBuffer;
typedef unsigned char BitmapBuffer;
typedef float RandomBuffer;
typedef int Lamp;
typedef struct
{
    float4 colorInfo;
    float4 sceneInfo;
} PostProcessingBuffer;

// Constants
#define NB_MAX_ITERATIONS 10
#define MAXDEPTH 10
#define CONST __global

#define NB_MAX_MATERIALS 65536 // Last 30 materials are reserved
#define gColorDepth 3

#define MATERIAL_NONE -1
#define TEXTURE_NONE -1
#define TEXTURE_MANDELBROT -2
#define TEXTURE_JULIA -3

// Globals
#define PI 3.14159265358979323846f

// Kinect
#define KINECT_COLOR_WIDTH 640
#define KINECT_COLOR_HEIGHT 480
#define KINECT_COLOR_DEPTH 4
#define KINECT_COLOR_SIZE 640 * 480 * 4

#define KINECT_DEPTH_WIDTH 320
#define KINECT_DEPTH_HEIGHT 240
#define KINECT_DEPTH_DEPTH 2
#define KINECT_DEPTH_SIZE 320 * 240 * 2

#define MAX_BITMAP_WIDTH 1920
#define MAX_BITMAP_HEIGHT 1080
#define MAX_BITMAP_SIZE 2073600 // 1920x1080

#define MATERIAL_DEFAULT_EMMISION_STRENGTH 0.6f
#define SKYBOX_LUNINANCE_STRENGTH 0.4f

enum FrameBufferType
{
    fbRGB = 0,
    fbBGR = 1
};

enum AdvancedIllumination
{
    aiNone = 0,
    aiBasic = 1,
    aiFull = 2,
    aiRandomIllumination = 3
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
typedef struct ALIGNMENT
{
    int2 size;                        // Image size
    enum CameraType cameraType;       // Camera type( Perspective, Orthographic,
                                      // Antialiazed, Volume Rendering )
    enum GraphicsLevel graphicsLevel; // Graphics level( No Shading=0,
                                      // Lambert=1, Specular=2, Reflections and
                                      // Refractions=3, Shadows=4 )
    int nbRayIterations; // Maximum number of ray iterations for current frame
    float transparentColor;   // Value above which r+g+b color is considered as
                              // transparent
    float viewDistance;       // Maximum viewing distance
    float shadowIntensity;    // Shadow intensity( off=0, pitch black=1)
    float eyeSeparation;      // 3D: Distance between both eyes
    int renderBoxes;          // Activate bounding box rendering( off=0, on=1 );
    int pathTracingIteration; // Current iteration for current frame
    int maxPathTracingIterations; // Maximum number of iterations for current
                                  // frame
    enum FrameBufferType frameBufferType;     // Frame buffer type( RGB or BGR )
    int timestamp;                            // Timestamp for the scene
    enum AtmosphericEffect atmosphericEffect; // Fog, etc
    int doubleSidedTriangles;                 // Use double-sided triangles
    int extendedGeometry;                     // Use extended geometry
    enum AdvancedIllumination advancedIllumination; // Advanced features (Global
                                                    // illumination, random
                                                    // lightning, etc)
    int draftMode;          // Draft mode when camera in motion
    int skyboxRadius;       // Skybox sphere radius
    int skyboxMaterialId;   // Skybox material Id
    int gradientBackground; // Gradient background
    float geometryEpsilon;  // Geometry epsilon
    float rayEpsilon;       // Ray epsilon
    float4 backgroundColor; // Background color
} SceneInfo;

typedef struct ALIGNMENT
{
    float4 origin;        // Origin of the ray
    float4 direction;     // Direction of the ray
    float4 inv_direction; // Inverted direction( Used for optimal Ray-Box
                          // intersection )
    int4 signs;           // Signs ( Used for optimal Ray-Box intersection )
} Ray;

typedef struct ALIGNMENT
{
    int primitiveId; // ID of the emitting primitive
    int materialId;  // ID of the emitting material
    float4 location; // Position in space
    float4 color;    // Light
} LightInformation;

// Enums
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
    ptQuad = 11
};

typedef struct ALIGNMENT
{
    float4 innerIllumination; // x: Inner illumination
    // y: Diffusion strength
    // z: <not used>
    // w: Noise
    float4 color;    // Color( R,G,B )
    float4 specular; // x: Value
    // y: Power
    // z: <not used>
    // w: <not used>
    float reflection; // Reflection rate( No reflection=0 -> Full reflection=1 )
    float refraction; // Refraction index( ex: glass=1.33 )
    float transparency; // Transparency rate( Opaque=0 -> Full transparency=1 )
    float opacity;      // Opacity strength
    int4 attributes; // x: Fast transparency( off=0, on=1 ). Fast transparency
                     // produces no shadows
    //    and drops intersections if rays intersects primitive with the same
    //    material ID
    // y: Procedural textures( off=0, on=1 )
    // z: Wireframe( off=0, on=1 ). Wire frame produces no shading
    // w: Wireframe Width
    int4 textureMapping; // x: U padding
    // y: V padding
    // z: Texture ID (Deprecated)
    // w: Texture color depth
    int4 textureOffset; // x: Offset in the diffuse map
    // y: Offset in the normal map
    // z: Offset in the bump map
    // w: Offset in the specular map
    int4 textureIds; // x: Diffuse map
    // y: Normal map
    // z: Bump map
    // w: Specular map
    int4 advancedTextureOffset; // x: Offset in the Reflection map
    // y: Offset in the Transparency map
    // z: Offset in the Ambient Occulsion map
    // w: not used
    int4 advancedTextureIds; // x: Reflection map
    // y: Transparency map
    // z: Ambient Occulsion map
    // w: not used
    float2 mappingOffset; // Texture mapping offsets based on
                          // sceneInfo.timestamp
} Material;

typedef struct ALIGNMENT
{
    float4 parameters[2]; // Bottom-Left and Top-Right corners
    int nbPrimitives;     // Number of primitives in the box
    int startIndex;       // Index of the first primitive in the box
    int2 indexForNextBox; // If no intersection, how many of the following boxes
                          // can be skipped?
} BoundingBox;

typedef struct ALIGNMENT
{
    // Vertices
    float4 p0;
    float4 p1;
    float4 p2;
    // Normals
    float4 n0;
    float4 n1;
    float4 n2;
    // Size( x,y,z )
    float4 size;
    // Type( See PrimitiveType )
    int type;
    // Index
    int index;
    // Material ID
    int materialId;
    // Texture coordinates
    float2 vt0;
    float2 vt1;
    float2 vt2;
} Primitive;

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

typedef struct ALIGNMENT
{
    unsigned char* buffer; // Pointer to the texture
    int offset; // Offset of the texture in the global texture buffer (the one
                // that will be transfered to the GPU)
    int3 size;  // Size of the texture
    enum TextureType type;
} TextureInfo;

// Post processing effect
enum PostProcessingType
{
    ppe_none,             // No effect
    ppe_depthOfField,     // Depth of field
    ppe_ambientOcclusion, // Ambient occlusion
    ppe_radiosity,        // Radiosity
    ppe_oneColor
};

typedef struct ALIGNMENT
{
    int type;
    float param1; // pointOfFocus;
    float param2; // strength;
    int param3;   // iterations;
} PostProcessingInfo;

// ________________________________________________________________________________
static void saturateVector(float4* v)
{
    (*v).x = ((*v).x < 0.f) ? 0.f : (*v).x;
    (*v).y = ((*v).y < 0.f) ? 0.f : (*v).y;
    (*v).z = ((*v).z < 0.f) ? 0.f : (*v).z;
    (*v).w = ((*v).w < 0.f) ? 0.f : (*v).w;

    (*v).x = ((*v).x > 1.f) ? 1.f : (*v).x;
    (*v).y = ((*v).y > 1.f) ? 1.f : (*v).y;
    (*v).z = ((*v).z > 1.f) ? 1.f : (*v).z;
    (*v).w = ((*v).w > 1.f) ? 1.f : (*v).w;
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
#define vectorReflection(__r, __i, __n) __r = __i - 2.f * dot(__i, __n) * __n;

/*
________________________________________________________________________________
incident: le vecteur norm? inverse ? la direction d?incidence de la source
lumineuse
n1      : index of refraction of original medium
n2      : index of refraction of new medium
________________________________________________________________________________
*/
static void vectorRefraction(float4* refracted, const float4 incident,
                             const float n1, const float4 normal,
                             const float n2)
{
    (*refracted) = incident;
    if (n2 != 0.f)
    {
        float eta = n1 / n2;
        float c1 = -dot(incident, normal);
        float cs2 = 1.f - eta * eta * (1.f - c1 * c1);
        if (cs2 >= 0.f)
            (*refracted) = eta * incident + (eta * c1 - sqrt(cs2)) * normal;
    }
}

/*
________________________________________________________________________________
__v : Vector to rotate
__c : Center of rotations
__a : Angles
________________________________________________________________________________
*/
static void vectorRotation(float4* vector, const float4 center,
                           const float4 angles)
{
    float4 __r = (*vector);
    /* X axis */
    __r.y = (*vector).y * half_cos(angles.x) - (*vector).z * half_sin(angles.x);
    __r.z = (*vector).y * half_sin(angles.x) + (*vector).z * half_cos(angles.x);
    (*vector) = __r;
    __r = (*vector);
    /* Y axis */
    __r.z = (*vector).z * half_cos(angles.y) - (*vector).x * half_sin(angles.y);
    __r.x = (*vector).z * half_sin(angles.y) + (*vector).x * half_cos(angles.y);
    (*vector) = __r;
}

/*
________________________________________________________________________________

Compute ray attributes
________________________________________________________________________________
*/
static void computeRayAttributes(Ray* ray)
{
    (*ray).inv_direction.x =
        ((*ray).direction.x != 0.f) ? 1.f / (*ray).direction.x : 0.f;
    (*ray).inv_direction.y =
        ((*ray).direction.y != 0.f) ? 1.f / (*ray).direction.y : 0.f;
    (*ray).inv_direction.z =
        ((*ray).direction.z != 0.f) ? 1.f / (*ray).direction.z : 0.f;
    (*ray).signs.x = ((*ray).inv_direction.x < 0);
    (*ray).signs.y = ((*ray).inv_direction.y < 0);
    (*ray).signs.z = ((*ray).inv_direction.z < 0);
}

/*
________________________________________________________________________________

Convert float4 into OpenGL RGB color
________________________________________________________________________________
*/
static void makeColor(const SceneInfo* sceneInfo, const float4* color,
                      CONST BitmapBuffer* bitmap, int index)
{
    (*color).x = ((*color).x > 1.f) ? 1.f : (*color).x;
    (*color).y = ((*color).y > 1.f) ? 1.f : (*color).y;
    (*color).z = ((*color).z > 1.f) ? 1.f : (*color).z;
    (*color).x = ((*color).x < 0.f) ? 0.f : (*color).x;
    (*color).y = ((*color).y < 0.f) ? 0.f : (*color).y;
    (*color).z = ((*color).z < 0.f) ? 0.f : (*color).z;

    int mdc_index = index * gColorDepth;
    switch ((*sceneInfo).frameBufferType)
    {
    case fbBGR:
    {
        // Delphi
        int y = index / (*sceneInfo).size.y;
        int x = index % (*sceneInfo).size.x;
        int i = (y + 1) * (*sceneInfo).size.y - x - 1;
        i *= gColorDepth;
        bitmap[i] = (BitmapBuffer)((*color).z * 255.f);     // Blue
        bitmap[i + 1] = (BitmapBuffer)((*color).y * 255.f); // Green
        bitmap[i + 2] = (BitmapBuffer)((*color).x * 255.f); // Red
        break;
    }
    default:
    {
        // OpenGL
        bitmap[mdc_index] = (BitmapBuffer)((*color).x * 255.f);     // Red
        bitmap[mdc_index + 1] = (BitmapBuffer)((*color).y * 255.f); // Green
        bitmap[mdc_index + 2] = (BitmapBuffer)((*color).z * 255.f); // Blue
        break;
    }
    }
}

/*
________________________________________________________________________________

Mandelbrot Set
________________________________________________________________________________
*/
static void juliaSet(CONST Primitive* primitive, CONST Material* materials,
                     const SceneInfo* sceneInfo, const float x, const float y,
                     float4* color)
{
    CONST Material* material = &materials[(*primitive).materialId];
    float W = (float)(*material).textureMapping.x;
    float H = (float)(*material).textureMapping.y;

    // pick some values for the constant c, this determines the shape of the
    // Julia Set
    float cRe = -0.7f + 0.4f * sin((*sceneInfo).timestamp / 1500.f);
    float cIm = 0.27015f + 0.4f * cos((*sceneInfo).timestamp / 2000.f);

    // calculate the initial real and imaginary part of z, based on the pixel
    // location and zoom and position values
    float newRe = 1.5f * (x - W / 2.f) / (0.5f * W);
    float newIm = (y - H / 2.f) / (0.5f * H);
    // i will represent the number of iterations
    int n;
    // start the iteration process
    float maxIterations = 40.f + (*sceneInfo).pathTracingIteration;
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
    (*color).x = 1.f - (*color).x * (n / maxIterations);
    (*color).y = 1.f - (*color).y * (n / maxIterations);
    (*color).z = 1.f - (*color).z * (n / maxIterations);
    (*color).w = 1.f - (n / maxIterations);
}

/*
________________________________________________________________________________

Mandelbrot Set
________________________________________________________________________________
*/
static void mandelbrotSet(CONST Primitive* primitive, CONST Material* materials,
                          const SceneInfo* sceneInfo, const float x,
                          const float y, float4* color)
{
    CONST Material* material = &materials[(*primitive).materialId];
    float W = (float)(*material).textureMapping.x;
    float H = (float)(*material).textureMapping.y;

    float MinRe = -2.f;
    float MaxRe = 1.f;
    float MinIm = -1.2f;
    float MaxIm = MinIm + (MaxRe - MinRe) * H / W;
    float Re_factor = (MaxRe - MinRe) / (W - 1.f);
    float Im_factor = (MaxIm - MinIm) / (H - 1.f);
    float maxIterations = NB_MAX_ITERATIONS + (*sceneInfo).pathTracingIteration;

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
            isInside = false;
        Z_im = 2.f * Z_re * Z_im + c_im;
        Z_re = Z_re2 - Z_im2 + c_re;
    }

    (*color).x = 1.f - (*color).x * (n / maxIterations);
    (*color).y = 1.f - (*color).y * (n / maxIterations);
    (*color).z = 1.f - (*color).z * (n / maxIterations);
    (*color).w = 1.f - (n / maxIterations);
}

// ----------
// Normal mapping
// --------------------
static void normalMap(const int index, CONST Material* material,
                      CONST BitmapBuffer* textures, float4* normal,
                      const float strength)
{
    int i = (*material).textureOffset.y + index;
    BitmapBuffer r, g;
    r = textures[i];
    g = textures[i + 1];
    (*normal).x -= strength * (r / 256.f - 0.5f);
    (*normal).y -= strength * (g / 256.f - 0.5f);
}

// ----------
// Bump mapping
// --------------------
static void bumpMap(const int index, CONST Material* material,
                    CONST BitmapBuffer* textures, float4* intersection,
                    float* value)
{
    int i = (*material).textureOffset.z + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    (*value) = 10.f * (r + g + b) / 768.f;
}

// ----------
// Specular mapping
// --------------------
static void specularMap(const int index, CONST Material* material,
                        CONST BitmapBuffer* textures, float4* specular)
{
    int i = (*material).textureOffset.w + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    (*specular).x *= (r + g + b) / 768.f;
}

// ----------
// Reflection mapping
// --------------------
static void reflectionMap(const int index, CONST Material* material,
                          CONST BitmapBuffer* textures, float4* attributes)
{
    int i = (*material).advancedTextureOffset.x + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    (*attributes).x *= (r + g + b) / 768.f;
}

// ----------
// Transparency mapping
// --------------------
static void transparencyMap(const int index, CONST Material* material,
                            CONST BitmapBuffer* textures, float4* attributes)
{
    int i = (*material).advancedTextureOffset.y + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    (*attributes).y *= (r + g + b) / 768.f;
    //   (*attributes).z = 10.f*b/256.f;
}

// ----------
// Ambient occlusion
// --------------------
static void ambientOcclusionMap(const int index, CONST Material* material,
                                CONST BitmapBuffer* textures,
                                float4* advancedAttributes)
{
    int i = (*material).advancedTextureOffset.z + index;
    BitmapBuffer r, g, b;
    r = textures[i];
    g = textures[i + 1];
    b = textures[i + 2];
    (*advancedAttributes).x = (r + g + b) / 768.f;
}

/*
________________________________________________________________________________

Sphere texture Mapping
________________________________________________________________________________
*/
static float4 sphereUVMapping(CONST Primitive* primitive,
                              CONST Material* materials,
                              CONST BitmapBuffer* textures,
                              float4* intersection, float4* normal,
                              float4* specular, float4* attributes,
                              float4* advancedAttributes)
{
    CONST Material* material = &materials[(*primitive).materialId];
    float4 result = (*material).color;
    float4 I = normalize((*intersection) - (*primitive).p0);
    float U = ((atan2(I.x, I.z) / PI) + 1.f) * .5f;
    float V = (asin(I.y) / PI) + .5f;

    int u = (*material).textureMapping.x * (U * (*primitive).vt1.x);
    int v = (*material).textureMapping.y * (V * (*primitive).vt1.y);

    if ((*material).textureMapping.x != 0)
        u %= (*material).textureMapping.x;
    if ((*material).textureMapping.y != 0)
        v %= (*material).textureMapping.y;
    const bool condition = u >= 0 && u < (*material).textureMapping.x &&
                           v >= 0 && v < (*material).textureMapping.y;
    if (condition)
    {
        int A = (v * (*material).textureMapping.x + u) *
                (*material).textureMapping.w;
        int B = (*material).textureMapping.x * (*material).textureMapping.y *
                (*material).textureMapping.w;
        int index = A % B;

        // Diffuse
        int i = (*material).textureOffset.x + index;
        BitmapBuffer r, g, b;
        r = textures[i];
        g = textures[i + 1];
        b = textures[i + 2];
        result.x = r / 256.f;
        result.y = g / 256.f;
        result.z = b / 256.f;

        float strength = 3.f;
        // Bump mapping
        if ((*material).textureIds.z != TEXTURE_NONE)
            bumpMap(index, material, textures, intersection, &strength);
        // Normal mapping
        if ((*material).textureIds.y != TEXTURE_NONE)
            normalMap(index, material, textures, normal, strength);
        // Specular mapping
        if ((*material).textureIds.w != TEXTURE_NONE)
            specularMap(index, material, textures, specular);
        // Reflection mapping
        if ((*material).advancedTextureIds.x != TEXTURE_NONE)
            reflectionMap(index, material, textures, attributes);
        // Transparency mapping
        if ((*material).advancedTextureIds.y != TEXTURE_NONE)
            transparencyMap(index, material, textures, attributes);
        // Ambient occulusion mapping
        if ((*material).advancedTextureIds.z != TEXTURE_NONE)
            ambientOcclusionMap(index, material, textures, advancedAttributes);
    }
    return result;
}

/*
________________________________________________________________________________

Cube texture mapping
________________________________________________________________________________
*/
static float4 cubeMapping(const SceneInfo* sceneInfo,
                          CONST Primitive* primitive, CONST Material* materials,
                          CONST BitmapBuffer* textures, float4* intersection,
                          float4* normal, float4* specular, float4* attributes,
                          float4* advancedAttributes)
{
    CONST Material* material = &materials[(*primitive).materialId];
    float4 result = (*material).color;

#ifdef USE_KINECT
    if (primitive.type.x == ptCamera)
    {
        int x = ((*intersection).x - (*primitive).p0.x + (*primitive).size.x) *
                material.textureMapping.x;
        int y = KINECT_COLOR_HEIGHT -
                ((*intersection).y - (*primitive).p0.y + (*primitive).size.y) *
                    material.textureMapping.y;

        x = (x + KINECT_COLOR_WIDTH) % KINECT_COLOR_WIDTH;
        y = (y + KINECT_COLOR_HEIGHT) % KINECT_COLOR_HEIGHT;

        const bool condition = x >= 0 && x < KINECT_COLOR_WIDTH && y >= 0 &&
                               y < KINECT_COLOR_HEIGHT;
        if (condition)
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
        int u =
            (((*primitive).type == ptCheckboard) ||
             ((*primitive).type == ptXZPlane) ||
             ((*primitive).type == ptXYPlane))
                ? ((*intersection).x - (*primitive).p0.x + (*primitive).size.x)
                : ((*intersection).z - (*primitive).p0.z + (*primitive).size.z);

        int v =
            (((*primitive).type == ptCheckboard) ||
             ((*primitive).type == ptXZPlane))
                ? ((*intersection).z + (*primitive).p0.z + (*primitive).size.z)
                : ((*intersection).y - (*primitive).p0.y + (*primitive).size.y);

        if ((*material).textureMapping.x != 0)
            u = u % (*material).textureMapping.x;
        if ((*material).textureMapping.y != 0)
            v = v % (*material).textureMapping.y;

        if (u >= 0 && u < (*material).textureMapping.x && v >= 0 &&
            v < (*material).textureMapping.x)
        {
            switch ((*material).textureIds.x)
            {
            case TEXTURE_MANDELBROT:
                mandelbrotSet(primitive, materials, sceneInfo, u, v, &result);
                break;
            case TEXTURE_JULIA:
                juliaSet(primitive, materials, sceneInfo, u, v, &result);
                break;
            default:
            {
                int A = (v * (*material).textureMapping.x + u) *
                        (*material).textureMapping.w;
                int B = (*material).textureMapping.x *
                        (*material).textureMapping.y *
                        (*material).textureMapping.w;
                int index = A % B;
                int i = (*material).textureOffset.x + index;
                BitmapBuffer r, g, b;
                r = textures[i];
                g = textures[i + 1];
                b = textures[i + 2];
                result.x = r / 256.f;
                result.y = g / 256.f;
                result.z = b / 256.f;

                float strength = 3.f;
                // Bump mapping
                if ((*material).textureIds.z != TEXTURE_NONE)
                    bumpMap(index, material, textures, intersection, &strength);
                // Normal mapping
                if ((*material).textureIds.y != TEXTURE_NONE)
                    normalMap(index, material, textures, normal, strength);
                // Specular mapping
                if ((*material).textureIds.w != TEXTURE_NONE)
                    specularMap(index, material, textures, specular);
                // Reflection mapping
                if ((*material).advancedTextureIds.x != TEXTURE_NONE)
                    reflectionMap(index, material, textures, attributes);
                // Transparency mapping
                if ((*material).advancedTextureIds.y != TEXTURE_NONE)
                    transparencyMap(index, material, textures, attributes);
                // Ambient occlusion mapping
                if ((*material).advancedTextureIds.z != TEXTURE_NONE)
                    ambientOcclusionMap(index, material, textures,
                                        advancedAttributes);
            }
            break;
            }
        }
    }
    return result;
}

/*
________________________________________________________________________________

Triangle texture Mapping
________________________________________________________________________________
*/
static float4 triangleUVMapping(const SceneInfo* sceneInfo,
                                CONST Primitive* primitive,
                                CONST Material* materials,
                                CONST BitmapBuffer* textures,
                                float4* intersection, const float4 areas,
                                float4* normal, float4* specular,
                                float4* attributes, float4* advancedAttributes)
{
    CONST Material* material = &materials[(*primitive).materialId];
    float4 result = (*material).color;

    float2 T = ((*primitive).vt0 * areas.x + (*primitive).vt1 * areas.y +
                (*primitive).vt2 * areas.z) /
               (areas.x + areas.y + areas.z);
    float2 mappingOffset = {0.f, 0.f};
    if ((*material).attributes.y == 1)
    {
        mappingOffset.x = (*material).mappingOffset.x * (*sceneInfo).timestamp;
        mappingOffset.y = (*material).mappingOffset.y * (*sceneInfo).timestamp;
    }
    int u = T.x * (*material).textureMapping.x + mappingOffset.x;
    int v = T.y * (*material).textureMapping.y + mappingOffset.y;

    u = u % (*material).textureMapping.x;
    v = v % (*material).textureMapping.y;
    const bool condition = u >= 0 && u < (*material).textureMapping.x &&
                           v >= 0 && v < (*material).textureMapping.y;
    if (condition)
    {
        switch ((*material).textureIds.x)
        {
        case TEXTURE_MANDELBROT:
            mandelbrotSet(primitive, materials, sceneInfo, u, v, &result);
            break;
        case TEXTURE_JULIA:
            juliaSet(primitive, materials, sceneInfo, u, v, &result);
            break;
        default:
        {
            int A = (v * (*material).textureMapping.x + u) *
                    (*material).textureMapping.w;
            int B = (*material).textureMapping.x *
                    (*material).textureMapping.y * (*material).textureMapping.w;
            int index = A % B;

            // Diffuse
            int i = (*material).textureOffset.x + index;
            BitmapBuffer r, g, b;
            r = textures[i];
            g = textures[i + 1];
            b = textures[i + 2];
#ifdef USE_KINECT
            if ((*material).textureIds.x == 0)
            {
                r = textures[index + 2];
                g = textures[index + 1];
                b = textures[index];
            }
#endif // USE_KINECT
            result.x = r / 256.f;
            result.y = g / 256.f;
            result.z = b / 256.f;

            float strength = 3.f;
            // Bump mapping
            if ((*material).textureIds.z != TEXTURE_NONE)
            {
                bumpMap(index, material, textures, intersection, &strength);
                (*attributes).w *= strength / 10.f;
            }
            // Normal mapping
            if ((*material).textureIds.y != TEXTURE_NONE)
                normalMap(index, material, textures, normal, strength);
            // Specular mapping
            if ((*material).textureIds.w != TEXTURE_NONE)
                specularMap(index, material, textures, specular);
            // Reflection mapping
            if ((*material).advancedTextureIds.x != TEXTURE_NONE)
                reflectionMap(index, material, textures, attributes);
            // Transparency mapping
            if ((*material).advancedTextureIds.y != TEXTURE_NONE)
                transparencyMap(index, material, textures, attributes);
            // Ambient occulusion mapping
            if ((*material).advancedTextureIds.z != TEXTURE_NONE)
                ambientOcclusionMap(index, material, textures,
                                    advancedAttributes);
        }
        }
    }
    return result;
}

/*
________________________________________________________________________________

Box intersection
________________________________________________________________________________
*/
static bool boxIntersection(CONST BoundingBox* box, const Ray* ray,
                            const float t0, const float t1)
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    tmin = ((*box).parameters[(*ray).signs.x].x - (*ray).origin.x) *
           (*ray).inv_direction.x;
    tmax = ((*box).parameters[1 - (*ray).signs.x].x - (*ray).origin.x) *
           (*ray).inv_direction.x;
    tymin = ((*box).parameters[(*ray).signs.y].y - (*ray).origin.y) *
            (*ray).inv_direction.y;
    tymax = ((*box).parameters[1 - (*ray).signs.y].y - (*ray).origin.y) *
            (*ray).inv_direction.y;

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    tzmin = ((*box).parameters[(*ray).signs.z].z - (*ray).origin.z) *
            (*ray).inv_direction.z;
    tzmax = ((*box).parameters[1 - (*ray).signs.z].z - (*ray).origin.z) *
            (*ray).inv_direction.z;

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

Ellipsoid intersection
________________________________________________________________________________
*/
static bool ellipsoidIntersection(const SceneInfo* sceneInfo,
                                  CONST Primitive* ellipsoid,
                                  CONST Material* materials, const Ray* ray,
                                  float4* intersection, float4* normal,
                                  float* shadowIntensity)
{
    // Shadow intensity
    (*shadowIntensity) = 1.f;

    // solve the equation sphere-ray to find the intersections
    float4 O_C = (*ray).origin - (*ellipsoid).p0;
    float4 dir = normalize((*ray).direction);

    float a = ((dir.x * dir.x) / ((*ellipsoid).size.x * (*ellipsoid).size.x)) +
              ((dir.y * dir.y) / ((*ellipsoid).size.y * (*ellipsoid).size.y)) +
              ((dir.z * dir.z) / ((*ellipsoid).size.z * (*ellipsoid).size.z));
    float b =
        ((2.f * O_C.x * dir.x) / ((*ellipsoid).size.x * (*ellipsoid).size.x)) +
        ((2.f * O_C.y * dir.y) / ((*ellipsoid).size.y * (*ellipsoid).size.y)) +
        ((2.f * O_C.z * dir.z) / ((*ellipsoid).size.z * (*ellipsoid).size.z));
    float c = ((O_C.x * O_C.x) / ((*ellipsoid).size.x * (*ellipsoid).size.x)) +
              ((O_C.y * O_C.y) / ((*ellipsoid).size.y * (*ellipsoid).size.y)) +
              ((O_C.z * O_C.z) / ((*ellipsoid).size.z * (*ellipsoid).size.z)) -
              1.f;

    float d = ((b * b) - (4.f * a * c));
    const float condition = d < 0.f || a == 0.f || b == 0.f || c == 0.f;
    if (condition)
        return false;
    d = sqrt(d);

    float t1 = (-b + d) / (2.f * a);
    float t2 = (-b - d) / (2.f * a);

    if (t1 <= (*sceneInfo).geometryEpsilon &&
        t2 <= (*sceneInfo).geometryEpsilon)
        return false; // both intersections are behind the ray origin

    float t = 0.f;
    if (t1 <= (*sceneInfo).geometryEpsilon)
        t = t2;
    else if (t2 <= (*sceneInfo).geometryEpsilon)
        t = t1;
    else
        t = (t1 < t2) ? t1 : t2;

    if (t < (*sceneInfo).geometryEpsilon)
        return false; // Too close to intersection
    (*intersection) = (*ray).origin + t * dir;

    (*normal) = (*intersection) - (*ellipsoid).p0;
    (*normal).x =
        2.f * (*normal).x / ((*ellipsoid).size.x * (*ellipsoid).size.x);
    (*normal).y =
        2.f * (*normal).y / ((*ellipsoid).size.y * (*ellipsoid).size.y);
    (*normal).z =
        2.f * (*normal).z / ((*ellipsoid).size.z * (*ellipsoid).size.z);

    (*normal) = normalize(*normal);
    return true;
}

/*
________________________________________________________________________________

Skybox mapping
________________________________________________________________________________
*/
static float4 skyboxMapping(const SceneInfo* sceneInfo,
                            CONST Material* materials,
                            CONST BitmapBuffer* textures, const Ray* ray)
{
    CONST Material* material = &materials[(*sceneInfo).skyboxMaterialId];
    float4 result = (*material).color;

    // solve the equation sphere-ray to find the intersections
    float4 dir = normalize((*ray).direction - (*ray).origin);

    float a = 2.f * dot(dir, dir);
    float b = 2.f * dot((*ray).origin, dir);
    float c = dot((*ray).origin, (*ray).origin) -
              ((*sceneInfo).skyboxRadius * (*sceneInfo).skyboxRadius);
    float d = b * b - 2.f * a * c;

    if (d <= 0.f || a == 0.f)
        return result;
    float r = sqrt(d);
    float t1 = (-b - r) / a;
    float t2 = (-b + r) / a;

    if (t1 <= (*sceneInfo).geometryEpsilon &&
        t2 <= (*sceneInfo).geometryEpsilon)
        return result; // both intersections are behind the ray origin

    float t = 0.f;
    if (t1 <= (*sceneInfo).geometryEpsilon)
        t = t2;
    else if (t2 <= (*sceneInfo).geometryEpsilon)
        t = t1;
    else
        t = (t1 < t2) ? t1 : t2;

    if (t < (*sceneInfo).geometryEpsilon)
        return result; // Too close to intersection
    float4 intersection = normalize((*ray).origin + t * dir);

    // Intersection found, no get skybox color

    float U = ((atan2(intersection.x, intersection.z) / PI) + 1.f) * .5f;
    float V = (asin(intersection.y) / PI) + .5f;

    int u = (*material).textureMapping.x * U;
    int v = (*material).textureMapping.y * V;

    if ((*material).textureMapping.x != 0)
        u %= (*material).textureMapping.x;
    if ((*material).textureMapping.y != 0)
        v %= (*material).textureMapping.y;
    const bool condition = u >= 0 && u < (*material).textureMapping.x &&
                           v >= 0 && v < (*material).textureMapping.y;
    if (condition)
    {
        int A = (v * (*material).textureMapping.x + u) *
                (*material).textureMapping.w;
        int B = (*material).textureMapping.x * (*material).textureMapping.y *
                (*material).textureMapping.w;
        int index = A % B;

        // Diffuse
        int i = (*material).textureOffset.x + index;
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

Sphere intersection
________________________________________________________________________________
*/
static bool sphereIntersection(const SceneInfo* sceneInfo,
                               CONST Primitive* sphere,
                               CONST Material* materials, const Ray* ray,
                               float4* intersection, float4* normal,
                               float* shadowIntensity)
{
    bool back = false;
    // solve the equation sphere-ray to find the intersections
    float4 O_C = (*ray).origin - (*sphere).p0;
    float4 dir = normalize((*ray).direction);

    float a = 2.f * dot(dir, dir);
    float b = 2.f * dot(O_C, dir);
    float c = dot(O_C, O_C) - ((*sphere).size.x * (*sphere).size.x);
    float d = b * b - 2.f * a * c;

    if (d <= 0.f || a == 0.f)
        return false;
    float r = sqrt(d);
    float t1 = (-b - r) / a;
    float t2 = (-b + r) / a;

    if (t1 <= (*sceneInfo).geometryEpsilon &&
        t2 <= (*sceneInfo).geometryEpsilon)
        return false; // both intersections are behind the ray origin

    float t = 0.f;
    if (t1 <= (*sceneInfo).geometryEpsilon)
    {
        back = true;
        t = t2;
    }
    else if (t2 <= (*sceneInfo).geometryEpsilon)
        t = t1;
    else
        t = (t1 < t2) ? t1 : t2;

    if (t < (*sceneInfo).geometryEpsilon)
        return false; // Too close to intersection
    (*intersection) = (*ray).origin + t * dir;

    if (materials[(*sphere).materialId].attributes.y == 0)
    {
        // Compute normal vector
        (*normal) = (*intersection) - (*sphere).p0;
    }
    else
    {
        // Procedural texture
        float4 newCenter;
        newCenter.x = (*sphere).p0.x +
                      0.008f * (*sphere).size.x *
                          cos((*sceneInfo).timestamp + (*intersection).x);
        newCenter.y = (*sphere).p0.y +
                      0.008f * (*sphere).size.y *
                          sin((*sceneInfo).timestamp + (*intersection).y);
        newCenter.z = (*sphere).p0.z +
                      0.008f * (*sphere).size.z *
                          sin(cos((*sceneInfo).timestamp + (*intersection).z));
        (*normal) = (*intersection) - newCenter;
    }
    (*normal) = normalize(*normal);
    if (back)
        (*normal) *= -1.f;

    // Shadow management
    r = dot(dir, (*normal));
    (*shadowIntensity) = (materials[(*sphere).materialId].transparency != 0.f)
                             ? (1.f - fabs(r))
                             : 1.f;

    return true;
}

static float4 project(const float4 A, const float4 B)
{
    return B * (dot(A, B) / dot(B, B));
}

/*
________________________________________________________________________________

Cylinder (*intersection)
________________________________________________________________________________
*/
static bool cylinderIntersection(const SceneInfo* sceneInfo,
                                 CONST Primitive* cylinder,
                                 CONST Material* materials, const Ray* ray,
                                 float4* intersection, float4* normal,
                                 float* shadowIntensity)
{
    float4 O_C = (*ray).origin - (*cylinder).p0;
    float4 dir = (*ray).direction;
    float4 n = cross(dir, (*cylinder).n1);
    float ln = length(n);

    // Parallel? (?)
    if (ln < (*sceneInfo).geometryEpsilon && ln > -(*sceneInfo).geometryEpsilon)
        return false;

    n = normalize(n);

    float d = fabs(dot(O_C, n));
    if (d > (*cylinder).size.y)
        return false;

    float4 O = cross(O_C, (*cylinder).n1);
    float t = -dot(O, n) / ln;
    if (t < 0.f)
        return false;

    O = normalize(cross(n, (*cylinder).n1));
    float s = fabs(sqrt((*cylinder).size.x * (*cylinder).size.x - d * d) /
                   dot(dir, O));

    float t1 = t - s;
    float t2 = t + s;

    // Calculate intersection point
    (*intersection) = (*ray).origin + t1 * dir;
    float4 HB1 = (*intersection) - (*cylinder).p0;
    float4 HB2 = (*intersection) - (*cylinder).p1;
    float scale1 = dot(HB1, (*cylinder).n1);
    float scale2 = dot(HB2, (*cylinder).n1);
    // Cylinder length
    if (scale1 < (*sceneInfo).geometryEpsilon ||
        scale2 > (*sceneInfo).geometryEpsilon)
    {
        (*intersection) = (*ray).origin + t2 * dir;
        HB1 = (*intersection) - (*cylinder).p0;
        HB2 = (*intersection) - (*cylinder).p1;
        scale1 = dot(HB1, (*cylinder).n1);
        scale2 = dot(HB2, (*cylinder).n1);
        // Cylinder length
        if (scale1 < (*sceneInfo).geometryEpsilon ||
            scale2 > (*sceneInfo).geometryEpsilon)
            return false;
        (*normal) = normalize(*normal);
    }

    float4 V = (*intersection) - (*cylinder).p2;
    (*normal) = V - project(V, (*cylinder).n1);
    (*normal) = normalize(*normal);

    // Shadow management
    (*shadowIntensity) = 1.f;
    return true;
}

/*
________________________________________________________________________________

Checkboard (*intersection)
________________________________________________________________________________
*/
static bool planeIntersection(const SceneInfo* sceneInfo,
                              CONST Primitive* primitive,
                              CONST Material* materials,
                              CONST BitmapBuffer* textures, const Ray* ray,
                              float4* intersection, float4* normal,
                              float* shadowIntensity, bool reverse)
{
    bool collision = false;
    float reverted = reverse ? -1.f : 1.f;
    switch ((*primitive).type)
    {
    case ptMagicCarpet:
    case ptCheckboard:
    {
        (*intersection).y = (*primitive).p0.y;
        float y = (*ray).origin.y - (*primitive).p0.y;
        if (reverted * (*ray).direction.y < 0.f &&
            reverted * (*ray).origin.y > reverted * (*primitive).p0.y)
        {
            (*normal).x = 0.f;
            (*normal).y = 1.f;
            (*normal).z = 0.f;
            (*intersection).x =
                (*ray).origin.x + y * (*ray).direction.x / -(*ray).direction.y;
            (*intersection).z =
                (*ray).origin.z + y * (*ray).direction.z / -(*ray).direction.y;
            collision = fabs((*intersection).x - (*primitive).p0.x) <
                            (*primitive).size.x &&
                        fabs((*intersection).z - (*primitive).p0.z) <
                            (*primitive).size.z;
        }
        break;
    }
    case ptXZPlane:
    {
        float y = (*ray).origin.y - (*primitive).p0.y;
        if (reverted * (*ray).direction.y < 0.f &&
            reverted * (*ray).origin.y > reverted * (*primitive).p0.y)
        {
            (*normal).x = 0.f;
            (*normal).y = 1.f;
            (*normal).z = 0.f;
            (*intersection).x =
                (*ray).origin.x + y * (*ray).direction.x / -(*ray).direction.y;
            (*intersection).y = (*primitive).p0.y;
            (*intersection).z =
                (*ray).origin.z + y * (*ray).direction.z / -(*ray).direction.y;
            collision = fabs((*intersection).x - (*primitive).p0.x) <
                            (*primitive).size.x &&
                        fabs((*intersection).z - (*primitive).p0.z) <
                            (*primitive).size.z;
        }
        const bool condition =
            !collision && reverted * (*ray).direction.y > 0.f &&
            reverted * (*ray).origin.y < reverted * (*primitive).p0.y;
        if (condition)
        {
            (*normal).x = 0.f;
            (*normal).y = -1.f;
            (*normal).z = 0.f;
            (*intersection).x =
                (*ray).origin.x + y * (*ray).direction.x / -(*ray).direction.y;
            (*intersection).y = (*primitive).p0.y;
            (*intersection).z =
                (*ray).origin.z + y * (*ray).direction.z / -(*ray).direction.y;
            collision = fabs((*intersection).x - (*primitive).p0.x) <
                            (*primitive).size.x &&
                        fabs((*intersection).z - (*primitive).p0.z) <
                            (*primitive).size.z;
        }
        break;
    }
    case ptYZPlane:
    {
        float x = (*ray).origin.x - (*primitive).p0.x;
        if (reverted * (*ray).direction.x < 0.f &&
            reverted * (*ray).origin.x > reverted * (*primitive).p0.x)
        {
            (*normal).x = 1.f;
            (*normal).y = 0.f;
            (*normal).z = 0.f;
            (*intersection).x = (*primitive).p0.x;
            (*intersection).y =
                (*ray).origin.y + x * (*ray).direction.y / -(*ray).direction.x;
            (*intersection).z =
                (*ray).origin.z + x * (*ray).direction.z / -(*ray).direction.x;
            collision = fabs((*intersection).y - (*primitive).p0.y) <
                            (*primitive).size.y &&
                        fabs((*intersection).z - (*primitive).p0.z) <
                            (*primitive).size.z;
        }
        const bool condition =
            !collision && reverted * (*ray).direction.x > 0.f &&
            reverted * (*ray).origin.x < reverted * (*primitive).p0.x;
        if (condition)
        {
            (*normal).x = -1.f;
            (*normal).y = 0.f;
            (*normal).z = 0.f;
            (*intersection).x = (*primitive).p0.x;
            (*intersection).y =
                (*ray).origin.y + x * (*ray).direction.y / -(*ray).direction.x;
            (*intersection).z =
                (*ray).origin.z + x * (*ray).direction.z / -(*ray).direction.x;
            collision = fabs((*intersection).y - (*primitive).p0.y) <
                            (*primitive).size.y &&
                        fabs((*intersection).z - (*primitive).p0.z) <
                            (*primitive).size.z;
        }
        break;
    }
    case ptXYPlane:
    {
        float z = (*ray).origin.z - (*primitive).p0.z;
        if (reverted * (*ray).direction.z < 0.f &&
            reverted * (*ray).origin.z > reverted * (*primitive).p0.z)
        {
            (*normal).x = 0.f;
            (*normal).y = 0.f;
            (*normal).z = 1.f;
            (*intersection).z = (*primitive).p0.z;
            (*intersection).x =
                (*ray).origin.x + z * (*ray).direction.x / -(*ray).direction.z;
            (*intersection).y =
                (*ray).origin.y + z * (*ray).direction.y / -(*ray).direction.z;
            collision = fabs((*intersection).x - (*primitive).p0.x) <
                            (*primitive).size.x &&
                        fabs((*intersection).y - (*primitive).p0.y) <
                            (*primitive).size.y;
        }
        const bool condition =
            !collision && reverted * (*ray).direction.z > 0.f &&
            reverted * (*ray).origin.z < reverted * (*primitive).p0.z;
        if (condition)
        {
            (*normal).x = 0.f;
            (*normal).y = 0.f;
            (*normal).z = -1.f;
            (*intersection).z = (*primitive).p0.z;
            (*intersection).x =
                (*ray).origin.x + z * (*ray).direction.x / -(*ray).direction.z;
            (*intersection).y =
                (*ray).origin.y + z * (*ray).direction.y / -(*ray).direction.z;
            collision = fabs((*intersection).x - (*primitive).p0.x) <
                            (*primitive).size.x &&
                        fabs((*intersection).y - (*primitive).p0.y) <
                            (*primitive).size.y;
        }
        break;
    }
    case ptCamera:
    {
        if (reverted * (*ray).direction.z < 0.f &&
            reverted * (*ray).origin.z > reverted * (*primitive).p0.z)
        {
            (*normal).x = 0.f;
            (*normal).y = 0.f;
            (*normal).z = 1.f;
            (*intersection).z = (*primitive).p0.z;
            float z = (*ray).origin.z - (*primitive).p0.z;
            (*intersection).x =
                (*ray).origin.x + z * (*ray).direction.x / -(*ray).direction.z;
            (*intersection).y =
                (*ray).origin.y + z * (*ray).direction.y / -(*ray).direction.z;
            collision = fabs((*intersection).x - (*primitive).p0.x) <
                            (*primitive).size.x &&
                        fabs((*intersection).y - (*primitive).p0.y) <
                            (*primitive).size.y;
        }
        break;
    }
    }

    if (collision)
    {
        // Shadow intensity
        (*shadowIntensity) = 1.f;

        float4 color = materials[(*primitive).materialId].color;
        if ((*primitive).type == ptCamera ||
            materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
        {
            // TODO?
            float4 specular = {0.f, 0.f, 0.f, 0.f};
            float4 attributes;
            float4 advancedAttributes;
            color = cubeMapping(sceneInfo, primitive, materials, textures,
                                intersection, normal, &specular, &attributes,
                                &advancedAttributes);
            (*shadowIntensity) = color.w;
        }

        if ((color.x + color.y + color.z) / 3.f >=
            (*sceneInfo).transparentColor)
            collision = false;
    }
    return collision;
}

/*
________________________________________________________________________________

Triangle intersection
________________________________________________________________________________
*/
static bool triangleIntersection(const SceneInfo* sceneInfo,
                                 CONST Primitive* triangle, const Ray* ray,
                                 float4* intersection, float4* normal,
                                 float4* areas, float* shadowIntensity,
                                 const bool processingShadows)
{
    // Reject rays using the barycentric coordinates of
    // the intersection point with respect to T.
    float4 E01 = (*triangle).p1 - (*triangle).p0;
    float4 E03 = (*triangle).p2 - (*triangle).p0;
    float4 P = cross((*ray).direction, E03);
    float det = dot(E01, P);

    if (fabs(det) < (*sceneInfo).geometryEpsilon)
        return false;

    float4 T = (*ray).origin - (*triangle).p0;
    float a = dot(T, P) / det;
    if (a < 0.f || a > 1.f)
        return false;

    float4 Q = cross(T, E01);
    float b = dot((*ray).direction, Q) / det;
    if (b < 0.f || b > 1.f)
        return false;

    // Reject rays using the barycentric coordinates of
    // the intersection point with respect to T'.
    if ((a + b) > 1.f)
    {
        float4 E23 = (*triangle).p0 - (*triangle).p1;
        float4 E21 = (*triangle).p1 - (*triangle).p1;
        float4 P_ = cross((*ray).direction, E21);
        float det_ = dot(E23, P_);
        if (fabs(det_) < (*sceneInfo).geometryEpsilon)
            return false;
        float4 T_ = (*ray).origin - (*triangle).p2;
        float a_ = dot(T_, P_) / det_;
        if (a_ < 0.f)
            return false;
        float4 Q_ = cross(T_, E23);
        float b_ = dot((*ray).direction, Q_) / det_;
        if (b_ < 0.f)
            return false;
    }

    // Compute the ray parameter of the intersection
    // point.
    float t = dot(E03, Q) / det;
    if (t < 0.f)
        return false;

    // Intersection
    (*intersection) = (*ray).origin + t * (*ray).direction;

    // Normal
    float4 v0 = ((*triangle).p0 - (*intersection));
    float4 v1 = ((*triangle).p1 - (*intersection));
    float4 v2 = ((*triangle).p2 - (*intersection));

    (*areas).x = 0.5f * length(cross(v1, v2));
    (*areas).y = 0.5f * length(cross(v0, v2));
    (*areas).z = 0.5f * length(cross(v0, v1));
    (*areas) = normalize(*areas);

    (*normal) = ((*triangle).n0 * (*areas).x + (*triangle).n1 * (*areas).y +
                 (*triangle).n2 * (*areas).z) /
                ((*areas).x + (*areas).y + (*areas).z);

    if ((*sceneInfo).doubleSidedTriangles)
    {
        // Double Sided triangles
        // Reject triangles with normal opposite to ray.
        float4 N = normalize((*ray).direction);
        if (processingShadows)
        {
            if (dot(N, (*normal)) <= 0.f)
                return false;
        }
        else
        {
            if (dot(N, (*normal)) >= 0.f)
                return false;
        }
    }

    float4 dir = normalize((*ray).direction);
    float r = dot(dir, (*normal));

    if (r > 0.f)
        (*normal) *= -1.f;

    // Shadow management
    (*shadowIntensity) = 1.f;
    return true;
}

/*
________________________________________________________________________________

(*intersection) Shader
________________________________________________________________________________
*/
static float4 intersectionShader(const SceneInfo* sceneInfo,
                                 CONST Primitive* primitive,
                                 CONST Material* materials,
                                 CONST BitmapBuffer* textures,
                                 float4* intersection, const float4 areas,
                                 float4* normal, float4* specular,
                                 float4* attributes, float4* advancedAttributes)
{
    float4 colorAtIntersection = materials[(*primitive).materialId].color;
    colorAtIntersection.w =
        0.f; // w attribute is used to dtermine light intensity of the material

    if ((*sceneInfo).extendedGeometry)
    {
        switch ((*primitive).type)
        {
        case ptCylinder:
        {
            if (materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
                colorAtIntersection =
                    sphereUVMapping(primitive, materials, textures,
                                    intersection, normal, specular, attributes,
                                    advancedAttributes);
            break;
        }
        case ptEnvironment:
        case ptSphere:
        case ptEllipsoid:
        {
            if (materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
                colorAtIntersection =
                    sphereUVMapping(primitive, materials, textures,
                                    intersection, normal, specular, attributes,
                                    advancedAttributes);
            break;
        }
        case ptCheckboard:
        {
            if (materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
                colorAtIntersection =
                    cubeMapping(sceneInfo, primitive, materials, textures,
                                intersection, normal, specular, attributes,
                                advancedAttributes);
            else
            {
                int x = (*sceneInfo).viewDistance +
                        (((*intersection).x - (*primitive).p0.x) /
                         (*primitive).size.x);
                int z = (*sceneInfo).viewDistance +
                        (((*intersection).z - (*primitive).p0.z) /
                         (*primitive).size.x);
                if (x % 2 == 0)
                {
                    if (z % 2 == 0)
                    {
                        colorAtIntersection.x = 1.f - colorAtIntersection.x;
                        colorAtIntersection.y = 1.f - colorAtIntersection.y;
                        colorAtIntersection.z = 1.f - colorAtIntersection.z;
                    }
                }
                else
                {
                    if (z % 2 != 0)
                    {
                        colorAtIntersection.x = 1.f - colorAtIntersection.x;
                        colorAtIntersection.y = 1.f - colorAtIntersection.y;
                        colorAtIntersection.z = 1.f - colorAtIntersection.z;
                    }
                }
            }
            break;
        }
        case ptXYPlane:
        case ptYZPlane:
        case ptXZPlane:
        case ptCamera:
        {
            if (materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
                colorAtIntersection =
                    cubeMapping(sceneInfo, primitive, materials, textures,
                                intersection, normal, specular, attributes,
                                advancedAttributes);
            break;
        }
        case ptTriangle:
        {
            if (materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
                colorAtIntersection =
                    triangleUVMapping(sceneInfo, primitive, materials, textures,
                                      intersection, areas, normal, specular,
                                      attributes, advancedAttributes);
            break;
        }
        }
    }
    else if (materials[(*primitive).materialId].textureIds.x != TEXTURE_NONE)
        colorAtIntersection =
            triangleUVMapping(sceneInfo, primitive, materials, textures,
                              intersection, areas, normal, specular, attributes,
                              advancedAttributes);
    return colorAtIntersection;
}

/*
________________________________________________________________________________

Shadows computation
We do not consider the object from which the ray is launched...
This object cannot shadow itself !

We now have to find the (*intersection) between the considered object and the
ray which origin is the considered 3D float4 and which direction is defined by
the light source center.
@return 1.f when pixel is in the shades
________________________________________________________________________________
*/
static float processShadows(
    const SceneInfo* sceneInfo, CONST BoundingBox* boudingBoxes,
    const int nbActiveBoxes, CONST Primitive* primitives,
    CONST Material* materials, CONST BitmapBuffer* textures,
    const int nbPrimitives, const float4 lampCenter, const float4 origin,
    const int objectId, const int iteration, float4* color)
{
    float result = 0.f;
    int cptBoxes = 0;
    (*color).x = 0.f;
    (*color).y = 0.f;
    (*color).z = 0.f;
    Ray r;
    r.direction = lampCenter - origin;
    r.origin = origin + normalize(r.direction) * (*sceneInfo).rayEpsilon;
    computeRayAttributes(&r);
    float minDistance = (iteration < 2)
                            ? (*sceneInfo).viewDistance
                            : (*sceneInfo).viewDistance / (iteration + 1);

    while (result < (*sceneInfo).shadowIntensity && cptBoxes < nbActiveBoxes)
    {
        CONST BoundingBox* box = &boudingBoxes[cptBoxes];
        if (boxIntersection(box, &r, 0.05f, minDistance))
        {
            int cptPrimitives = 0;
            while (result < (*sceneInfo).shadowIntensity &&
                   cptPrimitives < (*box).nbPrimitives)
            {
                float4 intersection = {0.f, 0.f, 0.f, 0.f};
                float4 normal = {0.f, 0.f, 0.f, 0.f};
                float4 areas = {0.f, 0.f, 0.f, 0.f};
                float shadowIntensity = 0.f;

                CONST Primitive* primitive =
                    &primitives[(*box).startIndex + cptPrimitives];
                if ((*primitive).index != objectId &&
                    materials[(*primitive).materialId].attributes.x == 0)
                {
                    bool hit = false;
                    if ((*sceneInfo).extendedGeometry)
                    {
                        switch ((*primitive).type)
                        {
                        case ptSphere:
                            hit =
                                sphereIntersection(sceneInfo, primitive,
                                                   materials, &r, &intersection,
                                                   &normal, &shadowIntensity);
                            break;
                        case ptCylinder:
                            hit = cylinderIntersection(sceneInfo, primitive,
                                                       materials, &r,
                                                       &intersection, &normal,
                                                       &shadowIntensity);
                            break;
                        case ptCamera:
                            hit = false;
                            break;
                        case ptEllipsoid:
                            hit = ellipsoidIntersection(sceneInfo, primitive,
                                                        materials, &r,
                                                        &intersection, &normal,
                                                        &shadowIntensity);
                            break;
                        case ptTriangle:
                            hit = triangleIntersection(sceneInfo, primitive, &r,
                                                       &intersection, &normal,
                                                       &areas, &shadowIntensity,
                                                       true);
                            break;
                        default:
                            hit = planeIntersection(sceneInfo, primitive,
                                                    materials, textures, &r,
                                                    &intersection, &normal,
                                                    &shadowIntensity, false);
                            break;
                        }
                    }
                    else
                    {
                        hit =
                            triangleIntersection(sceneInfo, primitive, &r,
                                                 &intersection, &normal, &areas,
                                                 &shadowIntensity, true);
                    }
                    if (hit)
                    {
                        float4 O_I = intersection - r.origin;
                        float4 O_L = r.direction;
                        float l = length(O_I);
                        if (l > (*sceneInfo).geometryEpsilon && l < length(O_L))
                        {
                            float ratio =
                                shadowIntensity * (*sceneInfo).shadowIntensity;
                            if (materials[(*primitive).materialId]
                                    .transparency != 0.f)
                            {
                                // Shadow color
                                O_L = normalize(O_L);
                                float a = fabs(dot(O_L, normal));
                                float r =
                                    (materials[(*primitive).materialId]
                                         .transparency == 0.f)
                                        ? 1.f
                                        : (1.f -
                                           0.8f * materials[(*primitive)
                                                                .materialId]
                                                      .transparency);
                                ratio *= r * a;
                                (*color).x +=
                                    ratio *
                                    (0.3f -
                                     0.3f * materials[(*primitive).materialId]
                                                .color.x);
                                (*color).y +=
                                    ratio *
                                    (0.3f -
                                     0.3f * materials[(*primitive).materialId]
                                                .color.y);
                                (*color).z +=
                                    ratio *
                                    (0.3f -
                                     0.3f * materials[(*primitive).materialId]
                                                .color.z);
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
            cptBoxes += (*box).indexForNextBox.x;
        }
    }
    result = max(0.f, min(result, (*sceneInfo).shadowIntensity));
    return result;
}

/*
________________________________________________________________________________

Primitive shader
________________________________________________________________________________
*/
static float4 primitiveShader(
    const int index, const SceneInfo* sceneInfo,
    const PostProcessingInfo* postProcessingInfo,
    CONST BoundingBox* boundingBoxes, const int nbActiveBoxes,
    CONST Primitive* primitives, const int nbActivePrimitives,
    CONST LightInformation* lightInformation, const int lightInformationSize,
    const int nbActiveLamps, CONST Material* materials,
    CONST BitmapBuffer* textures, CONST RandomBuffer* randoms,
    const float4 origin, float4* normal, const int objectId,
    float4* intersection, const float4 areas, float4* closestColor,
    const int iteration, float4* refractionFromColor, float* shadowIntensity,
    float4* totalBlinn, float4* attributes)
{
    CONST Primitive* primitive = &(primitives[objectId]);
    CONST Material* material = &materials[(*primitive).materialId];
    float4 lampsColor = {0.f, 0.f, 0.f, 0.f};

    // Lamp Impact
    (*shadowIntensity) = 0.f;

    // Bump
    float4 bumpNormal = {0.f, 0.f, 0.f, 0.f};
    float4 advancedAttributes = {0.f, 0.f, 0.f, 0.f};

    // Specular
    float4 specular;
    specular.x = (*material).specular.x;
    specular.y = (*material).specular.y;
    specular.z = (*material).specular.z;

    // Intersection color
    float4 intersectionColor =
        intersectionShader(sceneInfo, primitive, materials, textures,
                           intersection, areas, &bumpNormal, &specular,
                           attributes, &advancedAttributes);
    (*normal) += bumpNormal;
    (*normal) = normalize((*normal));

    const bool condition = (*sceneInfo).graphicsLevel == glNoShading ||
                           (*material).innerIllumination.x != 0.f ||
                           (*material).attributes.z != 0;
    if (condition)
    {
        // Wireframe returns constant color
        return intersectionColor;
    }

    if ((*sceneInfo).graphicsLevel > glNoShading)
    {
        (*closestColor) *= (*material).innerIllumination.x;
        int C = 1; // (lightInformationSize>1) ? 2 : 1;
        for (int c = 0; c < C; ++c)
        {
            int cptLamp =
                ((*sceneInfo).pathTracingIteration >= NB_MAX_ITERATIONS)
                    ? ((*sceneInfo).pathTracingIteration %
                           lightInformationSize +
                       C - 1)
                    : 0;

            if (lightInformation[cptLamp].primitiveId != (*primitive).index)
            {
                // randomize lamp center
                float4 center = lightInformation[cptLamp].location;

                int t =
                    (index + (*sceneInfo).timestamp) % (MAX_BITMAP_SIZE - 3);
                CONST Material* m =
                    &materials[lightInformation[cptLamp].materialId];
                const bool condition =
                    (*sceneInfo).pathTracingIteration >= NB_MAX_ITERATIONS &&
                    lightInformation[cptLamp].primitiveId >= 0 &&
                    lightInformation[cptLamp].primitiveId < nbActivePrimitives;
                if (condition)
                {
                    float a = (*m).innerIllumination.y * 10.f *
                              (*sceneInfo).pathTracingIteration /
                              (float)((*sceneInfo).maxPathTracingIterations);
                    center.x += randoms[t] * a;
                    center.y += randoms[t + 1] * a;
                    center.z += randoms[t + 2] * a;
                }

                float4 lightRay = center - (*intersection);
                float lightRayLength = length(lightRay);
                if (lightRayLength < (*m).innerIllumination.z)
                {
                    float4 shadowColor = {0.f, 0.f, 0.f, 0.f};

                    // --------------------------------------------------------------------------------
                    // Lambert
                    // --------------------------------------------------------------------------------
                    lightRay = normalize(lightRay);
                    float lambert = dot((*normal), lightRay);

                    const bool condition =
                        lambert > 0.f &&
                        (*sceneInfo).graphicsLevel >
                            glReflectionsAndRefractions &&
                        iteration < 4 && // No need to process shadows after 4
                                         // generations of rays... cannot be
                                         // seen anyway.
                        (*material).innerIllumination.x == 0.f;
                    if (condition)
                        (*shadowIntensity) = processShadows(
                            sceneInfo, boundingBoxes, nbActiveBoxes, primitives,
                            materials, textures, nbActivePrimitives, center,
                            (*intersection),
                            lightInformation[cptLamp].primitiveId, iteration,
                            &shadowColor);

                    if ((*sceneInfo).graphicsLevel > glNoShading)
                    {
                        float photonEnergy =
                            sqrt(lightRayLength / (*m).innerIllumination.z);
                        photonEnergy =
                            (photonEnergy > 1.f) ? 1.f : photonEnergy;
                        photonEnergy =
                            (photonEnergy < 0.f) ? 0.f : photonEnergy;

                        // Transparent materials are lighted on both sides but
                        // the amount of light received by the dark side depends
                        // on the transparency rate.
                        lambert *= (lambert < 0.f)
                                       ? -materials[(*primitive).materialId]
                                              .transparency
                                       : 1.f;

                        if (lightInformation[cptLamp].materialId !=
                            MATERIAL_NONE)
                        {
                            CONST Material* m =
                                &materials[lightInformation[cptLamp]
                                               .materialId];
                            lambert *=
                                (*m).innerIllumination.x; // Lamp illumination
                        }
                        else
                            lambert *= lightInformation[cptLamp].color.w;

                        if ((*material).innerIllumination.w != 0.f)
                            // Randomize lamp intensity depending on material
                            // noise, for more realistic rendering
                            lambert *=
                                (1.f + randoms[t] *
                                           (*material).innerIllumination.w *
                                           100.f);

                        lambert *= (1.f - (*shadowIntensity));
                        lambert += (*sceneInfo).backgroundColor.w;
                        lambert *= (1.f - photonEnergy);

                        // Lighted object, not in the shades
                        lampsColor +=
                            lambert * lightInformation[cptLamp].color -
                            shadowColor;

                        const bool condition =
                            (*sceneInfo).graphicsLevel > glPhong &&
                            (*shadowIntensity) < (*sceneInfo).shadowIntensity;

                        if (condition)
                        {
                            // --------------------------------------------------------------------------------
                            // Blinn - Phong
                            // --------------------------------------------------------------------------------
                            float4 viewRay =
                                normalize((*intersection) - origin);
                            float4 blinnDir = lightRay - viewRay;
                            const float temp = sqrt(dot(blinnDir, blinnDir));
                            if (temp != 0.f)
                            {
                                // Specular reflection
                                blinnDir = (1.f / temp) * blinnDir;
                                float blinnTerm = dot(blinnDir, (*normal));
                                blinnTerm = (blinnTerm < 0.f) ? 0.f : blinnTerm;

                                blinnTerm =
                                    specular.x * pow(blinnTerm, specular.y);
                                blinnTerm *= (1.f - photonEnergy);
                                (*totalBlinn).x +=
                                    lightInformation[cptLamp].color.x *
                                    lightInformation[cptLamp].color.w *
                                    blinnTerm;
                                (*totalBlinn).y +=
                                    lightInformation[cptLamp].color.y *
                                    lightInformation[cptLamp].color.w *
                                    blinnTerm;
                                (*totalBlinn).z +=
                                    lightInformation[cptLamp].color.z *
                                    lightInformation[cptLamp].color.w *
                                    blinnTerm;

                                // Get transparency from specular map
                                (*totalBlinn).w = specular.z;
                            }
                        }
                    }
                }
            }

            // Light impact on material
            (*closestColor) += intersectionColor * lampsColor;

            (*refractionFromColor) =
                intersectionColor; // Refraction depending on color;
            saturateVector(totalBlinn);
        }
    }

    // Ambient occlusion
    if ((*material).advancedTextureIds.z != TEXTURE_NONE)
        (*closestColor) *= advancedAttributes.x;

    // Saturate color
    saturateVector(closestColor);

    return (*closestColor);
}

/*
________________________________________________________________________________

Intersections with primitives
________________________________________________________________________________
*/
inline bool intersectionWithPrimitives(
    const SceneInfo* sceneInfo, CONST BoundingBox* boundingBoxes,
    const int nbActiveBoxes, CONST Primitive* primitives,
    const int nbActivePrimitives, CONST Material* materials,
    CONST BitmapBuffer* textures, const Ray* ray, const int iteration,
    int* closestPrimitive, float4* closestIntersection, float4* closestNormal,
    float4* closestAreas, float4* colorBox, const int currentMaterialId)
{
    bool intersections = false;
    float minDistance = (iteration < 2)
                            ? (*sceneInfo).viewDistance
                            : (*sceneInfo).viewDistance / (iteration + 1);

    Ray r;
    r.origin = (*ray).origin;
    r.direction = (*ray).direction - (*ray).origin;
    computeRayAttributes(&r);

    float4 intersection; // = {0.f, 0.f, 0.f, 0.f};
    float4 normal;       // = {0.f, 0.f, 0.f, 0.f};
    bool i = false;
    float shadowIntensity = 0.f;

    int cptBoxes = 0;
    while (cptBoxes < nbActiveBoxes)
    {
        CONST BoundingBox* box = &boundingBoxes[cptBoxes];
        if (boxIntersection(box, &r, 0.f, minDistance))
        {
            // Intersection with Box
            if ((*sceneInfo).renderBoxes == 0)
            {
                // Intersection with primitive within boxes
                for (int cptPrimitives = 0; cptPrimitives < (*box).nbPrimitives;
                     ++cptPrimitives)
                {
                    CONST Primitive* primitive =
                        &primitives[(*box).startIndex + cptPrimitives];
                    CONST Material* material =
                        &materials[(*primitive).materialId];
                    const bool condition =
                        (*material).attributes.x == 0 ||
                        ((*material).attributes.x == 1 &&
                         currentMaterialId != (*primitive).materialId);
                    if (condition) // !!!! TEST SHALL BE REMOVED TO INCREASE
                                   // TRANSPARENCY QUALITY !!!
                    {
                        float4 areas = {0.f, 0.f, 0.f, 0.f};
                        i = false;
                        if ((*sceneInfo).extendedGeometry)
                        {
                            switch ((*primitive).type)
                            {
                            case ptEnvironment:
                            case ptSphere:
                                i = sphereIntersection(sceneInfo, primitive,
                                                       materials, &r,
                                                       &intersection, &normal,
                                                       &shadowIntensity);
                                break;
                            case ptCylinder:
                                i = cylinderIntersection(sceneInfo, primitive,
                                                         materials, &r,
                                                         &intersection, &normal,
                                                         &shadowIntensity);
                                break;
                            case ptEllipsoid:
                                i = ellipsoidIntersection(sceneInfo, primitive,
                                                          materials, &r,
                                                          &intersection,
                                                          &normal,
                                                          &shadowIntensity);
                                break;
                            case ptTriangle:
                                i = triangleIntersection(sceneInfo, primitive,
                                                         &r, &intersection,
                                                         &normal, &areas,
                                                         &shadowIntensity,
                                                         false);
                                break;
                            default:
                                i = planeIntersection(sceneInfo, primitive,
                                                      materials, textures, &r,
                                                      &intersection, &normal,
                                                      &shadowIntensity, false);
                                break;
                            }
                        }
                        else
                            i = triangleIntersection(sceneInfo, primitive, &r,
                                                     &intersection, &normal,
                                                     &areas, &shadowIntensity,
                                                     false);

                        const float distance = length(intersection - r.origin);
                        const bool condition =
                            i && distance > (*sceneInfo).geometryEpsilon &&
                            distance < minDistance;
                        if (condition)
                        {
                            // Only keep intersection with the closest object
                            minDistance = distance;
                            (*closestPrimitive) =
                                (*box).startIndex + cptPrimitives;
                            (*closestIntersection) = intersection;
                            (*closestNormal) = normal;
                            (*closestAreas) = areas;
                            intersections = true;
                        }
                    }
                }
            }
            else
                (*colorBox) +=
                    materials[(*box).startIndex % NB_MAX_MATERIALS].color /
                    50.f;
            ++cptBoxes;
        }
        else
            cptBoxes += (*box).indexForNextBox.x;
    }
    return intersections;
}

/*
________________________________________________________________________________

Intersections with primitives
________________________________________________________________________________
*/
inline float4 intersectionsWithPrimitives(
    const int index, const SceneInfo* sceneInfo,
    CONST BoundingBox* boundingBoxes, const int nbActiveBoxes,
    CONST Primitive* primitives, const int nbActivePrimitives,
    CONST Material* materials, CONST BitmapBuffer* textures,
    CONST LightInformation* lightInformation, const int lightInformationSize,
    const int nbActiveLamps, CONST RandomBuffer* randoms,
    const PostProcessingInfo* postProcessingInfo, const Ray* ray)
{
    Ray r;
    r.origin = (*ray).origin;
    r.direction = (*ray).direction - (*ray).origin;
    computeRayAttributes(&r);

    float4 intersection = (*ray).origin;
    float4 normal = {0.f, 0.f, 0.f, 0.f};
    bool i = false;
    float shadowIntensity = 0.f;

    float4 colors[MAXDEPTH];
    for (int i = 0; i < MAXDEPTH; ++i)
    {
        colors[i].x = 0.f;
        colors[i].y = 0.f;
        colors[i].z = 0.f;
        colors[i].w = (*sceneInfo).viewDistance;
    }
    // bool normals[MAXDEPTH];
    // memset(&normals[0],0,sizeof(bool)*MAXDEPTH);

    int cptBoxes = 0;
    while (cptBoxes < nbActiveBoxes)
    {
        CONST BoundingBox* box = &boundingBoxes[cptBoxes];
        if (boxIntersection(box, &r, 0.f, (*sceneInfo).viewDistance))
        {
            // Intersection with primitive within boxes
            for (int cptPrimitives = 0; cptPrimitives < (*box).nbPrimitives;
                 ++cptPrimitives)
            {
                i = false;
                CONST Primitive* primitive =
                    &primitives[(*box).startIndex + cptPrimitives];
                CONST Material* material = &materials[(*primitive).materialId];
                float4 areas = {0.f, 0.f, 0.f, 0.f};
                if ((*sceneInfo).extendedGeometry)
                {
                    switch ((*primitive).type)
                    {
                    case ptEnvironment:
                    case ptSphere:
                        i = sphereIntersection(sceneInfo, primitive, materials,
                                               &r, &intersection, &normal,
                                               &shadowIntensity);
                        break;
                    case ptCylinder:
                        i = cylinderIntersection(sceneInfo, primitive,
                                                 materials, &r, &intersection,
                                                 &normal, &shadowIntensity);
                        break;
                    case ptEllipsoid:
                        i = ellipsoidIntersection(sceneInfo, primitive,
                                                  materials, &r, &intersection,
                                                  &normal, &shadowIntensity);
                        break;
                    case ptTriangle:
                        i = triangleIntersection(sceneInfo, primitive, &r,
                                                 &intersection, &normal, &areas,
                                                 &shadowIntensity, false);
                        break;
                    default:
                        i = planeIntersection(sceneInfo, primitive, materials,
                                              textures, &r, &intersection,
                                              &normal, &shadowIntensity, false);
                        break;
                    }
                }
                else
                {
                    i = triangleIntersection(sceneInfo, primitive, &r,
                                             &intersection, &normal, &areas,
                                             &shadowIntensity, false);
                }
                if (i)
                {
                    float dist = length(intersection - r.origin);
                    // if( dist>(*postProcessingInfo).param1 )
                    {
                        float4 color = (*material).color;
                        if (false && (*sceneInfo).graphicsLevel != glNoShading)
                        {
                            color *= (1.f - (*material).transparency);
                            float4 attributes;
                            attributes.x = (*material).reflection;
                            attributes.y = (*material).transparency;
                            attributes.z = (*material).refraction;
                            attributes.w = (*material).opacity;
                            float4 rBlinn = {0.f, 0.f, 0.f, 0.f};
                            float4 refractionFromColor;
                            float4 closestColor = (*material).color;
                            shadowIntensity = 0.f;
                            color = primitiveShader(
                                index, sceneInfo, postProcessingInfo,
                                boundingBoxes, nbActiveBoxes, primitives,
                                nbActivePrimitives, lightInformation,
                                lightInformationSize, nbActiveLamps, materials,
                                textures, randoms, r.origin, &normal,
                                (*box).startIndex + cptPrimitives,
                                &intersection, areas, &closestColor, 0,
                                &refractionFromColor, &shadowIntensity, &rBlinn,
                                &attributes);
                        }
                        for (int i = 0; i < MAXDEPTH; ++i)
                        {
                            if (dist < colors[i].w)
                            {
                                for (int j = MAXDEPTH - 1; j >= i; --j)
                                {
                                    colors[j + 1] = colors[j];
                                    // normals[j+1]=normals[j];
                                    // float
                                    // a=dot(normalize(r.direction-r.origin),normal);
                                    colors[j] = color; // *(fabs(a));
                                    colors[j].w = dist;
                                    // normals[j] = (a>=0.f);
                                }
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
            cptBoxes += (*box).indexForNextBox.x;
        }
    }

    float4 color = (*sceneInfo).backgroundColor;

    float s = 200.f;
    int C = 0;
    float T = colors[C].w;
    float a = 5 / s;
    for (float t = 0; C < MAXDEPTH && t < (*sceneInfo).viewDistance;
         t += (*sceneInfo).viewDistance / s)
    {
        if (t < T)
        {
            if (C % 2 == 0)
            {
                color.x -= a;
                color.y -= a;
                color.z -= a;
            }
            else
            {
                color.x -= 5.f * a;
                color.y -= 5.f * a;
                color.z -= 5.f * a;
            }
        }
        else
        {
            color.x += 5.f * colors[C].x;
            color.y += 5.f * colors[C].y;
            color.z += 5.f * colors[C].z;
            ++C;
            T = colors[C].w;
        }
    }

    color = normalize(color);
    color.w = colors[0].w;
    return color;
}

inline float4 launchVolumeRendering(
    const int index, CONST BoundingBox* boundingBoxes, const int nbActiveBoxes,
    CONST Primitive* primitives, const int nbActivePrimitives,
    CONST LightInformation* lightInformation, const int lightInformationSize,
    const int nbActiveLamps, CONST Material* materials,
    CONST BitmapBuffer* textures, CONST RandomBuffer* randoms, const Ray* ray,
    const SceneInfo* sceneInfo, const PostProcessingInfo* postProcessingInfo,
    float* depthOfField, CONST PrimitiveXYIdBuffer* primitiveXYId)
{
    (*primitiveXYId).x = -1;
    (*primitiveXYId).y = 1;
    (*primitiveXYId).z = 0;
    float4 intersectionColor = intersectionsWithPrimitives(
        index, sceneInfo, boundingBoxes, nbActiveBoxes, primitives,
        nbActivePrimitives, materials, textures, lightInformation,
        lightInformationSize, nbActiveLamps, randoms, postProcessingInfo, ray);
    return intersectionColor;
}

/*
________________________________________________________________________________

Calculate the reflected vector
We now have to know the colour of this (*intersection)
Color_from_object will compute the amount of light received by the
(*intersection) float4 and  will also compute the shadows.
The resulted color is stored in result.
The first parameter is the closest object to the (*intersection) (following
the ray). It can  be considered as a light source if its inner light rate
is > 0.
________________________________________________________________________________
*/
inline float4 launchRayTracing(
    const int index, CONST BoundingBox* boundingBoxes, const int nbActiveBoxes,
    CONST Primitive* primitives, const int nbActivePrimitives,
    CONST LightInformation* lightInformation, const int lightInformationSize,
    const int nbActiveLamps, CONST Material* materials,
    CONST BitmapBuffer* textures, CONST RandomBuffer* randoms, const Ray* ray,
    const SceneInfo* sceneInfo, const PostProcessingInfo* postProcessingInfo,
    float* depthOfField, CONST PrimitiveXYIdBuffer* primitiveXYId)
{
    float4 intersectionColor = {0.f, 0.f, 0.f, 0.f};
    float4 closestIntersection = {0.f, 0.f, 0.f, 0.f};
    float4 firstIntersection = {0.f, 0.f, 0.f, 0.f};
    float4 normal = {0.f, 0.f, 0.f, 0.f};
    float4 closestColor = {0.f, 0.f, 0.f, 0.f};
    int closestPrimitive = 0;
    bool carryon = true;
    Ray rayOrigin = (*ray);
    float initialRefraction = 1.f;
    int iteration = 0;
    (*primitiveXYId).x = -1;
    (*primitiveXYId).z = 0;
    int currentMaterialId = -2;

    // TODO
    float colorContributions[NB_MAX_ITERATIONS];
    float4 colors[NB_MAX_ITERATIONS];
    for (int i = 0; i < NB_MAX_ITERATIONS; ++i)
    {
        colorContributions[i] = 0.f;
        colors[i].x = 0.f;
        colors[i].y = 0.f;
        colors[i].z = 0.f;
        colors[i].w = 0.f;
    }

    float4 recursiveBlinn = {0.f, 0.f, 0.f, 0.f};

    // Variable declarations
    float shadowIntensity = 0.f;
    float4 refractionFromColor;
    float4 reflectedTarget;
    float4 colorBox = {0.f, 0.f, 0.f, 0.f};
    float4 latestIntersection = (*ray).origin;
    float rayLength = 0.f;

    // Reflected rays
    int reflectedRays = -1;
    Ray reflectedRay;
    float reflectedRatio;

    // Global illumination
    Ray pathTracingRay;
    float pathTracingRatio = 0.f;
    float4 pathTracingColor = {0.f, 0.f, 0.f, 0.f};
    bool processGI = false;

    float4 rBlinn = {0.f, 0.f, 0.f, 0.f};
    int currentMaxIteration =
        ((*sceneInfo).graphicsLevel < glReflectionsAndRefractions)
            ? 1
            : (*sceneInfo).nbRayIterations + (*sceneInfo).pathTracingIteration;
    currentMaxIteration = (currentMaxIteration > NB_MAX_ITERATIONS)
                              ? NB_MAX_ITERATIONS
                              : currentMaxIteration;

    while (iteration < currentMaxIteration &&
           rayLength < (*sceneInfo).viewDistance && carryon)
    {
        float4 areas = {0.f, 0.f, 0.f, 0.f};
        // If no intersection with lamps detected. Now compute intersection with
        // Primitives
        if (carryon)
        {
            carryon = intersectionWithPrimitives(
                sceneInfo, boundingBoxes, nbActiveBoxes, primitives,
                nbActivePrimitives, materials, textures, &rayOrigin, iteration,
                &closestPrimitive, &closestIntersection, &normal, &areas,
                &colorBox, currentMaterialId);
        }

        if (carryon)
        {
            currentMaterialId = primitives[closestPrimitive].materialId;

            float4 attributes;
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

                if ((*sceneInfo).advancedIllumination != aiNone)
                {
                    // Global illumination
                    const int t =
                        (index + (*sceneInfo).timestamp) %
                        ((*sceneInfo).size.x * (*sceneInfo).size.y - 3);
                    pathTracingRay.origin =
                        closestIntersection + normal * (*sceneInfo).rayEpsilon;
                    pathTracingRay.direction.x = randoms[t];
                    pathTracingRay.direction.y = randoms[t + 1];
                    pathTracingRay.direction.z = randoms[t + 2];
                    pathTracingRay.direction =
                        normalize(pathTracingRay.direction);

                    const float cos_theta =
                        dot(normalize(pathTracingRay.direction), normal);
                    if (cos_theta < 0.f)
                        pathTracingRay.direction = -pathTracingRay.direction;
                    pathTracingRay.direction += closestIntersection;
                    pathTracingRatio = (1.f - attributes.y) * fabs(cos_theta);
                    processGI = true;
                }

                // Primitive ID for current pixel
                (*primitiveXYId).x = primitives[closestPrimitive].index;
            }

            // Get object color
            rBlinn.w = attributes.y;
            colors[iteration] =
                primitiveShader(index, sceneInfo, postProcessingInfo,
                                boundingBoxes, nbActiveBoxes, primitives,
                                nbActivePrimitives, lightInformation,
                                lightInformationSize, nbActiveLamps, materials,
                                textures, randoms, rayOrigin.origin, &normal,
                                closestPrimitive, &closestIntersection, areas,
                                &closestColor, iteration, &refractionFromColor,
                                &shadowIntensity, &rBlinn, &attributes);

            // Primitive illumination
            float colorLight =
                colors[iteration].x + colors[iteration].y + colors[iteration].z;
            (*primitiveXYId).z +=
                (colorLight > (*sceneInfo).transparentColor) ? 16 : 0;

            float segmentLength =
                length(closestIntersection - latestIntersection);
            latestIntersection = closestIntersection;
            // ----------
            // Refraction
            // ----------
            float transparency = attributes.y;
            float a = 0.f;
            if (transparency != 0.f) // Transparency
            {
                float refraction = attributes.z;

                // Back of the object? If so, reset refraction to 1.f (air)
                if (initialRefraction == refraction)
                {
                    // Opacity
                    refraction = 1.f;
                    float length =
                        segmentLength * (attributes.w * (1.f - transparency));
                    rayLength += length;
                    rayLength = (rayLength > (*sceneInfo).viewDistance)
                                    ? (*sceneInfo).viewDistance
                                    : rayLength;
                    a = (rayLength / (*sceneInfo).viewDistance);
                    colors[iteration].x -= a;
                    colors[iteration].y -= a;
                    colors[iteration].z -= a;
                }

                // Actual refraction
                float4 O_E = normalize(closestIntersection - rayOrigin.origin);
                vectorRefraction(&reflectedTarget, O_E, refraction, normal,
                                 initialRefraction);

                colorContributions[iteration] = transparency - a;

                // Prepare next ray
                initialRefraction = refraction;

                if (reflectedRays == -1 && attributes.x != 0.f) // Reflection
                {
                    vectorReflection(reflectedRay.direction, O_E, normal);
                    reflectedRay.origin =
                        closestIntersection +
                        reflectedRay.direction * (*sceneInfo).rayEpsilon;
                    reflectedRay.direction =
                        closestIntersection + reflectedRay.direction;
                    reflectedRatio = attributes.x;
                    reflectedRays = iteration;
                }
            }
            else
            {
                rayLength += segmentLength;
                if (attributes.x != 0.f) // Reflection
                {
                    float4 O_E =
                        normalize(closestIntersection - rayOrigin.origin);
                    vectorReflection(reflectedTarget, O_E, normal);
                    colorContributions[iteration] = attributes.x;
                }
                else
                {
                    // No more intersections with primitives -> skybox
                    carryon = false;
                    colorContributions[iteration] = 1.f;
                }
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
                closestIntersection + reflectedTarget * (*sceneInfo).rayEpsilon;
            rayOrigin.direction = closestIntersection + reflectedTarget;

            // Noise management
            if ((*sceneInfo).pathTracingIteration != 0 &&
                materials[primitives[closestPrimitive].materialId].color.w !=
                    0.f)
            {
                // Randomize view
                float ratio =
                    materials[primitives[closestPrimitive].materialId].color.w;
                ratio *= (attributes.y == 0.f) ? 1000.f : 1.f;
                int rindex =
                    (index + (*sceneInfo).timestamp) % (MAX_BITMAP_SIZE - 3);
                rayOrigin.direction.x += randoms[rindex] * ratio;
                rayOrigin.direction.y += randoms[rindex + 1] * ratio;
                rayOrigin.direction.z += randoms[rindex + 2] * ratio;
            }
        }
        else
        {
            // Background
            if ((*sceneInfo).skyboxMaterialId != MATERIAL_NONE)
                colors[iteration] =
                    skyboxMapping(sceneInfo, materials, textures, &rayOrigin);
            else
            {
                if ((*sceneInfo).extendedGeometry == 2)
                {
                    float4 normal = {0.f, 1.f, 0.f, 0.f};
                    float4 dir =
                        normalize(rayOrigin.direction - rayOrigin.origin);
                    float angle = 0.5f - dot(normal, dir);
                    angle = (angle > 1.f) ? 1.f : angle;
                    colors[iteration] =
                        (1.f - angle) * (*sceneInfo).backgroundColor;
                }
                else
                    colors[iteration] = (*sceneInfo).backgroundColor;
            }
            colorContributions[iteration] = 1.f;
        }
        iteration++;
    }

    float4 areas = {0.f, 0.f, 0.f, 0.f};
    if ((*sceneInfo).graphicsLevel >= glReflectionsAndRefractions &&
        reflectedRays != -1) // TODO: Draft mode should only test
                             // (*sceneInfo).pathTracingIteration==iteration
    {
        // TODO: Dodgy implementation of reflections for transparent material
        if (intersectionWithPrimitives(sceneInfo, boundingBoxes, nbActiveBoxes,
                                       primitives, nbActivePrimitives,
                                       materials, textures, &reflectedRay,
                                       reflectedRays, &closestPrimitive,
                                       &closestIntersection, &normal, &areas,
                                       &colorBox, currentMaterialId))
        {
            float4 attributes;
            attributes.x =
                materials[primitives[closestPrimitive].materialId].reflection;
            float4 color =
                primitiveShader(index, sceneInfo, postProcessingInfo,
                                boundingBoxes, nbActiveBoxes, primitives,
                                nbActivePrimitives, lightInformation,
                                lightInformationSize, nbActiveLamps, materials,
                                textures, randoms, reflectedRay.origin, &normal,
                                closestPrimitive, &closestIntersection, areas,
                                &closestColor, iteration, &refractionFromColor,
                                &shadowIntensity, &rBlinn, &attributes);
            colors[reflectedRays] += color * reflectedRatio;

            (*primitiveXYId).w = shadowIntensity * 255;
        }
    }

    const bool condition =
        processGI && (*sceneInfo).pathTracingIteration >= NB_MAX_ITERATIONS;
    if (condition)
    {
        float alphaIntensity = 1.f;
        if ((*sceneInfo).advancedIllumination == aiFull)
        {
            // Global illumination
            if (intersectionWithPrimitives(
                    sceneInfo, boundingBoxes, nbActiveBoxes, primitives,
                    nbActivePrimitives, materials, textures, &pathTracingRay,
                    10, &closestPrimitive, &closestIntersection, &normal,
                    &areas, &colorBox, MATERIAL_NONE))
            {
                // Ambient occlusion and material emission
                if (primitives[closestPrimitive].materialId != MATERIAL_NONE)
                {
                    CONST Material* material =
                        &materials[primitives[closestPrimitive].materialId];
                    const float distanceToPrimitive =
                        length(closestIntersection - pathTracingRay.origin);
                    const float normalizedDistanceToPrimitive =
                        1.f - min(1.f, distanceToPrimitive /
                                           (*sceneInfo).viewDistance);
                    float4 attributes;
                    pathTracingColor = primitiveShader(
                        index, sceneInfo, postProcessingInfo, boundingBoxes,
                        nbActiveBoxes, primitives, nbActivePrimitives,
                        lightInformation, lightInformationSize, nbActiveLamps,
                        materials, textures, randoms, pathTracingRay.origin,
                        &normal, closestPrimitive, &closestIntersection, areas,
                        &closestColor, iteration, &refractionFromColor,
                        &shadowIntensity, &rBlinn, &attributes);
                    alphaIntensity -= (*sceneInfo).shadowIntensity *
                                      normalizedDistanceToPrimitive;
                    pathTracingRatio *= (MATERIAL_DEFAULT_EMMISION_STRENGTH +
                                         (*material).innerIllumination.x) *
                                        normalizedDistanceToPrimitive;
                }
            }
            else if ((*sceneInfo).skyboxMaterialId != MATERIAL_NONE)
            {
                // Background
                pathTracingColor = skyboxMapping(sceneInfo, materials, textures,
                                                 &pathTracingRay);
                pathTracingRatio *= SKYBOX_LUNINANCE_STRENGTH;
            }
        }
        else
        {
            if ((*sceneInfo).skyboxMaterialId != MATERIAL_NONE)
            {
                // Background
                pathTracingColor = skyboxMapping(sceneInfo, materials, textures,
                                                 &pathTracingRay);
                pathTracingRatio *= SKYBOX_LUNINANCE_STRENGTH;
            }
        }
        colors[0] =
            colors[0] * alphaIntensity + pathTracingColor * pathTracingRatio;
    }

    for (int i = iteration - 2; i >= 0; --i)
        colors[i] = colors[i] * (1.f - colorContributions[i]) +
                    colors[i + 1] * colorContributions[i];
    intersectionColor = colors[0];
    intersectionColor += recursiveBlinn;

    float len = length(firstIntersection - (*ray).origin);
    (*depthOfField) = len;
    if (closestPrimitive != -1)
    {
        CONST Primitive* primitive = &primitives[closestPrimitive];
        if (materials[(*primitive).materialId].attributes.z == 1) // Wireframe
            len = (*sceneInfo).viewDistance;
    }

    // --------------------------------------------------
    // Background color
    // --------------------------------------------------
    float D1 = (*sceneInfo).viewDistance * 0.95f;
    if ((*sceneInfo).atmosphericEffect == aeFog && len > D1)
    {
        float D2 = (*sceneInfo).viewDistance * 0.05f;
        float a = len - D1;
        float b = 1.f - (a / D2);
        intersectionColor =
            intersectionColor * b + (*sceneInfo).backgroundColor * (1.f - b);
    }

    // Primitive information
    (*primitiveXYId).y = iteration;

    // Depth of field
    intersectionColor -= colorBox;

    saturateVector(&intersectionColor);
    return intersectionColor;
}

/*
________________________________________________________________________________

Standard renderer
________________________________________________________________________________
*/
__kernel void k_standardRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    CONST BoundingBox* boundingBoxes, int nbActiveBoxes,
    CONST Primitive* primitives, int nbActivePrimitives,
    CONST LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, CONST Material* materials, CONST BitmapBuffer* textures,
    CONST RandomBuffer* randoms, float4 origin, float4 direction, float4 angles,
    const SceneInfo sceneInfo, const PostProcessingInfo postProcessingInfo,
    CONST PostProcessingBuffer* postProcessingBuffer,
    CONST PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = (stream_split + y) * sceneInfo.size.x + x;
    if (index > sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    // Antialisazing
    float2 AArotatedGrid[4] = {{3.f, 5.f},
                               {5.f, -3.f},
                               {-3.f, -5.f},
                               {-5.f, 3.f}};

    // Beware out of bounds error!
    // And only process pixels that need extra rendering
    const bool condition =
        index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS);
    if (condition)
        return;

    Ray ray;
    ray.origin = origin;
    ray.direction = direction;

    float4 rotationCenter = {0.f, 0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    bool antialiasingActivated = (sceneInfo.cameraType == ctAntialiazed);

    if (postProcessingInfo.type != ppe_depthOfField &&
        sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        // Randomize view for natural depth of field
        float a = postProcessingInfo.param1 / 20000.f;
        int rindex = (index + sceneInfo.timestamp) % (MAX_BITMAP_SIZE - 2);
        ray.origin.x +=
            randoms[rindex] * postProcessingBuffer[index].colorInfo.w * a;
        ray.origin.y +=
            randoms[rindex + 1] * postProcessingBuffer[index].colorInfo.w * a;
    }

    float dof = 0.f;

    if (sceneInfo.cameraType == ctOrthographic)
    {
        ray.direction.x =
            ray.origin.z * 0.001f * (float)(x - (sceneInfo.size.x / 2));
        ray.direction.y =
            -ray.origin.z * 0.001f *
            (float)(device_split + stream_split + y - (sceneInfo.size.y / 2));
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
            ray.direction.x - step.x * (float)(x - (sceneInfo.size.x / 2));
        ray.direction.y =
            ray.direction.y + step.y * (float)(device_split + stream_split + y -
                                               (sceneInfo.size.y / 2));
    }

    vectorRotation(&ray.origin, rotationCenter, angles);
    vectorRotation(&ray.direction, rotationCenter, angles);

    float4 color = {0.f, 0.f, 0.f, 0.f};
    Ray r = ray;
    if (antialiasingActivated)
    {
        for (int I = 0; I < 4; ++I)
        {
            r.direction.x = ray.direction.x + AArotatedGrid[I].x;
            r.direction.y = ray.direction.y + AArotatedGrid[I].y;
            float4 c =
                launchRayTracing(index, boundingBoxes, nbActiveBoxes,
                                 primitives, nbActivePrimitives,
                                 lightInformation, lightInformationSize,
                                 nbActiveLamps, materials, textures, randoms,
                                 &r, &sceneInfo, &postProcessingInfo, &dof,
                                 &primitiveXYIds[index]);
            color += c;
        }
    }
    else
    {
        r.direction.x = ray.direction.x +
                        AArotatedGrid[sceneInfo.pathTracingIteration % 4].x;
        r.direction.y = ray.direction.y +
                        AArotatedGrid[sceneInfo.pathTracingIteration % 4].y;
    }
    color +=
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, &r, &sceneInfo, &postProcessingInfo,
                         &dof, &primitiveXYIds[index]);

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
    }
    else
    {
        postProcessingBuffer[index].colorInfo.x += color.x;
        postProcessingBuffer[index].colorInfo.y += color.y;
        postProcessingBuffer[index].colorInfo.z += color.z;
    }
}

/*
________________________________________________________________________________

Standard renderer
________________________________________________________________________________
*/
__kernel void k_volumeRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    CONST BoundingBox* boundingBoxes, int nbActiveBoxes,
    CONST Primitive* primitives, int nbActivePrimitives,
    CONST LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, CONST Material* materials, CONST BitmapBuffer* textures,
    CONST RandomBuffer* randoms, float4 origin, float4 direction, float4 angles,
    const SceneInfo sceneInfo, const PostProcessingInfo postProcessingInfo,
    CONST PostProcessingBuffer* postProcessingBuffer,
    CONST PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = (stream_split + y) * sceneInfo.size.x + x;

    // Antialisazing
    float2 AArotatedGrid[4] = {{3.f, 5.f},
                               {5.f, -3.f},
                               {-3.f, -5.f},
                               {-5.f, 3.f}};

    // Beware out of bounds error!
    // And only process pixels that need extra rendering
    const bool condition =
        index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x ||
        (sceneInfo.pathTracingIteration >
             primitiveXYIds[index].y &&  // Still need to process iterations
         primitiveXYIds[index].w == 0 && // Shadows? if so, compute soft shadows
                                         // by randomizing light positions
         sceneInfo.pathTracingIteration > 0 &&
         sceneInfo.pathTracingIteration <= NB_MAX_ITERATIONS);

    if (condition)
        return;

    Ray ray;
    ray.origin = origin;
    ray.direction = direction;

    float4 rotationCenter = {0.f, 0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    bool antialiasingActivated = (sceneInfo.cameraType == ctAntialiazed);

    if (postProcessingInfo.type != ppe_depthOfField &&
        sceneInfo.pathTracingIteration >= NB_MAX_ITERATIONS)
    {
        // Randomize view for natural depth of field
        float a = postProcessingInfo.param1 / 20000.f;
        int rindex = (index + sceneInfo.timestamp) % (MAX_BITMAP_SIZE - 2);
        ray.origin.x +=
            randoms[rindex] * postProcessingBuffer[index].colorInfo.w * a;
        ray.origin.y +=
            randoms[rindex + 1] * postProcessingBuffer[index].colorInfo.w * a;
    }

    float dof = 0.f;

    if (sceneInfo.cameraType == ctOrthographic)
    {
        ray.direction.x =
            ray.origin.z * 0.001f * (float)(x - (sceneInfo.size.x / 2));
        ray.direction.y =
            -ray.origin.z * 0.001f *
            (float)(device_split + stream_split + y - (sceneInfo.size.y / 2));
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
            ray.direction.x - step.x * (float)(x - (sceneInfo.size.x / 2));
        ray.direction.y =
            ray.direction.y + step.y * (float)(device_split + stream_split + y -
                                               (sceneInfo.size.y / 2));
    }

    vectorRotation(&ray.origin, rotationCenter, angles);
    vectorRotation(&ray.direction, rotationCenter, angles);

    float4 color = {0.f, 0.f, 0.f, 0.f};
    Ray r = ray;
    if (antialiasingActivated)
    {
        for (int I = 0; I < 4; ++I)
        {
            r.direction.x = ray.direction.x + AArotatedGrid[I].x;
            r.direction.y = ray.direction.y + AArotatedGrid[I].y;
            float4 c = launchVolumeRendering(
                index, boundingBoxes, nbActiveBoxes, primitives,
                nbActivePrimitives, lightInformation, lightInformationSize,
                nbActiveLamps, materials, textures, randoms, &r, &sceneInfo,
                &postProcessingInfo, &dof, &primitiveXYIds[index]);
            color += c;
        }
    }
    else
    {
        r.direction.x = ray.direction.x +
                        AArotatedGrid[sceneInfo.pathTracingIteration % 4].x;
        r.direction.y = ray.direction.y +
                        AArotatedGrid[sceneInfo.pathTracingIteration % 4].y;
    }
    color += launchVolumeRendering(index, boundingBoxes, nbActiveBoxes,
                                   primitives, nbActivePrimitives,
                                   lightInformation, lightInformationSize,
                                   nbActiveLamps, materials, textures, randoms,
                                   &r, &sceneInfo, &postProcessingInfo, &dof,
                                   &primitiveXYIds[index]);

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
    }
    else
    {
        postProcessingBuffer[index].colorInfo.x += color.x;
        postProcessingBuffer[index].colorInfo.y += color.y;
        postProcessingBuffer[index].colorInfo.z += color.z;
    }
}

/*
________________________________________________________________________________

Anaglyph Renderer
________________________________________________________________________________
*/
__kernel void k_anaglyphRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    CONST BoundingBox* boundingBoxes, int nbActiveBoxes,
    CONST Primitive* primitives, int nbActivePrimitives,
    CONST LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, CONST Material* materials, CONST BitmapBuffer* textures,
    CONST RandomBuffer* randoms, float4 origin, float4 direction, float4 angles,
    const SceneInfo sceneInfo, const PostProcessingInfo postProcessingInfo,
    CONST PostProcessingBuffer* postProcessingBuffer,
    CONST PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error!
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float focus =
        primitiveXYIds[sceneInfo.size.x * sceneInfo.size.y / 2].x - origin.z;
    float eyeSeparation = sceneInfo.eyeSeparation * (focus / direction.z);

    float4 rotationCenter = {0.f, 0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    if (sceneInfo.pathTracingIteration == 0)
    {
        postProcessingBuffer[index].colorInfo.x = 0.f;
        postProcessingBuffer[index].colorInfo.y = 0.f;
        postProcessingBuffer[index].colorInfo.z = 0.f;
        postProcessingBuffer[index].colorInfo.w = 0.f;
    }

    float dof = postProcessingInfo.param1;
    Ray eyeRay;

    float ratio = (float)sceneInfo.size.x / (float)sceneInfo.size.y;
    float2 step;
    step.x = 4.f * ratio * angles.w / (float)sceneInfo.size.x;
    step.y = 4.f * angles.w / (float)sceneInfo.size.y;

    // Left eye
    eyeRay.origin.x = origin.x + eyeSeparation;
    eyeRay.origin.y = origin.y;
    eyeRay.origin.z = origin.z;

    eyeRay.direction.x =
        direction.x - step.x * (float)(x - (sceneInfo.size.x / 2));
    eyeRay.direction.y =
        direction.y + step.y * (float)(y - (sceneInfo.size.y / 2));
    eyeRay.direction.z = direction.z;

    // vectorRotation( eyeRay.origin, rotationCenter, angles );
    vectorRotation(&eyeRay.direction, rotationCenter, angles);

    float4 colorLeft =
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, &eyeRay, &sceneInfo,
                         &postProcessingInfo, &dof, &primitiveXYIds[index]);

    // Right eye
    eyeRay.origin.x = origin.x - eyeSeparation;
    eyeRay.origin.y = origin.y;
    eyeRay.origin.z = origin.z;

    eyeRay.direction.x =
        direction.x - step.x * (float)(x - (sceneInfo.size.x / 2));
    eyeRay.direction.y =
        direction.y + step.y * (float)(y - (sceneInfo.size.y / 2));
    eyeRay.direction.z = direction.z;

    vectorRotation(&eyeRay.direction, rotationCenter, angles);

    float4 colorRight =
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, &eyeRay, &sceneInfo,
                         &postProcessingInfo, &dof, &primitiveXYIds[index]);

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

/*
________________________________________________________________________________

3D Vision Renderer
________________________________________________________________________________
*/
__kernel void k_3DVisionRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    CONST BoundingBox* boundingBoxes, int nbActiveBoxes,
    CONST Primitive* primitives, int nbActivePrimitives,
    CONST LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, CONST Material* materials, CONST BitmapBuffer* textures,
    CONST RandomBuffer* randoms, float4 origin, float4 direction, float4 angles,
    const SceneInfo sceneInfo, const PostProcessingInfo postProcessingInfo,
    CONST PostProcessingBuffer* postProcessingBuffer,
    CONST PrimitiveXYIdBuffer* primitiveXYIds)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error!
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float focus =
        primitiveXYIds[sceneInfo.size.x * sceneInfo.size.y / 2].x - origin.z;
    float eyeSeparation = sceneInfo.eyeSeparation * (direction.z / focus);

    float4 rotationCenter = {0.f, 0.f, 0.f, 0.f};
    if (sceneInfo.cameraType == ctVR)
        rotationCenter = origin;

    if (sceneInfo.pathTracingIteration == 0)
    {
        postProcessingBuffer[index].colorInfo.x = 0.f;
        postProcessingBuffer[index].colorInfo.y = 0.f;
        postProcessingBuffer[index].colorInfo.z = 0.f;
        postProcessingBuffer[index].colorInfo.w = 0.f;
    }

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

    if (sqrt(eyeRay.direction.x * eyeRay.direction.x +
             eyeRay.direction.y * eyeRay.direction.y) > (halfWidth * 6))
        return;

    vectorRotation(&eyeRay.origin, rotationCenter, angles);
    vectorRotation(&eyeRay.direction, rotationCenter, angles);

    float4 color =
        launchRayTracing(index, boundingBoxes, nbActiveBoxes, primitives,
                         nbActivePrimitives, lightInformation,
                         lightInformationSize, nbActiveLamps, materials,
                         textures, randoms, &eyeRay, &sceneInfo,
                         &postProcessingInfo, &dof, &primitiveXYIds[index]);

    // Randomize light intensity
    int rindex = (index + sceneInfo.timestamp) % MAX_BITMAP_SIZE;
    color += sceneInfo.backgroundColor * randoms[rindex] * 5.f;

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

/*
________________________________________________________________________________

3D Vision Renderer
________________________________________________________________________________
*/
__kernel void k_fishEyeRenderer(
    const int2 occupancyParameters, int device_split, int stream_split,
    CONST BoundingBox* boundingBoxes, int nbActiveBoxes,
    CONST Primitive* primitives, int nbActivePrimitives,
    CONST LightInformation* lightInformation, int lightInformationSize,
    int nbActiveLamps, CONST Material* materials, CONST BitmapBuffer* textures,
    CONST RandomBuffer* randoms, float4 origin, float4 direction, float4 angles,
    const SceneInfo sceneInfo, const PostProcessingInfo postProcessingInfo,
    CONST PostProcessingBuffer* postProcessingBuffer,
    CONST PrimitiveXYIdBuffer* primitiveXYIds)
{
}

/*
________________________________________________________________________________

Post Processing Effect: Default
________________________________________________________________________________
*/
__kernel void k_default(const int2 occupancyParameters,
                        const SceneInfo sceneInfo,
                        CONST PostProcessingBuffer* postProcessingBuffer,
                        CONST BitmapBuffer* bitmap)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error!
    if (index > sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float4 localColor = postProcessingBuffer[index].colorInfo;
    if (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
        localColor /=
            (float)(sceneInfo.pathTracingIteration - NB_MAX_ITERATIONS + 1);
    makeColor(&sceneInfo, &localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Depth of field
________________________________________________________________________________
*/
__kernel void k_depthOfField(const int2 occupancyParameters,
                             const SceneInfo sceneInfo,
                             const PostProcessingInfo postProcessingInfo,
                             CONST PostProcessingBuffer* postProcessingBuffer,
                             CONST RandomBuffer* randoms,
                             CONST BitmapBuffer* bitmap)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error!
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float4 localColor = {0.f, 0.f, 0.f, 0.f};
    float depth = fabs(postProcessingBuffer[index].colorInfo.w -
                       postProcessingInfo.param1) /
                  sceneInfo.viewDistance;
    int wh = sceneInfo.size.x * sceneInfo.size.y;

    for (int i = 0; i < postProcessingInfo.param3; ++i)
    {
        int ix = i % wh;
        int iy = (i + 100) % wh;
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
    makeColor(&sceneInfo, &localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Ambiant Occlusion
________________________________________________________________________________
*/
__kernel void k_ambientOcclusion(
    const int2 occupancyParameters, SceneInfo sceneInfo,
    PostProcessingInfo postProcessingInfo,
    CONST PostProcessingBuffer* postProcessingBuffer,
    CONST RandomBuffer* randoms, CONST BitmapBuffer* bitmap)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;
    // Beware out of bounds error!
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    int wh = sceneInfo.size.x * sceneInfo.size.y;
    float occ = 0.f;
    float4 localColor = postProcessingBuffer[index].colorInfo;
    float depth = localColor.w;
    const int step = 16;
    float c = 0.f;
    int i = 0;
    for (int X = -step; X < step; X += 2)
    {
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
                if (postProcessingBuffer[localIndex].colorInfo.w < depth)
                    occ += 1.f - (postProcessingBuffer[localIndex].colorInfo.w -
                                  depth) /
                                     sceneInfo.viewDistance;
            }
            else
                occ += 1.f;
        }
    }
    occ /= 5.f * c;
    // occ += 0.3f; // Ambient light
    occ = (occ > 1.f) ? 1.f : occ;
    occ = (occ < 0.f) ? 0.f : occ;
    if (sceneInfo.pathTracingIteration > NB_MAX_ITERATIONS)
        localColor /=
            (float)(sceneInfo.pathTracingIteration - NB_MAX_ITERATIONS + 1);

    localColor.x -= occ;
    localColor.y -= occ;
    localColor.z -= occ;
    saturateVector(&localColor);

    localColor.w = 1.f;

    makeColor(&sceneInfo, &localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: radiosity
________________________________________________________________________________
*/
__kernel void k_radiosity(const int2 occupancyParameters, SceneInfo sceneInfo,
                          PostProcessingInfo postProcessingInfo,
                          CONST PrimitiveXYIdBuffer* primitiveXYIds,
                          CONST PostProcessingBuffer* postProcessingBuffer,
                          CONST RandomBuffer* randoms,
                          CONST BitmapBuffer* bitmap)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;

    // Beware out of bounds error!
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

    float4 localColor = {0.f, 0.f, 0.f, 0.f};
    float a = (1.f / NB_MAX_ITERATIONS) * primitiveXYIds[index].y;
    localColor.x = a;
    localColor.y = a;
    localColor.z = a;
    localColor.w = 1.f;
    makeColor(&sceneInfo, &localColor, bitmap, index);
}

/*
________________________________________________________________________________

Post Processing Effect: Default
________________________________________________________________________________
*/
__kernel void k_filter(const int2 occupancyParameters, SceneInfo sceneInfo,
                       PostProcessingInfo postProcessingInfo,
                       CONST PostProcessingBuffer* postProcessingBuffer,
                       CONST BitmapBuffer* bitmap)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int index = y * sceneInfo.size.x + x;
    // Beware out of bounds error!
    if (index >= sceneInfo.size.x * sceneInfo.size.y / occupancyParameters.x)
        return;

// Filters
#define NB_FILTERS 6
    const int2 filterSize[NB_FILTERS]; // = {{3, 3}, {5, 5}, {3, 3}, {3, 3}, {5,
                                       // 5}, {5, 5}};

    const float2 filterFactors[NB_FILTERS];
    // = {{1.f, 128.f}, {1.f, 0.f},  {1.f, 0.f},
    // {1.f, 0.f},   {0.2f, 0.f}, {0.125f, 0.f}}; // Factor and Bias

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
        {
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
                    c /= (float)(sceneInfo.pathTracingIteration -
                                 NB_MAX_ITERATIONS + 1);

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
        }
        // truncate values smaller than zero and larger than 255
        float factor = filterFactors[postProcessingInfo.param3].x;
        float bias = filterFactors[postProcessingInfo.param3].y;
        color.x += min(max(factor * localColor.x + bias / 255.f, 0.f), 1.f);
        color.y += min(max(factor * localColor.y + bias / 255.f, 0.f), 1.f);
        color.z += min(max(factor * localColor.z + bias / 255.f, 0.f), 1.f);
    }

    saturateVector(&color);
    localColor.w = 1.f;
    makeColor(&sceneInfo, &color, bitmap, index);
}

#ifdef DEBUG
/*
________________________________________________________________________________

Display struct sizes
________________________________________________________________________________
*/
__kernel void k_alignment()
{
    printf("OCL  [1] --== OCL Data structures ==--\n");
    printf("OCL  [1] Ray               :  %i\n", sizeof(Ray));
    printf("OCL  [1] SceneInfo         :  %i\n", sizeof(SceneInfo));
    printf("OCL  [1] PostProcessingInfo:  %i\n", sizeof(PostProcessingInfo));
    printf("OCL  [1] TextureInfo       :  %i\n", sizeof(TextureInfo));
    printf("OCL  [1] Primitive         :  %i\n", sizeof(Primitive));
    printf("OCL  [1] BoundingBox       :  %i\n", sizeof(BoundingBox));
    printf("OCL  [1] Material          :  %i\n", sizeof(Material));
    printf("OCL  [1] LightInformation  :  %i\n", sizeof(LightInformation));
    printf("OCL  [1] -----------------------------\n");
}
#endif
