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

#include "api.h"
#include "solr.h"

#ifdef WIN32
#ifdef USE_KINECT
#include <NuiApi.h>
#endif // USE_KINECT
#else
#include <string.h>
#include <string>
#endif // WIN32

#ifdef USE_OCULUS
// Oculus Rift
#include <OVR.h>
#endif // USE_OCULUS

#include <map>
#include <vector>

namespace solr
{
class SOLR_API GPUKernel
{
public:
    GPUKernel();
    virtual ~GPUKernel();

    virtual void initBuffers();
    virtual void cleanup();
    virtual void reshape();

public:
    virtual void setPlatformId(const int platform) = 0;
    virtual void setDeviceId(const int device) = 0;
    virtual void setKernelFilename(const std::string &kernelFilename) = 0;

public:
    virtual void queryDevice() = 0;
    virtual void recompileKernels() = 0;

public:
    // ---------- Rendering ----------
    virtual void render_begin(const float timer);
    virtual void render_end() = 0;
    BitmapBuffer *getBitmap() { return m_bitmap; }
    void generateScreenshot(const std::string &filename,
                            const unsigned int width, const unsigned int height,
                            const unsigned int quality);

public:
    // ---------- Primitives ----------
    int addPrimitive(PrimitiveType type, bool belongsToModel = false);
    void setPrimitive(const int &index, float x0, float y0, float z0, float w,
                      float h, float d, int materialId);
    void setPrimitive(const int &index, float x0, float y0, float z0, float x1,
                      float y1, float z1, float w, float h, float d,
                      int materialId);
    void setPrimitive(const int &index, float x0, float y0, float z0, float x1,
                      float y1, float z1, float x2, float y2, float z2, float w,
                      float h, float d, int materialId);
    unsigned int getPrimitiveAt(int x, int y);
    void setPrimitiveIsMovable(const int &index, bool movable);
    void setPrimitiveBellongsToModel(const int &index, bool bellongsToModel);

    // Scaling
    void scalePrimitives(float scale, unsigned int from, unsigned int to);

    // Rotation
    void rotatePrimitives(const vec3f &rotationCenter, const vec4f &angles);
    void rotatePrimitive(CPUPrimitive &primitive, const vec3f &rotationCenter,
                         const vec3f &cosAngles, const vec3f &sinAngles);
    void rotateBox(CPUBoundingBox &box, vec3f rotationCenter, vec3f cosAngles,
                   vec3f sinAngles);
    vec4f getRotation() { return m_rotation; }
    // Translation
    void translatePrimitives(const vec3f &);
    vec3f getTranslation() { return m_translation; }
    // Morphing
    void morphPrimitives();

    // Material
    void setPrimitiveMaterial(unsigned int index, int materialId);
    int getPrimitiveMaterial(unsigned int index);
    vec4f getPrimitiveCenter(unsigned int index);
    void getPrimitiveOtherCenter(unsigned int index, vec3f &otherCenter);
    void setPrimitiveCenter(unsigned int index, const vec3f &center);

    // Texture coordinates
    void setPrimitiveTextureCoordinates(const unsigned int index,
                                        const vec2f &vt0, const vec2f &vt1,
                                        const vec2f &vt2);

    // Normals
    void setPrimitiveNormals(unsigned int index, vec3f n0, vec3f n1, vec3f n2);

    // Lights
    int getLight(int index);
    void reorganizeLights();
    CPUPrimitive *getPrimitive(const unsigned int index);

public:
    // OpenGL
    int setGLMode(const int &glMode);
    void addVertex(float x, float y, float z);
    void addNormal(float x, float y, float z);
    void addTextureCoordinates(float x, float y);
    void translate(float x, float y, float z);
    void rotate(float x, float y, float z);

public:
    // ---------- Complex objects ----------
    int addCube(float x, float y, float z, float radius, int materialId);

    int addRectangle(float x, float y, float z, float w, float h, float d,
                     int materialId);

public:
    // ---------- Materials ----------
    int addMaterial();
    void setMaterial(unsigned int index, const Material &material);
    void setMaterial(unsigned int index, float r, float g, float b, float gloss,
                     float reflection, float refraction, bool procedural,
                     bool wireframe, int wireframeWidth, float transparency,
                     float opacity, int diffuseTextureId, int normalTextureId,
                     int bumpTextureId, int specularTextureId,
                     int reflectionTextureId, int transparentTextureId,
                     int ambientOcclusionTextureId, float specValue,
                     float specPower, float specCoef, float innerIllumination,
                     float illuminationDiffusion, float illuminationPropagation,
                     bool fastTransparency);

    void setMaterialColor(unsigned int index, float r, float g, float b);
    void setMaterialTextureId(unsigned int textureId);

    int getMaterialAttributes(
        int index, float &r, float &g, float &b, float &gloss,
        float &reflection, float &refraction, bool &procedural, bool &wireframe,
        int &wireframeDepth, float &transparency, float &opacity,
        int &diffuseTextureId, int &normalTextureId, int &bumpTextureId,
        int &specularTextureId, int &reflectionTextureId,
        int &transparencyTextureId, int &ambientOcclusionTextureId,
        float &specValue, float &specPower, float &specCoef,
        float &innerIllumination, float &illuminationDiffusion,
        float &illuminationPropagation, bool &fastTransparency);

    Material *getMaterial(const int index);
    int getCurrentMaterial();
    void setCurrentMaterial(const int currentMaterial);

public:
    // ---------- Camera ----------
    void setCamera(const vec3f &eye, const vec3f &dir, const vec4f &angles);

public:
    // ---------- Textures ----------
    void setTexture(const int index, const TextureInfo &textureInfo);
    void getTexture(const int index, TextureInfo &textureInfo);
    void setTexturesTransfered(const bool transfered)
    {
        m_texturesTransfered = transfered;
    }
    void realignTexturesAndMaterials();

    bool loadTextureFromFile(const int index, const std::string &filename);
    void buildLightInformationFromTexture(unsigned int index);
    void processTextureOffsets();

public:
    void setSceneInfo(int width, int height, float transparentColor,
                      int shadowsEnabled, float viewDistance,
                      float shadowIntensity, int nbRayIterations,
                      vec4f backgroundColor, int supportFor3DVision,
                      float eyeSeparation, bool renderBoxes,
                      int pathTracingIteration, int maxPathTracingIterations,
                      FrameBufferType frameBufferType, int timestamp,
                      int atmosphericEffect, int skyboxSize,
                      int skyboxMaterialId);

    // Scene
    void setSceneInfo(const SceneInfo &sceneInfo);
    SceneInfo &getSceneInfo();

    // Post processing
    void setPostProcessingInfo(PostProcessingType type, float param1,
                               float param2, int param3);
    void setPostProcessingInfo(const PostProcessingInfo &postProcessingInfo);
    PostProcessingInfo &getPostProcessingInfo() { return m_postProcessingInfo; }

public:
    // Vector Utilities
    float vectorLength(const vec3f &vector);
    void normalizeVector(vec3f &v);
    vec3f crossProduct(const vec3f &b, const vec3f &c);
    float dotProduct(const vec3f &a, const vec3f &b);

    // Bitmap export
    void saveBitmapToFile(const std::string &filename, BitmapBuffer *bitmap,
                          const int width, const int height, const int depth);

    // Oculus
public:
    void switchOculusVR();

protected:
#ifdef USE_OCULUS
    void initializeOVR();
    void finializeOVR();

private:
    // Oculus
    OVR::SensorFusion *m_sensorFusion;
    OVR::Ptr<OVR::SensorDevice> m_sensor;
    OVR::Ptr<OVR::DeviceManager> m_manager;
    OVR::Ptr<OVR::HMDDevice> m_HMD;
#endif             //  USE_OCULUS
    bool m_oculus; // True if Oculus is present and active

#ifdef USE_KINECT
public:
    // ---------- Kinect ----------
    void initializeKinectTextures();

    int updateSkeletons(unsigned int primitiveIndex, vec3f skeletonPosition,
                        float size, float radius, int materialId,
                        float head_radius, int head_materialId,
                        float hands_radius, int hands_materialId,
                        float feet_radius, int feet_materialId);

    bool getSkeletonPosition(int index, vec3f &position);
    BitmapBuffer *getDepthBitmap()
    {
        return m_hDepth;
    }
    BitmapBuffer *getVideoBitmap()
    {
        return m_hVideo;
    }
#endif // USE_KINECT

public:
    unsigned int getNbActiveBoxes();
    unsigned int getNbActivePrimitives();
    unsigned int getNbActiveLamps();
    unsigned int getNbActiveMaterials();
    unsigned int getNbActiveTextures();
    std::string getTextureFilename(const int index);
    TextureInfo &getTextureInformation(const int index);

    void resetAddingIndex()
    {
        m_addingIndex = 0;
    }
    void doneWithAdding(const bool &doneWithAdding)
    {
        m_doneWithAdding = doneWithAdding;
    }
    void resetFrame();
    void resetAll();

    void setDistortion(const float distortion)
    {
        m_distortion = distortion;
    }
    void setPointSize(const float pointSize);

public:
    // char* loadFromFile( const std::string& filename, size_t& length );
    void loadFromFile(const std::string &filename);
    void saveToFile(const std::string &filename);

public:
    virtual std::string getGPUDescription() = 0;

public:
    // Frames
    void setNbFrames(const int nbFrames);
    void setFrame(const int frame);
    int getNbFrames();
    int getFrame();
    void nextFrame();
    void previousFrame();

public:
    void rotateVector(vec3f &v, const vec3f &rotationCenter,
                      const vec3f &cosAngles, const vec3f &sinAngles);

public:
    int compactBoxes(bool reconstructBoxes);
    void streamDataToGPU();
    void displayBoxesInfo();
    void resetBoxes(bool resetPrimitives);

    void setPrimitivesTransfered(const bool value)
    {
        m_primitivesTransfered = value;
    }

public:
    vec3f &getViewPos()
    {
        return m_viewPos;
    }
    vec3f &getViewDir()
    {
        return m_viewDir;
    }
    vec4f &getViewAngles()
    {
        return m_angles;
    }

protected:
    // Bounding boxes management
    int processBoxes(const int boxSize, bool simulate);
    int processOutterBoxes(const int boxSize, const int boundingBoxesDepth);
    bool updateBoundingBox(CPUBoundingBox &box);
    bool updateOutterBoundingBox(CPUBoundingBox &box, const int depth);
    void resetBox(CPUBoundingBox &box, bool resetPrimitives);

    void recursiveDataStreamToGPU(const int depth, std::vector<long> &elements);

protected:
    // GPU
    BoundingBox *m_hBoundingBoxes;
    Primitive *m_hPrimitives;
    int *m_hLamps;
    Material *m_hMaterials;

    // Textures
    TextureInfo m_hTextures[NB_MAX_TEXTURES];
    std::map<int, std::string> m_textureFilenames;

    // Scene
    RandomBuffer *m_hRandoms;
    PrimitiveXYIdBuffer *m_hPrimitivesXYIds;

    // Acceleration structures
    int m_nbActiveBoxes[NB_MAX_FRAMES];
    int m_nbActivePrimitives[NB_MAX_FRAMES];
    int m_nbActiveLamps[NB_MAX_FRAMES];
    int m_nbActiveMaterials;
    int m_nbActiveTextures;
    int m_lightInformationSize;
    size_t m_maxPrimitivesPerBox;
    bool m_doneWithAdding;
    int m_addingIndex;

    // Camersa position
    vec3f m_viewPos;
    vec3f m_viewDir;
    vec4f m_angles;

    // Distortion (VR)
    float m_distortion;

protected:
    unsigned int m_frame;
    unsigned int m_nbFrames;
    float m_morph;
    unsigned int m_treeDepth;

protected:
    // Rendering
    BitmapBuffer *m_bitmap;

protected:
    bool m_primitivesTransfered;
    bool m_materialsTransfered;
    bool m_texturesTransfered;
    bool m_randomsTransfered;
    // Scene Size
    vec3f m_minPos[NB_MAX_FRAMES];
    vec3f m_maxPos[NB_MAX_FRAMES];

protected:
    // Scene
    SceneInfo m_sceneInfo;

    // Post Processing
    PostProcessingInfo m_postProcessingInfo;

    // Refresh
    bool m_refresh;

protected:
    // activate or deactivate logging
    bool m_activeLogging;

    // GPU information
    std::string m_gpuDescription;

    // GPUs & streams
    vec2i m_occupancyParameters;

protected:
    // CPU
    BoxContainer m_boundingBoxes[NB_MAX_FRAMES][BOUNDING_BOXES_TREE_DEPTH];
    PrimitiveContainer m_primitives[NB_MAX_FRAMES];
    LampContainer m_lamps[NB_MAX_FRAMES];
    LightInformation *m_lightInformation;

protected:
    int m_optimalNbOfBoxes;

protected:
    // OpenGL
    int m_GLMode;
    vec3fs m_vertices;
    vec3fs m_normals;
    vec2fs m_textCoords;
    vec3f m_translation;
    vec4f m_rotation;
    int m_currentMaterial;
    float m_pointSize;

protected:
    // Benchmark
    long m_counter;

// Kinect declarations
#ifdef USE_KINECT
protected:
    bool m_kinectEnabled;
    HANDLE m_skeletons;
    HANDLE m_hNextDepthFrameEvent;
    HANDLE m_hNextVideoFrameEvent;
    HANDLE m_hNextSkeletonEvent;
    HANDLE m_pVideoStreamHandle;
    HANDLE m_pDepthStreamHandle;
    NUI_SKELETON_FRAME m_skeletonFrame;

    int m_skeletonIndex;
    int m_skeletonsBody;
    int m_skeletonsLamp;

    BitmapBuffer *m_hVideo;
    BitmapBuffer *m_hDepth;

#endif // USE_KINECT
};

class SOLR_API SingletonKernel
{
private:
    SingletonKernel();

    static GPUKernel *m_kernel;

public:
    static GPUKernel *kernel();
};
} // namespace solr
