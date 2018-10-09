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

#include <solr_defines.h>
#include <common/Types.h>

// Raytracer
#include <Logging.h>
#include <engines/GPUKernel.h>

using namespace solr;

class Scene
{
public:
    Scene(const std::string& name);
    virtual ~Scene(void);

public:
    void initialize(GPUKernel* kernel, const int width, const int height);
    void animate();
    void render(const bool& animate);

    virtual void renderText();

    void createRandomMaterials(bool update, bool lightsOnly);
    void createMoleculeMaterials(bool update = false);
    void loadTextures(const std::string& path, const Strings& filters);

    void addCornellBox(int boxType);

    void saveToFile();
    void loadFromFile(const float scale);

    void createSkeleton();
    void animateSkeleton();

    void rotatePrimitives(const vec3f& rotationCenter, const vec4f& angles);

public:
    SceneInfo& getSceneInfo();
    std::string& getName();

    int getCornellBox() { return m_cornellBoxType; }
    void setCornellBoxType(int cornellBoxType) { m_cornellBoxType = cornellBoxType; }
    float getGroundHeight() { return m_groundHeight; }
    void setGroundHeight(const float h) { m_groundHeight = h; }

public:
    // void setMaterialTexture( const int& index, const int& texture );

public:
    int getNbPrimitives() { return m_gpuKernel->getNbActivePrimitives(); }
    GPUKernel* getKernel() { return m_gpuKernel; }
    int getNbHDRI() { return m_nbHDRI; }

public:
    void setCurrentModel(const int& currentModel) { m_currentModel = currentModel; }

protected:
    virtual void doInitialize() = 0;
    virtual void doPostInitialize(){};
    virtual void doAnimate() = 0;
    virtual void doAddLights() = 0;

protected:
    void createWorm(const vec3f& center, int boxId, int material);
    void createDog(const vec3f& center, int material, float size, int boxid);

protected:
    std::string m_name;
    GPUKernel* m_gpuKernel;

    // Textures
    int m_nbHDRI;

    // Scene information
    int m_nbBoxes;
    int m_cornellBoxType;
    float m_groundHeight;
    int m_maxPathTracingIterations;

    // Primitives
    int m_nbPrimitives;
    // Materials
    int m_nbMaterials;

    // Scene models
    int m_currentModel;

protected:
    // Animation
    vec3f m_rotationCenter;
    vec4f m_rotationAngles;

#ifdef USE_KINECT
protected:
    int m_skeletonPrimitiveIndex;
    float m_skeletonSize;
    float m_skeletonThickness;
    vec3f m_skeletonPosition;
    vec3f m_skeletonOldPosition;

    float m_skeletonKinectSpace;
    float m_skeletonKinectSize;
    int m_skeletonKinectStep;
    int m_skeletonKinectNbSpherePerBox;
    vec4f m_acceleration;

    vec4f m_previewViewPos;
#endif // USE_KINECT

#ifdef USE_SIXENSE
private:
    vec4f m_modelRotationAngle;
    vec3f m_modelPosition;
    vec4f m_modelTranslation;
#endif
};
