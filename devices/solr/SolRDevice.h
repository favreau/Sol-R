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

#pragma once

// helium
#include "helium/BaseDevice.h"
// solr
// #include "SolRStub.h"  // Disabled when building ANARI device standalone
// std
#include <atomic>
#include <memory>

namespace solr
{

enum class DeviceInitStatus
{
    UNINITIALIZED,
    SUCCESS,
    FAILURE
};

struct SolRDevice : public helium::BaseDevice
{
    /////////////////////////////////////////////////////////////////////////////
    // Main interface to accepting API calls
    /////////////////////////////////////////////////////////////////////////////

    void *mapArray(ANARIArray) override;
    void unmapArray(ANARIArray) override;

    ANARIArray1D newArray1D(const void *appMemory, ANARIMemoryDeleter deleter,
                            const void *deleterPtr, ANARIDataType,
                            uint64_t numItems1) override;

    ANARIArray2D newArray2D(const void *appMemory, ANARIMemoryDeleter deleter,
                            const void *deleterPtr, ANARIDataType,
                            uint64_t numItems1, uint64_t numItems2) override;

    ANARIArray3D newArray3D(const void *appMemory, ANARIMemoryDeleter deleter,
                            const void *deleterPtr, ANARIDataType,
                            uint64_t numItems1, uint64_t numItems2,
                            uint64_t numItems3) override;

    ANARICamera newCamera(const char *type) override;
    ANARIFrame newFrame() override;
    ANARIGeometry newGeometry(const char *type) override;
    ANARIGroup newGroup() override;
    ANARIInstance newInstance(const char *type) override;
    ANARILight newLight(const char *type) override;
    ANARIMaterial newMaterial(const char *material_type) override;
    ANARIRenderer newRenderer(const char *type) override;
    ANARISampler newSampler(const char *type) override;
    ANARISpatialField newSpatialField(const char *type) override;
    ANARISurface newSurface() override;
    ANARIVolume newVolume(const char *type) override;
    ANARIWorld newWorld() override;

    // Commit and release
    void commitParameters(ANARIObject object) override;
    void release(ANARIObject _obj) override;

    // Object + Parameter Introspection
    int getProperty(ANARIObject object, const char *name, ANARIDataType type,
                    void *mem, uint64_t size, uint32_t mask) override;

    // Frame operations
    void renderFrame(ANARIFrame) override;
    int frameReady(ANARIFrame, ANARIWaitMask) override;
    void discardFrame(ANARIFrame) override;

    /////////////////////////////////////////////////////////////////////////////
    // Helper/other functions and data members
    /////////////////////////////////////////////////////////////////////////////

    SolRDevice(ANARIStatusCallback defaultCallback, const void *userPtr);
    SolRDevice(ANARILibrary library);

    ~SolRDevice() override;

    void initDevice();

    int deviceGetProperty(const char *name, ANARIDataType type, void *mem,
                          uint64_t size, uint32_t mask) override;

    void deviceCommit();

    void flushCommitBuffer();

    // ANARI interface requirements
    const char **getObjectSubtypes(ANARIDataType objectType) override;
    const void *getObjectInfo(ANARIDataType objectType,
                              const char *objectSubtype, const char *infoName,
                              ANARIDataType infoType) override;
    const void *getParameterInfo(ANARIDataType objectType,
                                 const char *objectSubtype,
                                 const char *parameterName,
                                 ANARIDataType parameterType,
                                 const char *infoName,
                                 ANARIDataType infoType) override;

    // SolRStub *getSolRStub() const;  // Disabled when building ANARI device
    // standalone

private:
    DeviceInitStatus m_initStatus{DeviceInitStatus::UNINITIALIZED};
    // std::unique_ptr<SolRStub> m_solrStub;  // Disabled when building ANARI
    // device standalone

    // Device state
    std::atomic<uint64_t> m_frameCounter{0};
};

} // namespace solr