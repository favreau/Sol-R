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

#include "SolRDevice.h"
// std
#include <string>

// Disable verbose debug messages to avoid initialization issues
#define SOLR_VERBOSE_DEBUG 0

namespace solr
{

const char **query_extensions();

///////////////////////////////////////////////////////////////////////////////
// Generated function declarations ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

const char **query_object_types(ANARIDataType type);
const void *query_object_info(ANARIDataType type, const char *name,
                              ANARIDataType infoType);
const void *query_param_info(ANARIDataType type, const char *name,
                             ANARIDataType paramType, const char *infoType);

///////////////////////////////////////////////////////////////////////////////
// SolRDevice definitions /////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Data Arrays ////////////////////////////////////////////////////////////////

void *SolRDevice::mapArray(ANARIArray a)
{
#if SOLR_VERBOSE_DEBUG
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::mapArray called");

#endif
#endif
    // For now, return a placeholder - implement based on Sol-R's memory
    // management
    return nullptr;
}

void SolRDevice::unmapArray(ANARIArray a)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::unmapArray called");

#endif
    // Implement based on Sol-R's memory management
}

ANARIArray1D SolRDevice::newArray1D(const void *appMemory,
                                    ANARIMemoryDeleter deleter,
                                    const void *deleterPtr, ANARIDataType type,
                                    uint64_t numItems1)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newArray1D called");

#endif
    // Return a placeholder array object - implement proper array handling
    return (ANARIArray1D)this;
}

ANARIArray2D SolRDevice::newArray2D(const void *appMemory,
                                    ANARIMemoryDeleter deleter,
                                    const void *deleterPtr, ANARIDataType type,
                                    uint64_t numItems1, uint64_t numItems2)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newArray2D called");

#endif
    return (ANARIArray2D)this;
}

ANARIArray3D SolRDevice::newArray3D(const void *appMemory,
                                    ANARIMemoryDeleter deleter,
                                    const void *deleterPtr, ANARIDataType type,
                                    uint64_t numItems1, uint64_t numItems2,
                                    uint64_t numItems3)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newArray3D called");

#endif
    return (ANARIArray3D)this;
}

// Renderable Objects /////////////////////////////////////////////////////////

ANARICamera SolRDevice::newCamera(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newCamera called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARICamera)this;
}

ANARIFrame SolRDevice::newFrame()
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newFrame called");

#endif
    return (ANARIFrame)this;
}

ANARIGeometry SolRDevice::newGeometry(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newGeometry called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARIGeometry)this;
}

ANARIGroup SolRDevice::newGroup()
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newGroup called");

#endif
    return (ANARIGroup)this;
}

ANARIInstance SolRDevice::newInstance(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newInstance called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARIInstance)this;
}

ANARILight SolRDevice::newLight(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newLight called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARILight)this;
}

ANARIMaterial SolRDevice::newMaterial(const char *material_type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newMaterial called with type: %s",
                  material_type ? material_type : "NULL");

#endif
    return (ANARIMaterial)this;
}

ANARIRenderer SolRDevice::newRenderer(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newRenderer called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARIRenderer)this;
}

ANARISampler SolRDevice::newSampler(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newSampler called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARISampler)this;
}

ANARISpatialField SolRDevice::newSpatialField(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newSpatialField called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARISpatialField)this;
}

ANARISurface SolRDevice::newSurface()
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newSurface called");

#endif
    return (ANARISurface)this;
}

ANARIVolume SolRDevice::newVolume(const char *type)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG,
                  "SolRDevice::newVolume called with type: %s",
                  type ? type : "NULL");

#endif
    return (ANARIVolume)this;
}

ANARIWorld SolRDevice::newWorld()
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::newWorld called");

#endif
    return (ANARIWorld)this;
}

// Object + Parameter Introspection ///////////////////////////////////////////

void SolRDevice::commitParameters(ANARIObject object)
{
    if (this == (SolRDevice *)object)
    {
        deviceCommit();
    }
}

void SolRDevice::release(ANARIObject _obj)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::release called");

#endif
    // For now, don't actually release anything since we're using placeholders
}

int SolRDevice::getProperty(ANARIObject object, const char *name,
                            ANARIDataType type, void *mem, uint64_t size,
                            uint32_t mask)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::getProperty called");

#endif
    if (object == (ANARIObject)this)
    {
        return deviceGetProperty(name, type, mem, size, mask);
    }
    return 0;
}

// Frame Operations ////////////////////////////////////////////////////////////

void SolRDevice::renderFrame(ANARIFrame frame)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::renderFrame called");

#endif
    // Implement actual rendering using Sol-R
    // SolRStub disabled when building ANARI device standalone
    // This would integrate with Sol-R's rendering pipeline
    // For now, just increment frame counter
    m_frameCounter++;
}

int SolRDevice::frameReady(ANARIFrame frame, ANARIWaitMask m)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::frameReady called");

#endif
    // For now, always return ready
    return 1;
}

void SolRDevice::discardFrame(ANARIFrame frame)
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice::discardFrame called");

#endif
    // Implement frame discard logic
}

// Other SolRDevice definitions ///////////////////////////////////////////////

SolRDevice::SolRDevice(ANARIStatusCallback defaultCallback, const void *userPtr)
    : helium::BaseDevice(defaultCallback, userPtr)
{
    // SolRStub disabled when building ANARI device standalone
    // m_solrStub = std::make_unique<SolRStub>();
    // Note: deviceCommit() is called automatically by helium framework
}

SolRDevice::SolRDevice(ANARILibrary library)
    : helium::BaseDevice(library)
{
    // SolRStub disabled when building ANARI device standalone
    // m_solrStub = std::make_unique<SolRStub>();
    // Note: deviceCommit() is called automatically by helium framework
}

SolRDevice::~SolRDevice()
{
#if SOLR_VERBOSE_DEBUG

    reportMessage(ANARI_SEVERITY_DEBUG, "SolRDevice destroyed");

#endif
}

void SolRDevice::initDevice()
{
    if (m_initStatus != DeviceInitStatus::UNINITIALIZED)
        return;

    try
    {
        // SolRStub disabled when building ANARI device standalone
        m_initStatus = DeviceInitStatus::SUCCESS;
    }
    catch (const std::exception &e)
    {
        m_initStatus = DeviceInitStatus::FAILURE;
    }
}

void SolRDevice::deviceCommit()
{
    initDevice();

    // Note: Initialization status checking moved to after device is fully
    // initialized
}

int SolRDevice::deviceGetProperty(const char *name, ANARIDataType type,
                                  void *mem, uint64_t size, uint32_t mask)
{
    if (!name || !mem)
        return 0;

    std::string prop = name;
    if (prop == "feature" && type == ANARI_STRING_LIST)
    {
        *(const char ***)mem = query_extensions();
        return 1;
    }
    else if (prop == "solr" && type == ANARI_BOOL)
    {
        *(bool *)mem = true;
        return 1;
    }
    else if (prop == "version" && type == ANARI_STRING)
    {
        *(const char **)mem = "0.1.0";
        return 1;
    }
    else if (type == ANARI_STRING_LIST)
    {
        // Return empty list for unknown string list properties
        static const char *empty_list[] = {nullptr};
        *(const char ***)mem = empty_list;
        return 1;
    }

    return 0;
}

void SolRDevice::flushCommitBuffer()
{
    // This method can be used to batch commits if needed
}

// SolRStub *SolRDevice::getSolRStub() const
// {
//   return m_solrStub.get();
// }

const char **SolRDevice::getObjectSubtypes(ANARIDataType objectType)
{
    return query_object_types(objectType);
}

const void *SolRDevice::getObjectInfo(ANARIDataType objectType,
                                      const char *objectSubtype,
                                      const char *infoName,
                                      ANARIDataType infoType)
{
    return query_object_info(objectType, objectSubtype, infoType);
}

const void *SolRDevice::getParameterInfo(ANARIDataType objectType,
                                         const char *objectSubtype,
                                         const char *parameterName,
                                         ANARIDataType parameterType,
                                         const char *infoName,
                                         ANARIDataType infoType)
{
    return query_param_info(objectType, objectSubtype, parameterType, infoName);
}

} // namespace solr