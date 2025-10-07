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
#include "anari/backend/LibraryImpl.h"
#include "anari_library_solr_export.h"

namespace solr
{

const char **query_extensions();

struct SolRLibrary : public anari::LibraryImpl
{
    SolRLibrary(void *lib, ANARIStatusCallback defaultStatusCB,
                const void *statusCBPtr);

    ANARIDevice newDevice(const char *subtype) override;
    const char **getDeviceExtensions(const char *deviceType) override;
};

// Definitions ////////////////////////////////////////////////////////////////

SolRLibrary::SolRLibrary(void *lib, ANARIStatusCallback defaultStatusCB,
                         const void *statusCBPtr)
    : anari::LibraryImpl(lib, defaultStatusCB, statusCBPtr)
{
}

ANARIDevice SolRLibrary::newDevice(const char * /*subtype*/)
{
    return (ANARIDevice) new SolRDevice(this_library());
}

const char **SolRLibrary::getDeviceExtensions(const char * /*deviceType*/)
{
    return query_extensions();
}

} // namespace solr

// Define library entrypoint //////////////////////////////////////////////////

extern "C" SOLR_DEVICE_INTERFACE ANARI_DEFINE_LIBRARY_ENTRYPOINT(solr, handle,
                                                                 scb, scbPtr)
{
    return (ANARILibrary) new solr::SolRLibrary(handle, scb, scbPtr);
}

extern "C" SOLR_DEVICE_INTERFACE ANARIDevice
    makeSolRDevice(ANARIStatusCallback defaultCallback, const void *userPtr)
{
    return (ANARIDevice) new solr::SolRDevice(defaultCallback, userPtr);
}