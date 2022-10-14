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

#include "../GPUKernel.h"

namespace solr
{
class SOLR_API CudaKernel : public GPUKernel
{
public:
    CudaKernel();
    ~CudaKernel();

    virtual void initBuffers();
    virtual void cleanup();

public:
    virtual void setPlatformId(const int) {}
    virtual void setDeviceId(const int) {}
    virtual void setKernelFilename(const std::string&) {}
    virtual void recompileKernels() {}

public:
    // ---------- Devices ----------
    void initializeDevice();
    void releaseDevice();

    virtual void reshape();

    virtual void queryDevice();

    void resetBoxesAndPrimitives();

public:
    // ---------- Rendering ----------
    void render_begin(const float timer);
    void render_end();

public:
    virtual std::string getGPUDescription();

public:
    void setBlockSize(int x, int y, int z)
    {
        m_blockSize.x = x;
        m_blockSize.y = y;
        m_blockSize.z = z;
    }
    void setSharedMemSize(int sharedMemSize)
    {
        m_sharedMemSize = sharedMemSize;
    }

private:
    // Runtime kernel execution parameters
    vec4i m_blockSize;
    int m_sharedMemSize;
};
} // namespace solr
