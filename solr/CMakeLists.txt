# Copyright (c) 2011-2022, Cyrille Favreau
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

include_directories(${PROJECT_SOURCE_DIR}/solr ${PROJECT_SOURCE_DIR}/generated)

SET(SOLR_SOURCES
    SolRStub.cpp
    engines/random/RandomGenerator.cpp
    engines/GPUKernel.cpp
    #engines/cpu/CPUKernel.cpp
    io/PDBReader.cpp
    io/OBJReader.cpp
    io/SWCReader.cpp
    io/FileMarshaller.cpp
    images/ImageLoader.cpp
    images/jpge.cpp
    images/jpgd.cpp
    images/tgad.cpp
    opengl/rtgl.cpp
    games/MapReader.cpp
)

SET(SOLR_HEADERS
    Logging.h
    SolRStub.h
    engines/random/RandomGenerator.h
    engines/GPUKernel.h
    #engines/cpu/CPUKernel.h
    io/PDBReader.h
    io/OBJReader.h
    io/SWCReader.h
    io/FileMarshaller.h
    images/ImageLoader.h
    images/jpge.h
    images/jpgd.h
    images/tgad.h
    opengl/rtgl.h
    games/MapReader.h
)

SET(SOLR_PUBLIC_HEADERS
    ../generated/solr_defines.h
    engines/GPUKernel.h
    api.h
    solr.h
    consts.h
)

SET(SOLR_ENGINE_PUBLIC_HEADERS
    engines/GPUKernel.h
)

unset(SOLR_ENGINE_LIBRARIES)

# TODO: Include random number device library
#if( ... _FOUND)
#  SET(SOLR_ENGINE_LIBRARIES ${ ... _LIBRARY})
#endif( ... _FOUND)

if (CUDA_FOUND)
    if(UNIX)
        set(CUDA_NVCC_USER_ARGS
            -std=c++11
            -D_FORCE_INLINES
            --compiler-options "-fPIC"
            --maxrregcount=32
            --use_fast_math
            -gencode=arch=compute_52,code=sm_52
            -gencode=arch=compute_60,code=sm_60
            -gencode=arch=compute_70,code=sm_70
            -gencode=arch=compute_80,code=sm_80
            )
    else(UNIX)
        CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS 3.0 3.5 5.0 5.2 6.0 6.1 7.0 8.0 8.9)
        set(CUDA_NVCC_USER_ARGS -D_FORCE_INLINES --maxrregcount=32 --use_fast_math)
    endif(UNIX)

    set(CUDA_INCLUDE_USER_DIRECTORIES -I${PROJECT_SOURCE_DIR}/solr -I${PROJECT_SOURCE_DIR}/generated)
    set(CUDA_64_BIT_DEVICE_CODE ON)
    set(CUDA_VERBOSE_BUILD OFF)
    if (KINECT_FOUND)
        list(APPEND CUDA_NVCC_USER_ARGS -DUSE_KINECT)
    endif()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} ${CUDA_INCLUDE_USER_DIRECTORIES} ${CUDA_NVCC_USER_ARGS})
    CUDA_COMPILE(
		CUDARAYTRACER_O engines/cuda/CudaRayTracer.cu)
    CUDA_ADD_LIBRARY(
		solr ${SOLR_LIBRARY_TYPE}
		engines/cuda/CudaKernel.cpp
		engines/cuda/CudaKernel.h
		engines/cuda/CudaRayTracer.h
		engines/cuda/TextureMapping.cuh
		engines/cuda/VectorUtils.cuh
		engines/cuda/GeometryIntersections.cuh
		engines/cuda/GeometryShaders.cuh
		${CUDARAYTRACER_O}
		${SOLR_SOURCES}
		)
    TARGET_LINK_LIBRARIES(
		solr
		${FREEGLUT_LIBRARIES}
		${OPENGL_gl_LIBRARY}
		${KINECT_LIBRARIES}
		${OCULUS_SDK_LIBRARIES}
		${SIXENSESDK_LIBRARIES}
        ${SOLR_ENGINE_LIBRARIES}
    )
    list(APPEND SOLR_ENGINE_PUBLIC_HEADERS engines/cuda/CudaKernel.h)
endif()

if(OPENCL_FOUND)
    ADD_LIBRARY(
		solr ${SOLR_LIBRARY_TYPE} 
		engines/opencl/OpenCLKernel.cpp 
                engines/opencl/RayTracer.cl
                ${SOLR_SOURCES})

    # __STRICT_ANSI__ needs to be undefined for OpenCL to access x,y,z members of cl_float2/3/.. types
    target_compile_options(solr PRIVATE -U__STRICT_ANSI__)

    TARGET_LINK_LIBRARIES(
		solr
        ${OPENCL_LIBRARIES}
        ${OpenCL_LIBRARIES}
        ${FREEGLUT_LIBRARIES}
        ${OPENGL_gl_LIBRARY}
		${KINECT_LIBRARIES}
		${OCULUS_SDK_LIBRARIES}
		${SIXENSESDK_LIBRARIES}
        ${SOLR_ENGINE_LIBRARIES}
    )

    # ================================================================================
    # Install kernels
    # ================================================================================
    INSTALL( FILES engines/opencl/RayTracer.cl DESTINATION bin/kernels )

    list(APPEND SOLR_ENGINE_PUBLIC_HEADERS engines/opencl/OpenCLKernel.h)
endif()

# ================================================================================
# Install binaries
# ================================================================================
INSTALL(TARGETS solr 
    RUNTIME DESTINATION bin 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
# Install headers
INSTALL( FILES ${SOLR_PUBLIC_HEADERS} DESTINATION include/solr )
INSTALL( FILES ${SOLR_ENGINE_PUBLIC_HEADERS} DESTINATION include/solr/engines )
