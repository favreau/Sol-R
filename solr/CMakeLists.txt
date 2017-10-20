include_directories(${PROJECT_SOURCE_DIR}/solr ${PROJECT_SOURCE_DIR}/generated)

SET(SOLR_SOURCES
    DLL_API.h
    Logging.h
    types.h
    Consts.h
    SolRStub.cpp
    SolRStub.h
    engines/GPUKernel.cpp
    engines/GPUKernel.h
    #engines/cpu/CPUKernel.cpp
    #engines/cpu/CPUKernel.h
    io/PDBReader.cpp
    io/PDBReader.h
    io/OBJReader.cpp
    io/OBJReader.h
    io/FileMarshaller.cpp
    io/SWCReader.cpp
    io/SWCReader.h
    io/FileMarshaller.cpp
    io/FileMarshaller.h
    images/ImageLoader.cpp
    images/ImageLoader.h
    images/jpge.cpp
    images/jpge.h
    images/jpgd.cpp
    images/jpgd.h
    images/tgad.cpp
    images/tgad.h
    opengl/rtgl.cpp
    opengl/rtgl.h
    games/MapReader.cpp
    games/MapReader.h
)

if (CUDA_FOUND)
    if(UNIX)
        set(CUDA_NVCC_USER_ARGS
            -std=c++11
            -D_FORCE_INLINES
            --maxrregcount=32
            --use_fast_math
            -gencode=arch=compute_30,code=sm_30
            -gencode=arch=compute_35,code=sm_35
            -gencode=arch=compute_50,code=sm_50
            -gencode=arch=compute_52,code=sm_52
            )
    else(UNIX)
        CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS 3.0 3.5 5.0 5.2 6.0 6.1)
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
                )
endif()

if(OPENCL_FOUND)
    ADD_LIBRARY(
		solr ${SOLR_LIBRARY_TYPE} 
		engines/opencl/OpenCLKernel.cpp 
                engines/opencl/RayTracer.cl
                ${SOLR_SOURCES})

    TARGET_LINK_LIBRARIES(
		solr
        ${OPENCL_LIBRARIES}
        ${OpenCL_LIBRARIES}
        ${FREEGLUT_LIBRARIES}
        ${OPENGL_gl_LIBRARY}
		${KINECT_LIBRARIES}
		${OCULUS_SDK_LIBRARIES}
		${SIXENSESDK_LIBRARIES}
        )
    # ================================================================================
    # Install kernels
    # ================================================================================
    INSTALL( FILES engines/opencl/RayTracer.cl DESTINATION bin/kernels )
endif()

# ================================================================================
# Install binaries
# ================================================================================
INSTALL(TARGETS solr 
    RUNTIME DESTINATION bin 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
