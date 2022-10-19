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

cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME SolR)
project(${PROJECT_NAME})

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 1)
set(${PROJECT_NAME}_VERSION_PATCH 0)

set(CPACK_PACKAGE_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${${PROJECT_NAME}_VERSION_PATCH}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# ================================================================================
# Project
# ================================================================================

option(SOLR_RANDOM_DEVICE_ENABLED "Activate Random Device Generator" OFF)
option(SOLR_KINECT_ENABLED "Activate Kinect" OFF)
option(SOLR_OCULUS_ENABLED "Activate Oculus" OFF)
option(SOLR_SIXENSE_ENABLED "Activate Sixense Controller" OFF)
option(SOLR_LEAPMOTION_ENABLED "Activate Leap Motion Controller" OFF)
option(SOLR_DEFLECT_ENABLED "Activate Deflect streaming" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif(NOT CMAKE_BUILD_TYPE)

# Engine
set(SOLR_ENGINE "OPENCL" CACHE STRING "engine use (CUDA or OPENCL)")
set_property(CACHE SOLR_ENGINE PROPERTY STRINGS CUDA OPENCL)

# Library type
set(SOLR_LIBRARY_TYPE "STATIC" CACHE STRING "solr library type (STATIC or SHARED)")
set_property(CACHE SOLR_LIBRARY_TYPE PROPERTY STRINGS STATIC SHARED)

# Windows' math include does not define constants by default.
# Set this definition so it does.
# Also set NOMINMAX so the min and max functions are not overwritten with macros.
if(MSVC)
    add_compile_options(-D_USE_MATH_DEFINES -DNOMINMAX -std=c++11)
else(MSVC)
    # __STRICT_ANSI__ is required by OpenCL in order to access x,y,z.. member of
    # cl_float2/3/.. data types. Took me a little while to find out...
    add_compile_options(-fPIC -std=c++11 -U__STRICT_ANSI__)
    if(APPLE)
        add_compile_options(-Wno-c++11-extensions -Wno-narrowing -Wno-deprecated-declarations
            -Wno-macro-redefined -Wno-shift-negative-value -Wno-comment)
    endif(APPLE)
endif(MSVC)

# ================================================================================
# GL
# ================================================================================
find_package(OpenGL REQUIRED SYSTEM)
if (OpenGL_FOUND)
    message(STATUS "OpenGL found and selected for build")
    include_directories(${OPENGL_INCLUDE_DIR})
else(OpenGL_FOUND)
    message(ERROR " OpenGL not found!")
endif(OpenGL_FOUND)

# ================================================================================
# GLUT
# ================================================================================
find_package(FreeGLUT REQUIRED SYSTEM)
if (FREEGLUT_FOUND)
    message(STATUS "FreeGlut found and selected for build")
    include_directories(${FREEGLUT_INCLUDE_DIR})
else(FREEGLUT_FOUND)
    message(ERROR " FreeGlut not found!")
endif(FREEGLUT_FOUND)
	
# ================================================================================
# OpenCL 1.2
# ================================================================================
if (${SOLR_ENGINE} STREQUAL "OPENCL")
    set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS TRUE)
    find_package(OpenCL REQUIRED SYSTEM)
    if (OPENCL_FOUND)
        list(APPEND FIND_PACKAGES_DEFINES USE_OPENCL)
        message(STATUS "OpenCL found and selected for build")
        include_directories(${OPENCL_INCLUDE_DIRS})
    else(OPENCL_FOUND)
        message(ERROR " OpenCL not found. Project will not be built with that technology")
    endif(OPENCL_FOUND)
endif()

# ================================================================================
# CUDA 8.0
# ================================================================================
if (${SOLR_ENGINE} STREQUAL "CUDA")
    find_package(CUDA REQUIRED SYSTEM)
    if (CUDA_FOUND)
        list(APPEND FIND_PACKAGES_DEFINES USE_CUDA)
        message(STATUS "CUDA found and selected for build")
        include_directories(${CUDA_TOOLKIT_INCLUDE})
        if (MSVC)
            set(EXTRA_CXX_FLAGS "/DVERBOSE /D_CRT_SECURE_NO_WARNINGS")
        else(MSVC)
            if (APPLE)
                set(EXTRA_CXX_FLAGS "-DVERBOSE -msse2")
            else(APPLE)
                set(EXTRA_CXX_FLAGS "-DVERBOSE -msse2 -std=c++0x ")
            endif(APPLE)
        endif(MSVC)
    else(CUDA_FOUND)
        message(ERROR " CUDA not found. Project will not be built with that technology")
    endif(CUDA_FOUND)
endif()

# ================================================================================
# OpenMP
# ================================================================================
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

# ================================================================================
# Random Generation device
# ================================================================================
if(SOLR_RANDOM_DEVICE_ENABLED)
    # TODO: Find package of random device library and add includes/libs
    # include_directories(${ ... _INCLUDE_DIR})
    # list(APPEND FIND_PACKAGES_DEFINES USE_RANDOM_DEVICE)
endif(SOLR_RANDOM_DEVICE_ENABLED)

# ================================================================================
# KINECT 1.8
# ================================================================================
if(SOLR_KINECT_ENABLED)
    find_package(KINECT REQUIRED SYSTEM)
    if (KINECT_FOUND)
        message(STATUS "Kinect found and selected for build")
        include_directories(${KINECT_INCLUDE_DIR})
        list(APPEND FIND_PACKAGES_DEFINES USE_KINECT)
    else(KINECT_FOUND)
        message(ERROR " KINECT not found!")
    endif(KINECT_FOUND)
endif(SOLR_KINECT_ENABLED)

# ================================================================================
# OCULUS 0.2.5
# ================================================================================
if(SOLR_OCULUS_ENABLED)
    find_package(OculusSDK REQUIRED SYSTEM)
    if (OCULUS_SDK_FOUND)
        message(STATUS "Oculus found and selected for build")
        include_directories(${OCULUS_SDK_INCLUDE_DIR})
        list(APPEND FIND_PACKAGES_DEFINES USE_OCULUS)
    else(OCULUS_SDK_FOUND)
        message(ERROR " OCULUS not found!")
    endif(OCULUS_SDK_FOUND)
endif(SOLR_OCULUS_ENABLED)

# ================================================================================
# Sixense SDK
# ================================================================================
if(SOLR_SIXENSE_ENABLED)
    find_package(SixenseSdk REQUIRED SYSTEM)
    if (SIXENSESDK_FOUND)
        message(STATUS "Sixense found and selected for build")
        include_directories(${SIXENSESDK_INCLUDE_DIR})
        list(APPEND FIND_PACKAGES_DEFINES USE_SIXENSE)
        if(MSVC)
            set(SIXENSESDK_DLLS
                ${SIXENSESDK_sixense_LIBRARY_RELEASE}
                ${SIXENSESDK_sixense_utils_LIBRARY_RELEASE})
            foreach(SIXENSESDK_DLL ${SIXENSESDK_DLLS})
                if (SIXENSESDK_DLL MATCHES "_x64.lib")
                    string(REPLACE ".lib" ".dll" SIXENSESDK_DLL ${SIXENSESDK_DLL})
                    string(REPLACE "lib" "bin" SIXENSESDK_DLL ${SIXENSESDK_DLL})
                    install( FILES ${SIXENSESDK_DLL} DESTINATION bin )
                endif()
            endforeach()
        endif(MSVC)
    else(SIXENSESDK_FOUND)
        message(ERROR " Sixense not found!")
    endif(SIXENSESDK_FOUND)
endif(SOLR_SIXENSE_ENABLED)

# ================================================================================
# Leap Motion SDK 3.2.0
# ================================================================================
if(UNIX)
  set(LEAP_LIBRARY "")
endif(UNIX)
if(SOLR_LEAPMOTION_ENABLED)
    find_package(Leap REQUIRED SYSTEM)
    if (LEAP_FOUND)
        message(STATUS "Leap Motion found and selected for build")
        include_directories(${LEAP_INCLUDE_DIR})
        list(APPEND FIND_PACKAGES_DEFINES USE_LEAPMOTION)
        if(MSVC)
            string(REPLACE ".lib" ".dll" LEAP_DLL ${LEAP_LIBRARY})
            install( FILES ${LEAP_DLL} DESTINATION bin )
        endif(MSVC)
    else(LEAP_FOUND)
        message(ERROR " Leap Motion not found!")
    endif(LEAP_FOUND)
endif(SOLR_LEAPMOTION_ENABLED)

# ================================================================================
# Deflect
# ================================================================================
if(SOLR_DEFLECT_ENABLED)
    find_package(Deflect REQUIRED SYSTEM)
    if (Deflect_FOUND)
        message(STATUS "Deflect found and selected for build")
        include_directories(${DEFLECT_INCLUDE_DIR})
        list(APPEND FIND_PACKAGES_DEFINES USE_DEFLECT)
    else(Deflect_FOUND)
        message(ERROR " Deflect not found!")
    endif(Deflect_FOUND)
endif(SOLR_DEFLECT_ENABLED)

# ================================================================================
# Project
# ================================================================================
add_subdirectory(solr)
add_subdirectory(apps)

# ================================================================================
# Write defines.h and options.cmake
# ================================================================================
if(NOT OPTIONS_CMAKE)
    set(OPTIONS_CMAKE ${CMAKE_CURRENT_BINARY_DIR}/options.cmake)
endif(NOT OPTIONS_CMAKE)
set(DEFINES_FILE "${PROJECT_SOURCE_DIR}/${PROJECT_INCLUDE_NAME}/generated/solr_defines.h")
file(WRITE ${DEFINES_FILE}
	"// generated by CMake, do not edit.\n\n"
	"#ifndef ${PROJECT_NAME}_DEFINES_${SYSTEM}_H\n"
	"#define ${PROJECT_NAME}_DEFINES_${SYSTEM}_H\n\n")
file(WRITE ${OPTIONS_CMAKE} "# Optional modules enabled during build\n")
foreach(DEF ${FIND_PACKAGES_DEFINES})
	add_definitions(-D${DEF}=1)
	file(APPEND ${DEFINES_FILE}
	"#ifndef ${DEF}\n"
	"#  define ${DEF} 1\n"
	"#endif\n")
	if(NOT DEF STREQUAL SYSTEM)
		file(APPEND ${OPTIONS_CMAKE} "set(${DEF} ON)\n")
	endif()
endforeach()

string(REPLACE "\\" "/" SOLR_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
file(APPEND ${DEFINES_FILE} "\nstatic const char* DEFAULT_KERNEL_FILENAME=\"${SOLR_INSTALL_PREFIX}/bin/kernels/RayTracer.cl\";\n")
file(APPEND ${DEFINES_FILE} "\nstatic const char* DEFAULT_MEDIA_FOLDER=\"${SOLR_INSTALL_PREFIX}/bin\";\n")
file(APPEND ${DEFINES_FILE} "\n#endif\n")
