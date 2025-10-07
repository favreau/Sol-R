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

include(GenerateExportHeader)
include(GNUInstallDirs)

# Set policy for file(GENERATE) behavior
cmake_policy(SET CMP0070 NEW)

## Establish project ##

project(anari_library_solr VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH} LANGUAGES CXX)

## Compilation setup ##

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN TRUE)

## Find dependencies ##

# We already have the solr library target available

## Build library target ##

set(SOURCES
  Object.cpp
  SolRDevice.cpp
  SolRLibrary.cpp
  Extensions.cpp
  SimpleQueries.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Ensure C++17 is properly enabled
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

generate_export_header(${PROJECT_NAME}
  EXPORT_MACRO_NAME "SOLR_DEVICE_INTERFACE"
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
PRIVATE
  ${PROJECT_SOURCE_DIR}/solr
  ${PROJECT_SOURCE_DIR}/generated
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  anari::anari
PRIVATE
  anari::helium
  # solr  # TODO: Re-enable after fixing Sol-R CUDA compilation issues
)

target_compile_definitions(${PROJECT_NAME}
PRIVATE
  SOLR_VERSION_MAJOR=${SolR_VERSION_MAJOR}
  SOLR_VERSION_MINOR=${SolR_VERSION_MINOR}
  SOLR_VERSION_PATCH=${SolR_VERSION_PATCH}
  solr_EXPORTS
)

# Force C++17 to override Sol-R's C++11 setting
target_compile_options(${PROJECT_NAME} PRIVATE -std=c++17)

## Query code generation ##
# Temporarily disable automatic generation due to C++ compatibility issues
# file(GLOB DEVICE_JSONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.json")
# foreach(file ${DEVICE_JSONS})
#   configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${file} ${CMAKE_CURRENT_BINARY_DIR}/${file} COPYONLY)
# endforeach()

# anari_generate_queries(
#   DEVICE_TARGET ${PROJECT_NAME}
#   CPP_NAMESPACE solr
#   JSON_ROOT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}
#   JSON_DEFINITIONS_FILE ${CMAKE_CURRENT_BINARY_DIR}/solr_device.json
#   JSON_EXTENSION_FILES ${DEVICE_JSONS}
# )

## Install library + headers ##

install(TARGETS ${PROJECT_NAME}
  EXPORT SolR_Exports
  LIBRARY DESTINATION lib
  # on Windows put the dlls into bin
  RUNTIME DESTINATION bin
  # ... and the import lib into the devel package
  ARCHIVE DESTINATION lib
)

install(
FILES
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_export.h
DESTINATION
  include/anari/ext/solr
)

install(EXPORT SolR_Exports
  DESTINATION lib/cmake/SolR
  NAMESPACE SolR::
) 