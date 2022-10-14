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

#include <engines/GPUKernel.h>

namespace solr
{
class SOLR_API FileMarshaller
{
public:
    FileMarshaller() {}
    ~FileMarshaller() {}

public:
    vec4f loadFromFile(GPUKernel &kernel, const std::string &filename,
                       const vec4f &center, const float scale);
    void saveToFile(GPUKernel &kernel, const std::string &filename);
};
} // namespace solr
