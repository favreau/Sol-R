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

#include "anari/anari.h"

namespace solr
{

const char **query_object_types(ANARIDataType type)
{
    static const char *cameras[] = {"perspective", "orthographic", nullptr};
    static const char *geometries[] = {"triangle", "sphere", nullptr};
    static const char *lights[] = {"directional", "point", nullptr};
    static const char *materials[] = {"matte", nullptr};
    static const char *renderers[] = {"raycast", "pathtracer", nullptr};
    static const char *empty[] = {nullptr};

    switch (type)
    {
    case ANARI_CAMERA:
        return cameras;
    case ANARI_GEOMETRY:
        return geometries;
    case ANARI_LIGHT:
        return lights;
    case ANARI_MATERIAL:
        return materials;
    case ANARI_RENDERER:
        return renderers;
    default:
        return empty;
    }
}

const void *query_object_info(ANARIDataType type, const char *name,
                              ANARIDataType infoType)
{
    // Simple implementation - just return nullptr for now
    return nullptr;
}

const void *query_param_info(ANARIDataType type, const char *name,
                             ANARIDataType paramType, const char *infoType)
{
    // Simple implementation - just return nullptr for now
    return nullptr;
}

} // namespace solr