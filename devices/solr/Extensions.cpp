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

namespace solr
{

const char **query_extensions()
{
    static const char *extensions[] = {"KHR_CAMERA_ORTHOGRAPHIC",
                                       "KHR_CAMERA_PERSPECTIVE",
                                       "KHR_DEVICE_SYNCHRONIZATION",
                                       "KHR_FRAME_ACCUMULATION",
                                       "KHR_FRAME_COMPLETION_CALLBACK",
                                       "KHR_GEOMETRY_TRIANGLE",
                                       "KHR_GEOMETRY_SPHERE",
                                       "KHR_GEOMETRY_CYLINDER",
                                       "KHR_GEOMETRY_CONE",
                                       "KHR_INSTANCE_TRANSFORM",
                                       "KHR_LIGHT_DIRECTIONAL",
                                       "KHR_LIGHT_POINT",
                                       "KHR_LIGHT_SPOT",
                                       "KHR_MATERIAL_MATTE",
                                       "KHR_MATERIAL_PHYSICALLY_BASED",
                                       "KHR_RENDERER_AMBIENT_LIGHT",
                                       "KHR_RENDERER_BACKGROUND_COLOR",
                                       "KHR_SAMPLER_IMAGE2D",
                                       "KHR_SAMPLER_PRIMITIVE",
                                       nullptr};
    return extensions;
}

} // namespace solr