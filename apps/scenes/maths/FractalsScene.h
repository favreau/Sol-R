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

#include <scenes/Scene.h>

class FractalsScene : public Scene
{
public:
    FractalsScene(const std::string& name);
    ~FractalsScene(void);

protected:
    virtual void doInitialize();
    virtual void doAnimate();
    virtual void doAddLights();

private:
    void createFractals(float maxIterations, const vec4f& center, int material);
    vec4f MandelBox(vec3f V, const vec3f& Scale, float R, float S, float C);
    float DE(vec3f pos, const int iterations, const vec2f params);
    bool isSierpinskiCarpetPixelFilled(int i, vec3i v);
};
