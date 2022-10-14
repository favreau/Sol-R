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

class TrefoilKnotScene : public Scene
{
public:
    TrefoilKnotScene(const std::string& name);
    ~TrefoilKnotScene(void);

protected:
    virtual void doInitialize();
    virtual void doAnimate();
    virtual void doAddLights();

private:
    void trefoilKnot(float R, float t, vec4f& p);
    void torus(float R, float t, vec4f& p);
    void star(float R, float t, vec4f& p);
    void spring(float R, float t, vec4f& p);
    void heart(float R, float u, float v, vec4f& p);
    void thing(float R, float t, vec4f a, vec4f& p);
    void moebius(float R, float u, float v, float s, float du, float dv,
                 vec4f& p);
};
