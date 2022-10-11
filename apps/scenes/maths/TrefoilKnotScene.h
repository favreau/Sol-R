/* Copyright (c) 2011-2014, Cyrille Favreau
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Cyrille Favreau <cyrille_favreau@hotmail.com>
 *
 * This file is part of Sol-R <https://github.com/cyrillefavreau/Sol-R>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
