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

const size_t MaxDimensions = 7;

struct Matrix
{
    float values[MaxDimensions][MaxDimensions];
};

typedef std::vector<float> Vector;

class HypercubeScene : public Scene
{
public:
    HypercubeScene(const std::string& name);
    ~HypercubeScene(void);

protected:
    virtual void doInitialize();
    virtual void doAnimate();
    virtual void doAddLights();

private:
    float powerOfTwo(const size_t n);
    void createGeometry();
    void makeSubFaces(const size_t a, const size_t b, size_t& c,
                      const size_t n);
    void makeMatrix();
    Matrix identityMatrix();
    Matrix multiplyMatrices(const Matrix&, const Matrix&);
    Matrix makeRotationMatrix(const size_t ID, const size_t IDD,
                              const float angle);

    Vector cross(const Vector& u, const Vector& v);
    Vector multiplyVector(const Matrix& m, const Vector& v);
    Vector subtractVector(const Vector& u, const Vector& v);

    void drawFace(size_t Id);
    void drawFaces();

private:
    size_t _dimension;
    std::map<size_t, Vector> _vertices;
    Matrix _matrix;
    std::vector<float> _angles;
    int _primitiveIndex;
    float _scale;
    float _transparency;
    float _reflection;
    float _explosionFactor;
};
