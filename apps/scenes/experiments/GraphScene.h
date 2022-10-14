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

class GraphScene : public Scene
{
public:
    GraphScene(const std::string& name);
    ~GraphScene(void);

protected:
    virtual void doInitialize();
    virtual void doAnimate();
    virtual void doAddLights();

private:
    void buildGraph(const bool update);
    void buildChart(const std::string& filename);
    void buildFrame();

private:
    float m_values[100][100];
    vec2i m_valueSize;
    vec4f m_graphSize;
    vec4f m_graphScale;
    vec4f m_graphCenter;

    int m_nbGraphElements;
    float* m_graphValues;
    int m_startGraph;
    float m_graphSpace;
    int m_graphMaterial;
    int m_graphObjectsPerBox;
};
