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

#include <solr.h>

#include <random>

#include <map>
#include <mutex>
#include <thread>

namespace solr
{
class RandomGenerator
{
public:
    /**
     * @brief Get the Instance object
     *
     * @return RandomGenerator Reference to the object
     */
    static RandomGenerator& getInstance()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_instance)
            _instance = new RandomGenerator();
        return *_instance;
    }

    /**
     * @brief Initialized the random frame buffer, a buffer with the size of the
     * image, filled with random numbers and used by the engine for launching
     * random rays. Random numbers are constantly processed in a dedicated
     * thread. The random frame buffer is transferred to the GPU before
     * rendering.
     *
     * @param multiplier Multiplier applied to every random number
     */
    void initialize(const float multiplier);

    /**
     * @brief Pauses the thread
     *
     */
    void pause();
    /**
     * @brief Resumes the thread
     *
     */
    void resume();

    /**
     * @brief Stops the thread and releases potential randomization devices
     *
     */
    void close();

    /**
     * @brief Set the Buffer object
     *
     * @param buffer Pointer to the buffer filled by the randomizer
     * @param nbFloats Number of floats in the buffer
     */
    void reshape(const size_t nbFloats);

    RandomBuffer* getBuffer() { return _buffer.data(); }

    static std::mutex _mutex;
    static RandomGenerator* _instance;

private:
    void _getFloats();

    ~RandomGenerator();

    std::thread* _thread{nullptr};
    std::vector<RandomBuffer> _buffer;
    float _multiplier;
    size_t _count{0};
    bool _paused{false};

    std::mutex _randomMutex;
};
} // namespace solr