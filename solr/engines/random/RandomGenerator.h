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

#include <solr_defines.h>

#if USE_RANDOM_DEVICE
// WAITING TO BE VALIDATED BY HARDWARE PROVIDER
#else
#include <random>
#endif

#include <map>
#include <mutex>
#include <vector>

class RandomGenerator
{
public:
    /**
     * @brief Get the Instance object
     *
     * @return GeneralSettings* Pointer to the object
     */
    static RandomGenerator& getInstance()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_instance)
            _instance = new RandomGenerator();
        return *_instance;
    }

    void initialize();

    std::vector<float> getFloats(const size_t nbFloats, const float multiplier);

    static std::mutex _mutex;
    static RandomGenerator* _instance;

private:
    RandomGenerator();
    ~RandomGenerator();

#if USE_RANDOM_DEVICE
    // WAITING TO BE VALIDATED BY HARDWARE PROVIDER
#endif

    bool _initialized{false};
};