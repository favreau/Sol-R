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

#include "RandomGenerator.h"

#include "Logging.h"

#ifdef USE_RANDOM_DEVICE
#include <iomanip>
#endif

RandomGenerator* RandomGenerator::_instance = nullptr;
std::mutex RandomGenerator::_mutex;

RandomGenerator::RandomGenerator() {}

RandomGenerator::~RandomGenerator() {}

void RandomGenerator::initialize()
{
#if USE_RANDOM_DEVICE
    // WAITING TO BE VALIDATED BY HARDWARE PROVIDER
#endif
}

std::vector<float> RandomGenerator::getFloats(const size_t nbFloats,
                                              const float multiplier)
{
    std::vector<float> floats;
    floats.resize(nbFloats);
#if USE_RANDOM_DEVICE
    // WAITING TO BE VALIDATED BY HARDWARE PROVIDER
#else
    for (int i = 0; i < nbFloats; ++i)
        floats[i] = multiplier * (rand() % 200 - 100) / 100.f;
#endif
    return floats;
}
