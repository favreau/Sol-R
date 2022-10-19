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

namespace solr
{

RandomGenerator* RandomGenerator::_instance = nullptr;
std::mutex RandomGenerator::_mutex;

RandomGenerator::~RandomGenerator()
{
    close();
}

void RandomGenerator::close()
{
    if (_thread)
    {
        delete _thread;
        _thread = nullptr;
    }
}

void RandomGenerator::initialize(const float multiplier)
{
    _multiplier = multiplier;
    _count = 0;
    _thread = new std::thread(&RandomGenerator::_getFloats, this);
    if (_thread)
    {
        LOG_INFO(1, "Thread for random numbers is up and running!");
        _thread->detach();
    }
}

void RandomGenerator::reshape(const size_t nbFloats)
{
    LOG_INFO(1, "Reshaping random generator to " << nbFloats << " floats");
    _randomMutex.lock();
    _buffer.resize(nbFloats);
    _randomMutex.unlock();
}

void RandomGenerator::_getFloats()
{
    while (true)
    {
        if (_paused)
            continue;
        _randomMutex.lock();
        LOG_INFO(3, "Fetching " << _buffer.size() << " from CPU");
        for (size_t i = 0; i < _buffer.size() && !_paused; ++i)
            _buffer[i] = _multiplier * (rand() % 200 - 100) / 100.f;
        _randomMutex.unlock();
    }
}

void RandomGenerator::pause()
{
    _paused = true;
    // Wait for thread to complete current loop
    _randomMutex.lock();
    _randomMutex.unlock();
}

void RandomGenerator::resume()
{
    _paused = false;
    _randomMutex.unlock();
}

} // namespace solr