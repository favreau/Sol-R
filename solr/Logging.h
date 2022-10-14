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

#ifdef WIN32
#include <windows.h>
#ifdef LOGGING
#include <ETWLoggingModule.h>
#include <ETWResources.h>
#else
#include <sstream>
#endif
#else
#include <string>
#endif // WIN32

#include <iostream>
#include <time.h>

inline std::string getTimestamp()
{
#ifdef WIN32
    char buffer[80] = {'\0'};
    time_t now = time(&now);
    struct tm local_time;
    localtime_s(&local_time, &now);
    strftime(buffer, BUFSIZ, "%m/%d/%Y %H:%M:%S", &local_time);
    return std::string(buffer);
#else
    return "";
#endif // WIN32
}

#ifdef WIN32
#if 0
#define LOG_INFO(__level, __msg)                                         \
    {                                                                    \
        if (__level == 1)                                                \
            std::cout << getTimestamp() << " INFO  [" << __level << "] " \
                      << __msg << std::endl;                             \
    }
#define LOG_ERROR(__msg) \
    std::cout << getTimestamp() << " ERROR [1] " << __msg << std::endl;
#else
#define LOG_INFO(__level, __msg)                                  \
    {                                                             \
        if (__level == 1)                                         \
        {                                                         \
            std::stringstream __s;                                \
            __s << getTimestamp() << " [" << GetCurrentThreadId() \
                << "] [INFO]  " << __msg;                         \
            OutputDebugString(__s.str().c_str());                 \
            std::cout << "[INFO ] " << __s.str() << std::endl;    \
        }                                                         \
    }

#define LOG_ERROR(__msg)                                                      \
    {                                                                         \
        std::stringstream __s;                                                \
        __s << getTimestamp() << " [" << GetCurrentThreadId() << "] [ERROR] " \
            << __msg;                                                         \
        OutputDebugString(__s.str().c_str());                                 \
        std::cerr << "[ERROR] " << __s.str() << std::endl;                    \
    }
#endif // 0
#else
#define LOG_INFO(__level, __msg)                                            \
    {                                                                       \
        if (__level == 1)                                                   \
            std::cout << "INFO [" << __level << "] " << __msg << std::endl; \
    }
#define LOG_ERROR(__msg) std::cerr << "ERR  [F] " << __msg << std::endl;
#endif
