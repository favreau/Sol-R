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

#include "Utils.h"

// System
#ifdef WIN32
#include <windows.h>
#else
#include <dirent.h>
#endif // WIN32
#include <algorithm>

Strings getFilesFromFolder(const std::string& folder, const Strings& extensions)
{
    Strings filenames;
#ifdef WIN32
    // Textures
    for (Strings::const_iterator it = extensions.begin();
         it != extensions.end(); ++it)
    {
        std::string fullFilter(folder);
        fullFilter += "/*";
        fullFilter += (*it);

        WIN32_FIND_DATA FindData;
        HANDLE hFind = FindFirstFile(fullFilter.c_str(), &FindData);
        if (hFind != INVALID_HANDLE_VALUE)
            do
            {
                std::string fullPath(folder);
                fullPath += "/";
                fullPath += FindData.cFileName;
                filenames.push_back(fullPath);
            } while (FindNextFile(hFind, &FindData));
    }
#else
    DIR* dp;
    struct dirent* dirp;
    if ((dp = opendir(folder.c_str())) != NULL)
    {
        while ((dirp = readdir(dp)) != NULL)
        {
            const std::string path = dirp->d_name;

            bool ok = false;
            for (Strings::const_iterator it = extensions.begin();
                 !ok && it != extensions.end(); ++it)
                if (path.find(*it) != std::string::npos)
                    ok = true;

            if (ok)
            {
                std::string filename(folder);
                filename += "/";
                filename += dirp->d_name;
                filenames.push_back(filename);
            }
        }
        closedir(dp);
    }
#endif

    std::sort(filenames.begin(), filenames.end());
    return filenames;
}
