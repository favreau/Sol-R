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

#include <consts.h>
#include <map>
#include <solr.h>

namespace solr
{
typedef struct
{
    unsigned char imageTypeCode;
    short int imageWidth;
    short int imageHeight;
    unsigned char bitCount;
} TGAFILE;

class ImageLoader
{
public:
    ImageLoader(void);
    ~ImageLoader(void);

public:
    // BITMAP
    bool loadBMP24(const int index, const std::string &filename, TextureInfo *textureInformations);

    // JPEG
    // https://code.google.com/p/jpeg-compressor
    bool loadJPEG(const int index, const std::string &filename, TextureInfo *textureInformations);

    // TGA
    bool loadTGA(const int index, const std::string &filename, TextureInfo *textureInformations);
};
}
