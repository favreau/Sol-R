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

#include <iostream>

#include "../Logging.h"
#include "../consts.h"

#include "ImageLoader.h"

#include "jpgd.h"
#include "tgad.h"

namespace solr
{
#ifndef WIN32
#define uint16 unsigned short
#define uint32 unsigned long
#define uint8 unsigned char
typedef struct
{
    uint16 bfType;      // specifies the file type "BM" 0x424d
    uint32 bfSize;      // specifies the size in bytes of the bitmap file
    uint16 bfReserved1; // reserved; must be 0
    uint16 bfReserved2; // reserved; must be 0
    uint32 bfOffBits;
} BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
    uint32 biSize;
    uint32 biWidth;
    uint32 biHeight;
    uint16 biPlanes;
    uint16 biBitCount;
    uint32 biCompression;
    uint32 biSizeImage;
    uint32 biXPelsPerMeter;
    uint32 biYPelsPerMeter;
    uint32 biClrUsed;
    uint32 biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;
#endif // WIN32

ImageLoader::ImageLoader(void) {}

ImageLoader::~ImageLoader(void) {}

bool ImageLoader::loadBMP24(const int index, const std::string &filename,
                            TextureInfo *textureInformations)
{
    FILE *filePtr(0);                  // our file pointer
    BITMAPFILEHEADER bitmapFileHeader; // our bitmap file header
    BITMAPINFOHEADER bitmapInfoHeader;
    unsigned int imageIdx = 0; // image index counter
    char tempRGB;              // our swap variable

// open filename in read binary mode
#ifdef WIN32
    fopen_s(&filePtr, filename.c_str(), "rb");
#else
    filePtr = fopen(filename.c_str(), "rb");
#endif
    if (filePtr == NULL)
    {
        LOG_ERROR("Failed to load " << filename);
        return false;
    }

    // read the bitmap file header
    size_t status =
        fread(&bitmapFileHeader, sizeof(BITMAPFILEHEADER), 1, filePtr);

    // verify that this is a bmp file by check bitmap id
    if (bitmapFileHeader.bfType != 0x4D42)
    {
        LOG_ERROR("Failed to load " << filename << ", wrong bitmap id");
        fclose(filePtr);
        return false;
    }

    // read the bitmap info header
    status = fread(&bitmapInfoHeader, sizeof(BITMAPINFOHEADER), 1, filePtr);

    // move file point to the begging of bitmap data
    fseek(filePtr, bitmapFileHeader.bfOffBits, SEEK_SET);

    unsigned int bitmapSize = bitmapInfoHeader.biWidth *
                              bitmapInfoHeader.biHeight *
                              bitmapInfoHeader.biBitCount / 8;

    // Add Texture to CPU Memory
    if (textureInformations[index].buffer != 0)
    {
        delete[] textureInformations[index].buffer;
        LOG_INFO(1, "Replacing existing texture " << index);
    }
    textureInformations[index].buffer = new unsigned char[bitmapSize];
    textureInformations[index].offset = 0;
    textureInformations[index].size.x = bitmapInfoHeader.biWidth;
    textureInformations[index].size.y = bitmapInfoHeader.biHeight;
    textureInformations[index].size.z = bitmapInfoHeader.biBitCount / 8;

    // verify memory allocation
    if (!textureInformations[index].buffer)
    {
        free(textureInformations[index].buffer);
        fclose(filePtr);
        LOG_ERROR("Failed to load " << filename
                                    << ", invalid memory allocation");
        return false;
    }

    // read in the bitmap image data
    status = fread(textureInformations[index].buffer, bitmapSize, 1, filePtr);

    // make sure bitmap image data was read
    if (textureInformations[index].buffer == NULL)
    {
        fclose(filePtr);
        LOG_ERROR("Failed to load " << filename
                                    << ", bitmap image could not be read");
        return false;
    }

    // swap the r and b values to get RGB (bitmap is BGR)
    for (imageIdx = 0; imageIdx < bitmapSize; imageIdx += 3)
    {
        tempRGB = textureInformations[index].buffer[imageIdx];
        textureInformations[index].buffer[imageIdx] =
            textureInformations[index].buffer[imageIdx + 2];
        textureInformations[index].buffer[imageIdx + 2] = tempRGB;
    }

    // close file and return bitmap image data
    fclose(filePtr);

    LOG_INFO(3,
             "[" << index << "] Successfully loaded " << filename << std::endl
                 << "biSize         : " << bitmapInfoHeader.biSize << std::endl
                 << "biWidth        : " << bitmapInfoHeader.biWidth << std::endl
                 << "biHeight       : " << bitmapInfoHeader.biHeight
                 << std::endl
                 << "biPlanes       : " << bitmapInfoHeader.biPlanes
                 << std::endl
                 << "biBitCount     : " << bitmapInfoHeader.biBitCount
                 << std::endl
                 << "biCompression  : " << bitmapInfoHeader.biCompression
                 << std::endl
                 << "biSizeImage    : " << bitmapSize << "/"
                 << bitmapInfoHeader.biSizeImage << std::endl
                 << "biXPelsPerMeter: " << bitmapInfoHeader.biXPelsPerMeter
                 << std::endl
                 << "biXPelsPerMeter: " << bitmapInfoHeader.biYPelsPerMeter);

    LOG_INFO(3, "Slot " << index << ": Successfully loaded texture " << filename
                        << " (" << textureInformations[index].size.x << ","
                        << textureInformations[index].size.y << ","
                        << textureInformations[index].size.z << ")");

    return true;
}

bool ImageLoader::loadJPEG(const int index, const std::string &filename,
                           TextureInfo *textureInformations)
{
    int width, height, actual_comps, req_comps(3);
    BitmapBuffer *buffer =
        jpgd::decompress_jpeg_image_from_file(filename.c_str(), &width, &height,
                                              &actual_comps, req_comps);

    if (buffer != 0)
    {
#if 1
        // Vertical Flip
        size_t size = width * height * actual_comps;
        BitmapBuffer *revBuffer = new BitmapBuffer[size];
        for (size_t i(0); i < size; i += actual_comps)
        {
            revBuffer[i + 2] = buffer[size - 1 - i];
            revBuffer[i + 1] = buffer[size - 2 - i];
            revBuffer[i] = buffer[size - 3 - i];
        }
        delete[] buffer;
#endif

        if (textureInformations[index].buffer != 0)
            delete[] textureInformations[index].buffer;
#if 1
        textureInformations[index].buffer = revBuffer;
#else
        textureInformations[index].buffer = buffer;
#endif
        textureInformations[index].size.x = width;
        textureInformations[index].size.y = height;
        textureInformations[index].size.z = actual_comps;
        textureInformations[index].offset = 0;

        LOG_INFO(3, "Slot " << index << ": Successfully loaded texture "
                            << filename << " ("
                            << textureInformations[index].size.x << ","
                            << textureInformations[index].size.y << ","
                            << textureInformations[index].size.z << ")");

        return true;
    }
    return false;
}

bool ImageLoader::loadTGA(const int index, const std::string &filename,
                          TextureInfo *textureInformations)
{
#if 0
    TGAFILE tgaFile;

    FILE* filePtr;
    unsigned char ucharBad;
    short int sintBad;
    uint32 imageSize;
    int colorMode;
    unsigned char colorSwap;

    // Open the TGA file.
    filePtr = fopen(filename.c_str(), "rb");
    if (filePtr == NULL)
    {
        return false;
    }

    // Read the two first bytes we don't need.
    fread(&ucharBad, sizeof(unsigned char), 1, filePtr);
    fread(&ucharBad, sizeof(unsigned char), 1, filePtr);

    // Which type of image gets stored in imageTypeCode.
    fread(&tgaFile.imageTypeCode, sizeof(unsigned char), 1, filePtr);

#if 0
    // For our purposes, the type code should be 2 (uncompressed RGB image)
    // or 3 (uncompressed black-and-white images).
    if (tgaFile.imageTypeCode != 2 && tgaFile.imageTypeCode != 3)
    {
        fclose(filePtr);
        return false;
    }
#endif // 0

    // Read 13 bytes of data we don't need.
    fread(&sintBad, sizeof(short int), 1, filePtr);
    fread(&sintBad, sizeof(short int), 1, filePtr);
    fread(&ucharBad, sizeof(unsigned char), 1, filePtr);
    fread(&sintBad, sizeof(short int), 1, filePtr);
    fread(&sintBad, sizeof(short int), 1, filePtr);

    // Read the image's width and height.
    fread(&tgaFile.imageWidth, sizeof(short int), 1, filePtr);
    fread(&tgaFile.imageHeight, sizeof(short int), 1, filePtr);

    // Read the bit depth.
    fread(&tgaFile.bitCount, sizeof(unsigned char), 1, filePtr);

    // Read one byte of data we don't need.
    fread(&ucharBad, sizeof(unsigned char), 1, filePtr);

    // Color mode -> 3 = BGR, 4 = BGRA.
    colorMode = tgaFile.bitCount / 8;
    imageSize = tgaFile.imageWidth * tgaFile.imageHeight * colorMode;

    // Allocate memory for the image data.
    textureInformations[index].buffer = (unsigned char*)malloc(sizeof(unsigned char)*imageSize);;
    textureInformations[index].offset = 0;
    textureInformations[index].size.x = tgaFile.imageWidth;
    textureInformations[index].size.y = tgaFile.imageHeight;
    textureInformations[index].size.z = tgaFile.bitCount/8;

    // Read the image data.
    fread(textureInformations[index].buffer, sizeof(unsigned char), imageSize, filePtr);

    // Change from BGR to RGB so OpenGL can read the image data.
    for (int imageIdx = 0; imageIdx < imageSize; imageIdx += colorMode)
    {
        colorSwap = textureInformations[index].buffer[imageIdx];
        textureInformations[index].buffer[imageIdx] = textureInformations[index].buffer[imageIdx + 2];
        textureInformations[index].buffer[imageIdx + 2] = colorSwap;
    }
    fclose(filePtr);
#else
    Texture texture;
    LoadTGA(&texture, const_cast<char *>(filename.c_str()));

#if 0
    //Vertical Flip
    size_t colorDepth=texture.bpp/8;
    size_t size=texture.width*texture.height*colorDepth;
    BitmapBuffer* revBuffer=new BitmapBuffer[size];
    for (size_t i(0); i<size; i+=colorDepth)
    {
        revBuffer[i+2] = texture.imageData[size-1-i];
        revBuffer[i+1] = texture.imageData[size-2-i];
        revBuffer[i  ] = texture.imageData[size-3-i];
    }

    if(textureInformations[index].buffer!=NULL) delete [] textureInformations[index].buffer;
    memcpy(texture.imageData,revBuffer,size);
    delete [] revBuffer;
#endif // 0

    textureInformations[index].buffer = texture.imageData;
    textureInformations[index].size.x = texture.width;
    textureInformations[index].size.y = texture.height;
    textureInformations[index].size.z = texture.bpp / 8;
#endif

    LOG_INFO(3, "Slot " << index << ": Successfully loaded texture " << filename
                        << " (" << textureInformations[index].size.x << ","
                        << textureInformations[index].size.y << ","
                        << textureInformations[index].size.z << ")");

    return true;
}
} // namespace solr
