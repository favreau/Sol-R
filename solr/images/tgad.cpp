#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <stdlib.h>
#include <string.h>

#include "../Logging.h"

#include "tgad.h"

/********************************************************************************
/name :		LoadTGA(Texture * texture, char * filename) * /function:  Open and
test the file to make sure it is a valid TGA file			* /parems:	texture,
pointer to a Texture structure								*
/			filename, string pointing to file to open *
/********************************************************************************/

unsigned char uTGAcompare[12] = {0, 0, 2, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0}; // Uncompressed TGA Header
unsigned char cTGAcompare[12] = {0, 0, 10, 0, 0, 0,
                                 0, 0, 0,  0, 0, 0}; // Compressed TGA Header
TGAHeader tgaheader;                                 // TGA header
TGA tga;                                             // TGA image data

bool LoadTGA(Texture *texture, char *filename) // Load a TGA file
{
    FILE *fTGA;                   // File pointer to texture file
    fTGA = fopen(filename, "rb"); // Open file for reading

    if (fTGA == NULL) // If it didn't open....
    {
        LOG_ERROR("Could not open texture file"
                  << filename); // Display an error message
        return false;           // Exit function
    }

    TGAHeader tgaheader;
    if (fread(&tgaheader, sizeof(TGAHeader), 1, fTGA) ==
        0) // Attempt to read 12 byte header from file
    {
        LOG_ERROR("Could not read file header"); // If it fails, display an
                                                 // error message
        if (fTGA != NULL) // Check to seeiffile is still open
        {
            fclose(fTGA); // If it is, close it
        }
        return false; // Exit function
    }

    if (memcmp(uTGAcompare, &tgaheader, sizeof(tgaheader)) ==
        0) // See if header matches the predefined header of
    {      // an Uncompressed TGA image
        LoadUncompressedTGA(texture, filename,
                            fTGA); // If so, jump to Uncompressed TGA loading
                                   // code
    }
    else if (memcmp(cTGAcompare, &tgaheader, sizeof(tgaheader)) ==
             0) // See if header matches the predefined header of
    {           // an RLE compressed TGA image
        LoadCompressedTGA(texture, filename,
                          fTGA); // If so, jump to Compressed TGA loading code
    }
    else // If header matches neither type
    {
        LOG_ERROR("TGA file be type 2 or type 10 "); // Display an error
        fclose(fTGA);
        return false; // Exit function
    }
    return true; // All went well, continue on
}

bool LoadUncompressedTGA(Texture *texture, char *filename,
                         FILE *fTGA) // Load an uncompressed TGA (note, much of
                                     // this code is based on NeHe's
{
    // TGA Loading code nehe.gamedev.net)
    if (fread(tga.header, sizeof(tga.header), 1, fTGA) == 0) // Read TGA header
    {
        LOG_ERROR("Could not read info header"); // Display error
        if (fTGA != NULL)                        // if file is still open
        {
            fclose(fTGA); // Close it
        }
        return false; // Return failular
    }

    texture->width =
        tga.header[1] * 256 +
        tga.header[0]; // Determine The TGA Width	(highbyte*256+lowbyte)
    texture->height =
        tga.header[3] * 256 +
        tga.header[2]; // Determine The TGA Height	(highbyte*256+lowbyte)
    texture->bpp = tga.header[4]; // Determine the bits per pixel
    tga.Width = texture->width;   // Copy width into local structure
    tga.Height = texture->height; // Copy height into local structure
    tga.Bpp = texture->bpp;       // Copy BPP into local structure

    if ((texture->width <= 0) || (texture->height <= 0) ||
        ((texture->bpp != 24) && (texture->bpp != 32))) // Make sure all
                                                        // information is valid
    {
        LOG_ERROR("Invalid texture information"); // Display Error
        if (fTGA != NULL)                         // Check if file is still open
        {
            fclose(fTGA); // If so, close it
        }
        return false; // Return failed
    }

    /*
    if(texture->bpp == 24)													//
    If the BPP of the image is 24... texture->type	= GL_RGB;
    // Set Image type to GL_RGB else
    // Else if its 32 BPP
       texture->type	= GL_RGBA;											// Set image type to
    GL_RGBA
    */

    tga.bytesPerPixel = (tga.Bpp / 8); // Compute the number of BYTES per pixel
    tga.imageSize =
        (tga.bytesPerPixel * tga.Width *
         tga.Height); // Compute the total amout ofmemory needed to store data
    texture->imageData =
        (unsigned char *)malloc(tga.imageSize); // Allocate that much memory

    if (texture->imageData == NULL) // If no space was allocated
    {
        LOG_ERROR("Could not allocate memory for image"); // Display Error
        fclose(fTGA);                                     // Close the file
        return false;                                     // Return failed
    }

    if (fread(texture->imageData, 1, tga.imageSize, fTGA) !=
        tga.imageSize) // Attempt to read image data
    {
        LOG_ERROR("Could not read image data"); // Display Error
        if (texture->imageData != NULL)         // If imagedata has data in it
        {
            free(texture->imageData); // Delete data from memory
        }
        fclose(fTGA); // Close file
        return false; // Return failed
    }

    // Byte Swapping Optimized By Steve Thomas
    for (unsigned int cswap = 0; cswap < (int)tga.imageSize;
         cswap += tga.bytesPerPixel)
    {
        texture->imageData[cswap] ^= texture->imageData[cswap + 2] ^=
            texture->imageData[cswap] ^= texture->imageData[cswap + 2];
    }

    fclose(fTGA); // Close file
    return true;  // Return success
}

bool LoadCompressedTGA(Texture *texture, char *filename,
                       FILE *fTGA) // Load COMPRESSED TGAs
{
    if (fread(tga.header, sizeof(tga.header), 1, fTGA) == 0) // Attempt to read
                                                             // header
    {
        LOG_ERROR("Could not read info header"); // Display Error
        if (fTGA != NULL)                        // If file is open
        {
            fclose(fTGA); // Close it
        }
        return false; // Return failed
    }

    texture->width =
        tga.header[1] * 256 +
        tga.header[0]; // Determine The TGA Width	(highbyte*256+lowbyte)
    texture->height =
        tga.header[3] * 256 +
        tga.header[2]; // Determine The TGA Height	(highbyte*256+lowbyte)
    texture->bpp = tga.header[4]; // Determine Bits Per Pixel
    tga.Width = texture->width;   // Copy width to local structure
    tga.Height = texture->height; // Copy width to local structure
    tga.Bpp = texture->bpp;       // Copy width to local structure

    if ((texture->width <= 0) || (texture->height <= 0) ||
        ((texture->bpp != 24) && (texture->bpp != 32))) // Make sure all texture
                                                        // info is ok
    {
        LOG_ERROR("Invalid texture information"); // If it isnt...Display error
        if (fTGA != NULL)                         // Check if file is open
        {
            fclose(fTGA); // Ifit is, close it
        }
        return false; // Return failed
    }

    /*
    if(texture->bpp == 24)													//
    If the BPP of the image is 24... texture->type	= GL_RGB;
    // Set Image type to GL_RGB else
    // Else if its 32 BPP
       texture->type	= GL_RGBA;											// Set image type to
    GL_RGBA
    */

    tga.bytesPerPixel = (tga.Bpp / 8); // Compute BYTES per pixel
    tga.imageSize =
        (tga.bytesPerPixel * tga.Width * tga.Height); // Compute amout of memory
                                                      // needed to store image
    texture->imageData =
        (unsigned char *)malloc(tga.imageSize); // Allocate that much memory

    if (texture->imageData == NULL) // If it wasnt allocated correctly..
    {
        LOG_ERROR("Could not allocate memory for image"); // Display Error
        fclose(fTGA);                                     // Close file
        return false;                                     // Return failed
    }

    unsigned int pixelcount =
        tga.Height * tga.Width;    // Nuber of pixels in the image
    unsigned int currentpixel = 0; // Current pixel being read
    unsigned int currentbyte = 0;  // Current byte
    unsigned char *colorbuffer =
        (unsigned char *)malloc(tga.bytesPerPixel); // Storage for 1 pixel

    do
    {
        unsigned char chunkheader = 0; // Storage for "chunk" header

        if (fread(&chunkheader, sizeof(unsigned char), 1, fTGA) ==
            0) // Read in the 1 byte header
        {
            LOG_ERROR("Could not read RLE header"); // Display Error
            if (fTGA != NULL)                       // If file is open
            {
                fclose(fTGA); // Close file
            }
            if (texture->imageData != NULL) // If there is stored image data
            {
                free(texture->imageData); // Delete image data
            }
            return false; // Return failed
        }

        if (chunkheader < 128) // If the ehader is < 128, it means the that is
                               // the number of RAW color packets minus 1
        {                      // that follow the header
            chunkheader++;     // add 1 to get number of following color values
            for (short counter = 0; counter < chunkheader;
                 counter++) // Read RAW color values
            {
                if (fread(colorbuffer, 1, tga.bytesPerPixel, fTGA) !=
                    tga.bytesPerPixel) // Try to read 1 pixel
                {
                    LOG_ERROR("Could not read image data"); // IF we cant,
                                                            // display an error

                    if (fTGA != NULL) // See if file is open
                    {
                        fclose(fTGA); // If so, close file
                    }

                    if (colorbuffer != NULL) // See if colorbuffer has data in
                                             // it
                    {
                        free(colorbuffer); // If so, delete it
                    }

                    if (texture->imageData != NULL) // See if there is stored
                                                    // Image data
                    {
                        free(texture->imageData); // If so, delete it too
                    }

                    return false; // Return failed
                }
                // write to memory
                texture->imageData[currentbyte] =
                    colorbuffer[2]; // Flip R and B vcolor values around in the
                                    // process
                texture->imageData[currentbyte + 1] = colorbuffer[1];
                texture->imageData[currentbyte + 2] = colorbuffer[0];

                if (tga.bytesPerPixel == 4) // if its a 32 bpp image
                {
                    texture->imageData[currentbyte + 3] =
                        colorbuffer[3]; // copy the 4th byte
                }

                currentbyte +=
                    tga.bytesPerPixel; // Increase thecurrent byte by the number
                                       // of bytes per pixel
                currentpixel++;        // Increase current pixel by 1

                if (currentpixel > pixelcount) // Make sure we havent read too
                                               // many pixels
                {
                    LOG_ERROR("Too many pixels read"); // if there is too
                                                       // many... Display an
                                                       // error!

                    if (fTGA != NULL) // If there is a file open
                    {
                        fclose(fTGA); // Close file
                    }

                    if (colorbuffer != NULL) // If there is data in colorbuffer
                    {
                        free(colorbuffer); // Delete it
                    }

                    if (texture->imageData != NULL) // If there is Image data
                    {
                        free(texture->imageData); // delete it
                    }

                    return false; // Return failed
                }
            }
        }
        else // chunkheader > 128 RLE data, next color reapeated chunkheader -
             // 127 times
        {
            chunkheader -= 127; // Subteact 127 to get rid of the ID bit
            if (fread(colorbuffer, 1, tga.bytesPerPixel, fTGA) !=
                tga.bytesPerPixel) // Attempt to read following color
                                   // values
            {
                LOG_ERROR("Could not read from file"); // If attempt fails..
                                                       // Display error (again)

                if (fTGA != NULL) // If thereis a file open
                {
                    fclose(fTGA); // Close it
                }

                if (colorbuffer != NULL) // If there is data in the colorbuffer
                {
                    free(colorbuffer); // delete it
                }

                if (texture->imageData != NULL) // If thereis image data
                {
                    free(texture->imageData); // delete it
                }

                return false; // return failed
            }

            for (short counter = 0; counter < chunkheader;
                 counter++) // copy the color into the image data as many times
                            // as dictated
            {               // by the header
                texture->imageData[currentbyte] =
                    colorbuffer[2]; // switch R and B bytes areound while
                                    // copying
                texture->imageData[currentbyte + 1] = colorbuffer[1];
                texture->imageData[currentbyte + 2] = colorbuffer[0];

                if (tga.bytesPerPixel == 4) // If TGA images is 32 bpp
                {
                    texture->imageData[currentbyte + 3] =
                        colorbuffer[3]; // Copy 4th byte
                }

                currentbyte +=
                    tga.bytesPerPixel; // Increase current byte by the number of
                                       // bytes per pixel
                currentpixel++;        // Increase pixel count by 1

                if (currentpixel > pixelcount) // Make sure we havent written
                                               // too many pixels
                {
                    LOG_ERROR("Too many pixels read"); // if there is too
                                                       // many... Display an
                                                       // error!

                    if (fTGA != NULL) // If there is a file open
                    {
                        fclose(fTGA); // Close file
                    }

                    if (colorbuffer != NULL) // If there is data in colorbuffer
                    {
                        free(colorbuffer); // Delete it
                    }

                    if (texture->imageData != NULL) // If there is Image data
                    {
                        free(texture->imageData); // delete it
                    }

                    return false; // Return failed
                }
            }
        }
    }

    while (currentpixel < pixelcount); // Loop while there are still pixels left
    fclose(fTGA);                      // Close the file
    return true;                       // return success
}
