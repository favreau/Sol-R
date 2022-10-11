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

#define _ALLOW_KEYWORD_MACROS

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

#include "../Logging.h"
#include "rtgl.h"
#include "solr.h"
#include <engines/GPUKernel.h>

#include <math.h>

namespace solr
{
SceneInfo gSceneInfo;
PostProcessingInfo glPostProcessingInfo;
const int gTotalPathTracingIterations = 1;
vec4f gRotationCenter = make_vec4f();
float gScale = 1.0f;
// Current Material
int gCurrentTexture(-1);
bool gLighting(false);
// Camera. THIS IS UGLY
vec3f gEye = make_vec3f(gScale / 10.f, 0.f, -20.f * gScale);
vec3f gDir = make_vec3f(gScale / 10.f, 0.f, -20.f * gScale + 5000.f);
vec4f gAngles = make_vec4f();

// OpenCL
void setOpenCLPlatform(const int platform)
{
#ifdef USE_OPENCL
    gOpenCLPlatform = platform;
#endif // USE_OPENCL
}

void setOpenCLDevice(const int device)
{
#ifdef USE_OPENCL
    gOpenCLDevice = device;
#endif // USE_OPENCL
}

void Initialize(const int width, const int height, const char *openCLKernel = 0)
{
#ifdef USE_OPENCL
    LOG_INFO(1, "Intializing Raytracing engine: " << gOpenCLPlatform << ","
                                                  << gOpenCLDevice);
#else
    LOG_INFO(1, "Intializing Raytracing engine");
#endif // USE_OPENCL
    // Scene
    gSceneInfo.size.x = width;
    gSceneInfo.size.y = height;
    gSceneInfo.graphicsLevel = glFull;
    gSceneInfo.nbRayIterations = 5;
    gSceneInfo.transparentColor = 0.f;
    gSceneInfo.viewDistance = 100000.f;
    gSceneInfo.shadowIntensity = 0.9f;
    gSceneInfo.eyeSeparation = 1.3f;
    gSceneInfo.backgroundColor.x = 1.f;
    gSceneInfo.backgroundColor.y = 1.f;
    gSceneInfo.backgroundColor.z = 1.f;
    gSceneInfo.backgroundColor.w = 0.f;
    gSceneInfo.renderBoxes = 0;
    gSceneInfo.pathTracingIteration = 0;
    gSceneInfo.maxPathTracingIterations = gTotalPathTracingIterations;
    gSceneInfo.frameBufferType = ftRGB;
    gSceneInfo.timestamp = 0;
    gSceneInfo.atmosphericEffect = aeNone;
    gSceneInfo.cameraType = ctPerspective;

    glPostProcessingInfo.type = ppe_none;
    glPostProcessingInfo.param1 = 10000.f;
    glPostProcessingInfo.param2 = 1000.f;
    glPostProcessingInfo.param3 = 200;
    gSceneInfo.pathTracingIteration = 0;

    // Kernel
    SingletonKernel::kernel()->setSceneInfo(gSceneInfo);
    SingletonKernel::kernel()->initBuffers();
    createRandomMaterials(false, false);
}

void glCompactBoxes()
{
    SingletonKernel::kernel()->compactBoxes(true);
}

/*
________________________________________________________________________________

Create Random Materials
________________________________________________________________________________
*/
void createRandomMaterials(bool update, bool lightsOnly)
{
    srand(static_cast<int>(time(0)));
    int start(0);
    int end(NB_MAX_MATERIALS);
    if (lightsOnly)
    {
        start = 120;
        end = 130;
    }
    // Materials
    for (int i(start); i < end; ++i)
    {
        vec4f specular = make_vec4f();
        specular.x = 1.f;
        specular.y = 500.f;
        specular.z = 0.f;
        specular.w = 0.f;

        float reflection = 0.f;
        float refraction = 0.f;
        float transparency = 0.f;
        int textureId = TEXTURE_NONE;
        vec4f innerIllumination =
            make_vec4f(0.f, 40000.f, gSceneInfo.viewDistance);
        bool procedural = false;
        bool wireframe = false;
        int wireframeDepth = 0;
        float r, g, b, gloss;
        float opacity = gSceneInfo.viewDistance;
        bool fastTransparency = false;

        r = 0.5f + (rand() % 255) / 512.f;
        g = 0.5f + (rand() % 255) / 512.f;
        b = 0.5f + (rand() % 255) / 512.f;
        gloss = 0.f;

        // if( i>0 && i<100 )
        {
            reflection = float(rand() % 11) / 10.f;
            refraction = 1.f + float(rand() % 11) / 10.f;
            transparency = float(rand() % 11) / 10.f;
            // reflection=1.f; refraction=1.66f; transparency=0.8f;
        }

        switch (i)
        {
        case DEFAULT_LIGHT_MATERIAL:
            r = 1.f;
            g = 1.f;
            b = 1.f;
            innerIllumination.x = 1.f;
            break;
        }

        int material = update ? i : SingletonKernel::kernel()->addMaterial();
        SingletonKernel::kernel()->setMaterial(
            material, r, g, b, gloss, reflection, refraction, procedural,
            wireframe, wireframeDepth, transparency, opacity, textureId,
            TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
            TEXTURE_NONE, TEXTURE_NONE, specular.x, specular.y, specular.w,
            innerIllumination.x, innerIllumination.y, innerIllumination.z,
            fastTransparency);
    }
}

void glBegin(GLint mode)
{
    SingletonKernel::kernel()->setGLMode(mode);
}

int glEnd()
{
    return SingletonKernel::kernel()->setGLMode(-1);
}

void glEnable(GLenum cap)
{
    switch (cap)
    {
    case GL_LIGHTING:
    {
        if (!gLighting)
        {
            int p = SingletonKernel::kernel()->addPrimitive(ptSphere);
            SingletonKernel::kernel()->setPrimitive(
                p, 20.f * gScale, 20.f * gScale, -20.f * gScale, 0.1f * gScale,
                0.1f * gScale, 0.1f * gScale, DEFAULT_LIGHT_MATERIAL);
            gLighting = true;
            LOG_INFO(3, "[OpenGL] Light Added");
        }
    }
    break;
    }
    //::glEnable(cap);
}

void glDisable(GLenum cap)
{
    ::glDisable(cap);
}

void glClear(GLbitfield mask)
{
    ::glClear(mask);
}

void glVertex3f(GLfloat x, GLfloat y, GLfloat z)
{
    //::glVertex3f(x,y,z);
    SingletonKernel::kernel()->addVertex(x * gScale, y * gScale, z * gScale);
}

void glVertex3fv(const GLfloat *v)
{
    //::glVertex3f(x,y,z);
    SingletonKernel::kernel()->addVertex(v[0] * gScale, v[1] * gScale,
                                         v[2] * gScale);
}

void glNormal3f(GLfloat x, GLfloat y, GLfloat z)
{
    //::glNormal3f(x,y,z);
    SingletonKernel::kernel()->addNormal(x, y, z);
}

void glNormal3fv(const GLfloat *n)
{
    SingletonKernel::kernel()->addNormal(n[0], n[1], n[2]);
}

void glColor3f(GLfloat red, GLfloat green, GLfloat blue)
{
    glColor4f(red, green, blue, 0.f);
}

void glColor4f(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha)
{
    /*
  bool found(false);
  unsigned int i(0);
  while( i<::SingletonKernel::kernel()->getNbActiveMaterials() && !found )
  {
     Material* mat=::SingletonKernel::kernel()->getMaterial(i);
     if (mat->color.x==red && mat->color.x==green && mat->color.x==blue &&
  mat->transparency.x==alpha)
     {
        found=true;
     }
     else
     {
        ++i;
     }
  }
  if( found )
  {
     ::SingletonKernel::kernel()->setMaterialColor(i,red,green,blue);
  }
  else
  */
    {
        int m = SingletonKernel::kernel()->getCurrentMaterial();
        ++m;
        SingletonKernel::kernel()->setMaterial(
            m, red, green, blue, 0.f, 0.f, 1.2f, false, 0.f, 0, alpha,
            gSceneInfo.viewDistance, MATERIAL_NONE, MATERIAL_NONE,
            MATERIAL_NONE, MATERIAL_NONE, TEXTURE_NONE, TEXTURE_NONE,
            TEXTURE_NONE, 1.f, 200.f, 1000.f, 0.f, 0.f, 0.f, false);
        SingletonKernel::kernel()->setCurrentMaterial(m);
    }
}

void glRasterPos2f(GLfloat x, GLfloat y)
{
    ::glRasterPos2f(x, y);
}

void glRasterPos3f(GLfloat x, GLfloat y, GLfloat z)
{
    ::glRasterPos3f(x, y, z);
}

void glTexParameterf(GLenum target, GLenum pname, GLfloat param)
{
    ::glTexParameterf(target, pname, param);
}

void glTexCoord2f(GLfloat s, GLfloat t)
{
    //::glTexCoord2f(s,t);
    SingletonKernel::kernel()->addTextureCoordinates(s, t);
}

void glTexCoord3f(GLfloat x, GLfloat y, GLfloat)
{
    //::glTexCoord3f(x,y,z);
    SingletonKernel::kernel()->addTextureCoordinates(x, y);
}

void glTexEnvf(GLenum target, GLenum pname, GLfloat param)
{
    ::glTexEnvf(target, pname, param);
}

void glTexImage2D(GLenum target, GLint level, GLint internalformat,
                  GLsizei width, GLsizei height, GLint border, GLenum format,
                  GLenum type, const GLvoid *pixels)
{
    ::glTexImage2D(target, level, internalformat, width, height, border, format,
                   type, pixels);
}

/*
 * GLUT
 */

void glutInit(int *pargc, char **argv)
{
    ::glutInit(pargc, argv);
}

void glutInitWindowPosition(int x, int y)
{
    ::glutInitWindowPosition(x, y);
}

void glutReshapeWindow(int width, int height)
{
    ::glutReshapeWindow(width, height);
}

void glViewport(int a, int b, int width, int height)
{
    // Initialize
    Initialize(width, height);
    ::glViewport(a, b, width, height);
}

void glutInitWindowSize(int width, int height, const char *kernel)
{
    // Initialize
    Initialize(width, height, kernel);

    ::glutInitWindowSize(width, height);
}

void glutInitDisplayMode(unsigned int displayMode)
{
    ::glutInitDisplayMode(displayMode);
}

void glutMainLoop(void)
{
    ::glutMainLoop();
}

int glutCreateWindow(const char *title)
{
    return ::glutCreateWindow(title);
}

void glutDestroyWindow(int window)
{
    ::glutDestroyWindow(window);
}

void glutFullScreen(void)
{
    ::glutFullScreen();
}

void glLoadIdentity()
{
    render();
    ::glLoadIdentity();
}

int glutGet(GLenum query)
{
    return ::glutGet(query);
}

int glutDeviceGet(GLenum query)
{
    return ::glutDeviceGet(query);
}

int glutGetModifiers(void)
{
    return ::glutGetModifiers();
}

int glutLayerGet(GLenum query)
{
    return ::glutLayerGet(query);
}

void glutKeyboardFunc(void (*callback)(unsigned char, int, int))
{
    ::glutKeyboardFunc(callback);
}

void glutDisplayFunc(void (*callback)(void))
{
    ::glutDisplayFunc(callback);
}

void glutMouseFunc(void (*callback)(int, int, int, int))
{
    ::glutMouseFunc(callback);
}

void glutMotionFunc(void (*callback)(int, int))
{
    ::glutMotionFunc(callback);
}

void glutTimerFunc(unsigned int time, void (*callback)(int), int value)
{
    ::glutTimerFunc(time, callback, value);
}

int glutCreateMenu(void (*callback)(int menu))
{
    return ::glutCreateMenu(callback);
}

void glutDestroyMenu(int menu)
{
    ::glutDestroyMenu(menu);
}

void glutAddMenuEntry(const char *label, int value)
{
    ::glutAddMenuEntry(label, value);
}

void glutAttachMenu(int button)
{
    ::glutAttachMenu(button);
}

void glutBitmapString(void *font, const unsigned char *string)
{
#ifndef __APPLE__
//::glutBitmapString(font, string);
#endif
}

void glutPostRedisplay(void)
{
    ::glutPostRedisplay();
}

void glutSwapBuffers(void)
{
    ::glutSwapBuffers();
}

void gluSphere(void *, GLfloat radius, GLint, GLint)
{
    int p = SingletonKernel::kernel()->addPrimitive(ptSphere);
    vec3f translation = SingletonKernel::kernel()->getTranslation();
    int m = SingletonKernel::kernel()->getCurrentMaterial();
    SingletonKernel::kernel()->setPrimitive(p, translation.x * gScale,
                                            translation.y * gScale,
                                            translation.z * gScale,
                                            radius * gScale, 0.f, 0.f, m);
}

void glutWireSphere(GLdouble radius, GLint, GLint)
{
    int p = SingletonKernel::kernel()->addPrimitive(ptSphere);
    vec3f translation = SingletonKernel::kernel()->getTranslation();
    int m = SingletonKernel::kernel()->getCurrentMaterial();
    SingletonKernel::kernel()->setPrimitive(p, translation.x * gScale,
                                            translation.y * gScale,
                                            translation.z * gScale,
                                            static_cast<float>(radius) * gScale,
                                            0.f, 0.f, m);
}

GLUquadricObj *gluNewQuadric(void)
{
    return 0;
}

void glClearColor(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha)
{
    gSceneInfo.backgroundColor.x = red;
    gSceneInfo.backgroundColor.y = green;
    gSceneInfo.backgroundColor.z = blue;
    gSceneInfo.backgroundColor.w = alpha;
}

void glMaterialfv(GLenum face, GLenum pname, const GLfloat *params)
{
    switch (pname)
    {
    case GL_AMBIENT:
        glColor3f(params[0], params[1], params[2]);
        break;
    }
}

void glGenTextures(GLsizei n, GLuint *textures)
{
    ++gCurrentTexture;
    SingletonKernel::kernel()->setTexturesTransfered(false);
    *textures = gCurrentTexture;
}

void glBindTexture(GLenum target, GLuint texture)
{
    switch (target)
    {
    case GL_TEXTURE_2D:
        SingletonKernel::kernel()->setMaterialTextureId(gCurrentTexture);
        break;
    }
}

int gluBuild2DMipmaps(GLenum target, GLint components, GLint width,
                      GLint height, GLenum format, GLenum type,
                      const void *data)
{
    TextureInfo textureInfo;
    textureInfo.size.x = width;
    textureInfo.size.y = height;

    switch (format)
    {
    case GL_RGB:
        textureInfo.size.z = 3;
        break;
    case GL_RGBA:
        textureInfo.size.z = 4;
        break;
    }
    textureInfo.offset = 0;
    textureInfo.buffer = (unsigned char *)data;

    SingletonKernel::kernel()->setTexture(gCurrentTexture, textureInfo);
    return 0;
}

void setAngles(GLfloat x, GLfloat y, GLfloat z)
{
    gAngles.x = x;
    gAngles.y = y;
    gAngles.z = z;
}

void glFlush()
{
    ::glFlush();
}

void glTexSubImage2D(GLenum target, GLint level, GLint xoffset, GLint yoffset,
                     GLsizei width, GLsizei height, GLenum format, GLenum type,
                     const GLvoid *data);

void glPushAttrib(GLbitfield mask) {}

void glPopAttrib() {}

void glTexParameteri(GLenum target, GLenum pname, GLint param) {}

void glBlendFunc(GLenum sfactor, GLenum dfactor) {}

void glMatrixMode(GLenum mode) {}

void glPushMatrix() {}

void glPopMatrix() {}

GLenum glGetError()
{
    return ::glGetError();
}

void glVertex2i(GLint x, GLint y)
{
    glVertex3f(static_cast<GLfloat>(x), static_cast<GLfloat>(y), 0.f);
}

void glOrtho(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top,
             GLdouble nearVal, GLdouble farVal)
{
    SingletonKernel::kernel()->getSceneInfo().cameraType = ctOrthographic;
}

void render()
{
    if (SingletonKernel::kernel())
    {
        const vec4f rotation = SingletonKernel::kernel()->getRotation();
        SingletonKernel::kernel()->setCamera(gEye, gDir, rotation);
        SingletonKernel::kernel()->setSceneInfo(gSceneInfo);
        SingletonKernel::kernel()->setPostProcessingInfo(glPostProcessingInfo);
        if (!gLighting)
        {
            // if no light is defined, I add one
            int p = SingletonKernel::kernel()->addPrimitive(ptSphere);
            SingletonKernel::kernel()->setPrimitive(
                p, 20.f * gScale, 20.f * gScale, 20.f * gScale, 0.1f * gScale,
                0.1f * gScale, 0.1f * gScale, DEFAULT_LIGHT_MATERIAL);

            // p = ::SingletonKernel::kernel()->addPrimitive(ptSphere);
            // ::SingletonKernel::kernel()->setPrimitive(p,0.f,0.f,0.f,0.1f*gScale,0.1f*gScale,0.1f*gScale,0);
            gLighting = true;
        }

        SingletonKernel::kernel()->compactBoxes(false);
        for (int i(0); i < gTotalPathTracingIterations; ++i)
        {
            SingletonKernel::kernel()->render_begin(0);
            SingletonKernel::kernel()->render_end();
        }
        SingletonKernel::kernel()->resetFrame();
        gLighting = false;
    }
}

void glTranslatef(GLfloat x, GLfloat y, GLfloat z)
{
    SingletonKernel::kernel()->translate(x * gScale, y * gScale,
                                         -z * gScale); // Z is inverted!!
}

void glRotatef(GLfloat angle, GLfloat x, GLfloat y, GLfloat z)
{
    SingletonKernel::kernel()->rotate(angle * x * PI / 180.f,
                                      angle * y * PI / 180.f,
                                      angle * z * PI / 180.f);
}

void gluLookAt(GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ, GLdouble centerX,
               GLdouble centerY, GLdouble centerZ, GLdouble upX, GLdouble upY,
               GLdouble upZ)
{
    gEye.x = static_cast<float>(eyeX * gScale);
    gEye.y = static_cast<float>(eyeY * gScale);
    gEye.z = static_cast<float>(eyeZ * gScale - 5000.f);
    gDir.x = static_cast<float>(centerX * gScale);
    gDir.y = static_cast<float>(centerY * gScale);
    gDir.z = static_cast<float>(centerZ * gScale);
}

void glTexSubImage2D(GLenum target, GLint level, GLint xoffset, GLint yoffset,
                     GLsizei width, GLsizei height, GLenum format, GLenum type,
                     const GLvoid *data)
{
    ::glTexSubImage2D(target, level, xoffset, yoffset, width, height, format,
                      type, data);
}

void *gluNewNurbsRenderer()
{
    return 0;
}

void glutSpecialFunc(void (*func)(int key, int x, int y))
{
    ::glutSpecialFunc(func);
}

void glutReshapeFunc(void (*func)(int width, int height))
{
    ::glutReshapeFunc(func);
}

void glutIdleFunc(void (*func)(void))
{
    ::glutIdleFunc(func);
}

void gluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear,
                    GLdouble zFar)
{
    gEye.z = -20.f * static_cast<float>(aspect) /
             tanf(static_cast<float>(fovy)) * gScale;
    gDir.z = static_cast<float>(gEye.z + zNear * gScale + 5000.f);
    gSceneInfo.viewDistance =
        static_cast<float>(gDir.z + zFar * gScale + 5000.f);
}

void glutSetCursor(int cursor)
{
    ::glutSetCursor(cursor);
}

void glPointSize(GLfloat size)
{
    SingletonKernel::kernel()->setPointSize(size);
}

void glReadPixels(GLint x, GLint y, GLsizei width, GLsizei height,
                  GLenum format, GLenum type, GLvoid *data)
{
    ::glReadPixels(x, y, width, height, format, type, data);
}
} // namespace solr
