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

#include "../api.h"

typedef unsigned int GLenum;
typedef unsigned char GLboolean;
typedef unsigned int GLbitfield;
typedef signed char GLbyte;
typedef short GLshort;
typedef int GLint;
typedef int GLsizei;
typedef unsigned char GLubyte;
typedef unsigned short GLushort;
typedef unsigned int GLuint;
typedef float GLfloat;
typedef float GLclampf;
typedef double GLdouble;
typedef double GLclampd;
typedef void GLvoid;

#ifndef __cplusplus
typedef int bool;
#endif // #__cplusplus

namespace solr
{
// OpenGL
/*************************************************************/

/* LightName */
#define GL_LIGHT0 0x4000
#define GL_LIGHT1 0x4001
#define GL_LIGHT2 0x4002
#define GL_LIGHT3 0x4003
#define GL_LIGHT4 0x4004
#define GL_LIGHT5 0x4005
#define GL_LIGHT6 0x4006
#define GL_LIGHT7 0x4007

/* ErrorCode */
//#define GL_NO_ERROR                       0
#define GL_INVALID_ENUM 0x0500
#define GL_INVALID_VALUE 0x0501
#define GL_INVALID_OPERATION 0x0502
#define GL_STACK_OVERFLOW 0x0503
#define GL_STACK_UNDERFLOW 0x0504
#define GL_OUT_OF_MEMORY 0x0505

/* Boolean */
//#define GL_TRUE                           1
//#define GL_FALSE                          0

/* MatrixMode */
#define GL_MODELVIEW 0x1700
#define GL_PROJECTION 0x1701
#define GL_TEXTURE 0x1702

/* StringName */
#define GL_VENDOR 0x1F00
#define GL_RENDERER 0x1F01
#define GL_VERSION 0x1F02
#define GL_EXTENSIONS 0x1F03

/* BlendingFactorDest */
//#define GL_ZERO                           0
//#define GL_ONE                            1
#define GL_SRC_COLOR 0x0300
#define GL_ONE_MINUS_SRC_COLOR 0x0301
#define GL_SRC_ALPHA 0x0302
#define GL_ONE_MINUS_SRC_ALPHA 0x0303
#define GL_DST_ALPHA 0x0304
#define GL_ONE_MINUS_DST_ALPHA 0x0305

/* BeginMode */
#define GL_POINTS 0x0000
#define GL_LINES 0x0001
#define GL_LINE_LOOP 0x0002
#define GL_LINE_STRIP 0x0003
#define GL_TRIANGLES 0x0004
#define GL_TRIANGLE_STRIP 0x0005
#define GL_TRIANGLE_FAN 0x0006
#define GL_QUADS 0x0007
#define GL_QUAD_STRIP 0x0008
#define GL_POLYGON 0x0009

/* DataType */
#define GL_BYTE 0x1400
#define GL_UNSIGNED_BYTE 0x1401
#define GL_SHORT 0x1402
#define GL_UNSIGNED_SHORT 0x1403
#define GL_INT 0x1404
#define GL_UNSIGNED_INT 0x1405
#define GL_FLOAT 0x1406
#define GL_2_BYTES 0x1407
#define GL_3_BYTES 0x1408
#define GL_4_BYTES 0x1409
#define GL_DOUBLE 0x140A
//#define GL_DOUBLE_EXT                     0x140A

/* PixelFormat */
#define GL_COLOR_INDEX 0x1900
#define GL_STENCIL_INDEX 0x1901
#define GL_DEPTH_COMPONENT 0x1902
#define GL_RED 0x1903
#define GL_GREEN 0x1904
#define GL_BLUE 0x1905
#define GL_ALPHA 0x1906
#define GL_RGB 0x1907
#define GL_RGBA 0x1908
#define GL_LUMINANCE 0x1909
#define GL_LUMINANCE_ALPHA 0x190A
/*      GL_ABGR_EXT */

/* GetPName */
#define GL_CURRENT_COLOR 0x0B00
#define GL_CURRENT_INDEX 0x0B01
#define GL_CURRENT_NORMAL 0x0B02
#define GL_CURRENT_TEXTURE_COORDS 0x0B03
#define GL_CURRENT_RASTER_COLOR 0x0B04
#define GL_CURRENT_RASTER_INDEX 0x0B05
#define GL_CURRENT_RASTER_TEXTURE_COORDS 0x0B06
#define GL_CURRENT_RASTER_POSITION 0x0B07
#define GL_CURRENT_RASTER_POSITION_VALID 0x0B08
#define GL_CURRENT_RASTER_DISTANCE 0x0B09
#define GL_POINT_SMOOTH 0x0B10
#define GL_POINT_SIZE 0x0B11
#define GL_SMOOTH_POINT_SIZE_RANGE 0x0B12
#define GL_SMOOTH_POINT_SIZE_GRANULARITY 0x0B13
#define GL_LINE_SMOOTH 0x0B20
#define GL_LINE_WIDTH 0x0B21
#define GL_SMOOTH_LINE_WIDTH_RANGE 0x0B22
#define GL_SMOOTH_LINE_WIDTH_GRANULARITY 0x0B23
#define GL_LINE_STIPPLE 0x0B24
#define GL_LINE_STIPPLE_PATTERN 0x0B25
#define GL_LINE_STIPPLE_REPEAT 0x0B26
#define GL_LIST_MODE 0x0B30
#define GL_MAX_LIST_NESTING 0x0B31
#define GL_LIST_BASE 0x0B32
#define GL_LIST_INDEX 0x0B33
#define GL_POLYGON_MODE 0x0B40
#define GL_POLYGON_SMOOTH 0x0B41
#define GL_POLYGON_STIPPLE 0x0B42
#define GL_EDGE_FLAG 0x0B43
#define GL_CULL_FACE 0x0B44
#define GL_CULL_FACE_MODE 0x0B45
#define GL_FRONT_FACE 0x0B46
#define GL_LIGHTING 0x0B50
#define GL_LIGHT_MODEL_LOCAL_VIEWER 0x0B51
#define GL_LIGHT_MODEL_TWO_SIDE 0x0B52
#define GL_LIGHT_MODEL_AMBIENT 0x0B53
#define GL_SHADE_MODEL 0x0B54
#define GL_COLOR_MATERIAL_FACE 0x0B55
#define GL_COLOR_MATERIAL_PARAMETER 0x0B56
#define GL_COLOR_MATERIAL 0x0B57
#define GL_FOG 0x0B60
#define GL_FOG_INDEX 0x0B61
#define GL_FOG_DENSITY 0x0B62
#define GL_FOG_START 0x0B63
#define GL_FOG_END 0x0B64
#define GL_FOG_MODE 0x0B65
#define GL_FOG_COLOR 0x0B66
#define GL_DEPTH_RANGE 0x0B70
#define GL_DEPTH_TEST 0x0B71
#define GL_DEPTH_WRITEMASK 0x0B72
#define GL_DEPTH_CLEAR_VALUE 0x0B73
#define GL_DEPTH_FUNC 0x0B74
#define GL_ACCUM_CLEAR_VALUE 0x0B80
#define GL_STENCIL_TEST 0x0B90
#define GL_STENCIL_CLEAR_VALUE 0x0B91
#define GL_STENCIL_FUNC 0x0B92
#define GL_STENCIL_VALUE_MASK 0x0B93
#define GL_STENCIL_FAIL 0x0B94
#define GL_STENCIL_PASS_DEPTH_FAIL 0x0B95
#define GL_STENCIL_PASS_DEPTH_PASS 0x0B96
#define GL_STENCIL_REF 0x0B97
#define GL_STENCIL_WRITEMASK 0x0B98
#define GL_MATRIX_MODE 0x0BA0
#define GL_NORMALIZE 0x0BA1
#define GL_VIEWPORT 0x0BA2
#define GL_MODELVIEW_STACK_DEPTH 0x0BA3
#define GL_PROJECTION_STACK_DEPTH 0x0BA4
#define GL_TEXTURE_STACK_DEPTH 0x0BA5
#define GL_MODELVIEW_MATRIX 0x0BA6
#define GL_PROJECTION_MATRIX 0x0BA7
#define GL_TEXTURE_MATRIX 0x0BA8
#define GL_ATTRIB_STACK_DEPTH 0x0BB0
#define GL_CLIENT_ATTRIB_STACK_DEPTH 0x0BB1
#define GL_ALPHA_TEST 0x0BC0
#define GL_ALPHA_TEST_FUNC 0x0BC1
#define GL_ALPHA_TEST_REF 0x0BC2
#define GL_DITHER 0x0BD0
#define GL_BLEND_DST 0x0BE0
#define GL_BLEND_SRC 0x0BE1
#define GL_BLEND 0x0BE2
#define GL_LOGIC_OP_MODE 0x0BF0
#define GL_INDEX_LOGIC_OP 0x0BF1
//#define GL_LOGIC_OP                       GL_INDEX_LOGIC_OP
#define GL_COLOR_LOGIC_OP 0x0BF2
#define GL_AUX_BUFFERS 0x0C00
#define GL_DRAW_BUFFER 0x0C01
#define GL_READ_BUFFER 0x0C02
#define GL_SCISSOR_BOX 0x0C10
#define GL_SCISSOR_TEST 0x0C11
#define GL_INDEX_CLEAR_VALUE 0x0C20
#define GL_INDEX_WRITEMASK 0x0C21
#define GL_COLOR_CLEAR_VALUE 0x0C22
#define GL_COLOR_WRITEMASK 0x0C23
#define GL_INDEX_MODE 0x0C30
#define GL_RGBA_MODE 0x0C31
#define GL_DOUBLEBUFFER 0x0C32
#define GL_STEREO 0x0C33
#define GL_RENDER_MODE 0x0C40
#define GL_PERSPECTIVE_CORRECTION_HINT 0x0C50
#define GL_POINT_SMOOTH_HINT 0x0C51
#define GL_LINE_SMOOTH_HINT 0x0C52
#define GL_POLYGON_SMOOTH_HINT 0x0C53
#define GL_FOG_HINT 0x0C54
#define GL_TEXTURE_GEN_S 0x0C60
#define GL_TEXTURE_GEN_T 0x0C61
#define GL_TEXTURE_GEN_R 0x0C62
#define GL_TEXTURE_GEN_Q 0x0C63
#define GL_PIXEL_MAP_I_TO_I_SIZE 0x0CB0
#define GL_PIXEL_MAP_S_TO_S_SIZE 0x0CB1
#define GL_PIXEL_MAP_I_TO_R_SIZE 0x0CB2
#define GL_PIXEL_MAP_I_TO_G_SIZE 0x0CB3
#define GL_PIXEL_MAP_I_TO_B_SIZE 0x0CB4
#define GL_PIXEL_MAP_I_TO_A_SIZE 0x0CB5
#define GL_PIXEL_MAP_R_TO_R_SIZE 0x0CB6
#define GL_PIXEL_MAP_G_TO_G_SIZE 0x0CB7
#define GL_PIXEL_MAP_B_TO_B_SIZE 0x0CB8
#define GL_PIXEL_MAP_A_TO_A_SIZE 0x0CB9
#define GL_UNPACK_SWAP_BYTES 0x0CF0
#define GL_UNPACK_LSB_FIRST 0x0CF1
#define GL_UNPACK_ROW_LENGTH 0x0CF2
#define GL_UNPACK_SKIP_ROWS 0x0CF3
#define GL_UNPACK_SKIP_PIXELS 0x0CF4
#define GL_UNPACK_ALIGNMENT 0x0CF5
#define GL_PACK_SWAP_BYTES 0x0D00
#define GL_PACK_LSB_FIRST 0x0D01
#define GL_PACK_ROW_LENGTH 0x0D02
#define GL_PACK_SKIP_ROWS 0x0D03
#define GL_PACK_SKIP_PIXELS 0x0D04
#define GL_PACK_ALIGNMENT 0x0D05
#define GL_MAP_COLOR 0x0D10
#define GL_MAP_STENCIL 0x0D11
#define GL_INDEX_SHIFT 0x0D12
#define GL_INDEX_OFFSET 0x0D13
#define GL_RED_SCALE 0x0D14
#define GL_RED_BIAS 0x0D15
#define GL_ZOOM_X 0x0D16
#define GL_ZOOM_Y 0x0D17
#define GL_GREEN_SCALE 0x0D18
#define GL_GREEN_BIAS 0x0D19
#define GL_BLUE_SCALE 0x0D1A
#define GL_BLUE_BIAS 0x0D1B
#define GL_ALPHA_SCALE 0x0D1C
#define GL_ALPHA_BIAS 0x0D1D
#define GL_DEPTH_SCALE 0x0D1E
#define GL_DEPTH_BIAS 0x0D1F
#define GL_MAX_EVAL_ORDER 0x0D30
#define GL_MAX_LIGHTS 0x0D31
#define GL_MAX_CLIP_PLANES 0x0D32
#define GL_MAX_TEXTURE_SIZE 0x0D33
#define GL_MAX_PIXEL_MAP_TABLE 0x0D34
#define GL_MAX_ATTRIB_STACK_DEPTH 0x0D35
#define GL_MAX_MODELVIEW_STACK_DEPTH 0x0D36
#define GL_MAX_NAME_STACK_DEPTH 0x0D37
#define GL_MAX_PROJECTION_STACK_DEPTH 0x0D38
#define GL_MAX_TEXTURE_STACK_DEPTH 0x0D39
#define GL_MAX_VIEWPORT_DIMS 0x0D3A
#define GL_MAX_CLIENT_ATTRIB_STACK_DEPTH 0x0D3B
#define GL_SUBPIXEL_BITS 0x0D50
#define GL_INDEX_BITS 0x0D51
#define GL_RED_BITS 0x0D52
#define GL_GREEN_BITS 0x0D53
#define GL_BLUE_BITS 0x0D54
#define GL_ALPHA_BITS 0x0D55
#define GL_DEPTH_BITS 0x0D56
#define GL_STENCIL_BITS 0x0D57
#define GL_ACCUM_RED_BITS 0x0D58
#define GL_ACCUM_GREEN_BITS 0x0D59
#define GL_ACCUM_BLUE_BITS 0x0D5A
#define GL_ACCUM_ALPHA_BITS 0x0D5B
#define GL_NAME_STACK_DEPTH 0x0D70
#define GL_AUTO_NORMAL 0x0D80
#define GL_MAP1_COLOR_4 0x0D90
#define GL_MAP1_INDEX 0x0D91
#define GL_MAP1_NORMAL 0x0D92
#define GL_MAP1_TEXTURE_COORD_1 0x0D93
#define GL_MAP1_TEXTURE_COORD_2 0x0D94
#define GL_MAP1_TEXTURE_COORD_3 0x0D95
#define GL_MAP1_TEXTURE_COORD_4 0x0D96
#define GL_MAP1_VERTEX_3 0x0D97
#define GL_MAP1_VERTEX_4 0x0D98
#define GL_MAP2_COLOR_4 0x0DB0
#define GL_MAP2_INDEX 0x0DB1
#define GL_MAP2_NORMAL 0x0DB2
#define GL_MAP2_TEXTURE_COORD_1 0x0DB3
#define GL_MAP2_TEXTURE_COORD_2 0x0DB4
#define GL_MAP2_TEXTURE_COORD_3 0x0DB5
#define GL_MAP2_TEXTURE_COORD_4 0x0DB6
#define GL_MAP2_VERTEX_3 0x0DB7
#define GL_MAP2_VERTEX_4 0x0DB8
#define GL_MAP1_GRID_DOMAIN 0x0DD0
#define GL_MAP1_GRID_SEGMENTS 0x0DD1
#define GL_MAP2_GRID_DOMAIN 0x0DD2
#define GL_MAP2_GRID_SEGMENTS 0x0DD3
#define GL_TEXTURE_1D 0x0DE0
#define GL_TEXTURE_2D 0x0DE1
#define GL_FEEDBACK_BUFFER_POINTER 0x0DF0
#define GL_FEEDBACK_BUFFER_SIZE 0x0DF1
#define GL_FEEDBACK_BUFFER_TYPE 0x0DF2
#define GL_SELECTION_BUFFER_POINTER 0x0DF3
#define GL_SELECTION_BUFFER_SIZE 0x0DF4
#define GL_POLYGON_OFFSET_UNITS 0x2A00
#define GL_POLYGON_OFFSET_POINT 0x2A01
#define GL_POLYGON_OFFSET_LINE 0x2A02
#define GL_POLYGON_OFFSET_FILL 0x8037
#define GL_POLYGON_OFFSET_FACTOR 0x8038
#define GL_TEXTURE_BINDING_1D 0x8068
#define GL_TEXTURE_BINDING_2D 0x8069
#define GL_TEXTURE_BINDING_3D 0x806A
#define GL_VERTEX_ARRAY 0x8074
#define GL_NORMAL_ARRAY 0x8075
#define GL_COLOR_ARRAY 0x8076
#define GL_INDEX_ARRAY 0x8077
#define GL_TEXTURE_COORD_ARRAY 0x8078
#define GL_EDGE_FLAG_ARRAY 0x8079
#define GL_VERTEX_ARRAY_SIZE 0x807A
#define GL_VERTEX_ARRAY_TYPE 0x807B
#define GL_VERTEX_ARRAY_STRIDE 0x807C
#define GL_NORMAL_ARRAY_TYPE 0x807E
#define GL_NORMAL_ARRAY_STRIDE 0x807F
#define GL_COLOR_ARRAY_SIZE 0x8081
#define GL_COLOR_ARRAY_TYPE 0x8082
#define GL_COLOR_ARRAY_STRIDE 0x8083
#define GL_INDEX_ARRAY_TYPE 0x8085
#define GL_INDEX_ARRAY_STRIDE 0x8086
#define GL_TEXTURE_COORD_ARRAY_SIZE 0x8088
#define GL_TEXTURE_COORD_ARRAY_TYPE 0x8089
#define GL_TEXTURE_COORD_ARRAY_STRIDE 0x808A
#define GL_EDGE_FLAG_ARRAY_STRIDE 0x808C
/*      GL_VERTEX_ARRAY_COUNT_EXT */
/*      GL_NORMAL_ARRAY_COUNT_EXT */
/*      GL_COLOR_ARRAY_COUNT_EXT */
/*      GL_INDEX_ARRAY_COUNT_EXT */
/*      GL_TEXTURE_COORD_ARRAY_COUNT_EXT */
/*      GL_EDGE_FLAG_ARRAY_COUNT_EXT */

/* MaterialParameter */
#define GL_EMISSION 0x1600
#define GL_SHININESS 0x1601
#define GL_AMBIENT_AND_DIFFUSE 0x1602
#define GL_COLOR_INDEXES 0x1603

/* LightParameter */
#define GL_AMBIENT 0x1200
#define GL_DIFFUSE 0x1201
#define GL_SPECULAR 0x1202
#define GL_POSITION 0x1203
#define GL_SPOT_DIRECTION 0x1204
#define GL_SPOT_EXPONENT 0x1205
#define GL_SPOT_CUTOFF 0x1206
#define GL_CONSTANT_ATTENUATION 0x1207
#define GL_LINEAR_ATTENUATION 0x1208
#define GL_QUADRATIC_ATTENUATION 0x1209

/* TextureCoordName */
#define GL_S 0x2000
#define GL_T 0x2001
#define GL_R 0x2002
#define GL_Q 0x2003

/* TextureEnvMode */
#define GL_MODULATE 0x2100
#define GL_DECAL 0x2101
/*      GL_BLEND */
/*      GL_REPLACE */
/*      GL_ADD */

/* TextureEnvParameter */
#define GL_TEXTURE_ENV_MODE 0x2200
#define GL_TEXTURE_ENV_COLOR 0x2201

/* TextureEnvTarget */
#define GL_TEXTURE_ENV 0x2300

/* TextureGenMode */
#define GL_EYE_LINEAR 0x2400
#define GL_OBJECT_LINEAR 0x2401
#define GL_SPHERE_MAP 0x2402

/* TextureGenParameter */
#define GL_TEXTURE_GEN_MODE 0x2500
#define GL_OBJECT_PLANE 0x2501
#define GL_EYE_PLANE 0x2502

/* TextureMagFilter */
#define GL_NEAREST 0x2600
#define GL_LINEAR 0x2601

/* TextureMinFilter */
/*      GL_NEAREST */
/*      GL_LINEAR */
#define GL_NEAREST_MIPMAP_NEAREST 0x2700
#define GL_LINEAR_MIPMAP_NEAREST 0x2701
#define GL_NEAREST_MIPMAP_LINEAR 0x2702
#define GL_LINEAR_MIPMAP_LINEAR 0x2703

/* TextureParameterName */
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_TEXTURE_WRAP_S 0x2802
#define GL_TEXTURE_WRAP_T 0x2803
/*      GL_TEXTURE_BORDER_COLOR */
/*      GL_TEXTURE_PRIORITY */

/* TextureTarget */
/*      GL_TEXTURE_1D */
/*      GL_TEXTURE_2D */
#define GL_PROXY_TEXTURE_1D 0x8063
#define GL_PROXY_TEXTURE_2D 0x8064

/* TextureWrapMode */
#define GL_CLAMP 0x2900
#define GL_REPEAT 0x2901

/* AttribMask */
#define GL_CURRENT_BIT 0x00000001
#define GL_POINT_BIT 0x00000002
#define GL_LINE_BIT 0x00000004
#define GL_POLYGON_BIT 0x00000008
#define GL_POLYGON_STIPPLE_BIT 0x00000010
#define GL_PIXEL_MODE_BIT 0x00000020
#define GL_LIGHTING_BIT 0x00000040
#define GL_FOG_BIT 0x00000080
#define GL_DEPTH_BUFFER_BIT 0x00000100
#define GL_ACCUM_BUFFER_BIT 0x00000200
#define GL_STENCIL_BUFFER_BIT 0x00000400
#define GL_VIEWPORT_BIT 0x00000800
#define GL_TRANSFORM_BIT 0x00001000
#define GL_ENABLE_BIT 0x00002000
#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_HINT_BIT 0x00008000
#define GL_EVAL_BIT 0x00010000
#define GL_LIST_BIT 0x00020000
#define GL_TEXTURE_BIT 0x00040000
#define GL_SCISSOR_BIT 0x00080000
//#define GL_ALL_ATTRIB_BITS                0x000fffff

/* DrawBufferMode */
//#define GL_NONE                           0
#define GL_FRONT_LEFT 0x0400
#define GL_FRONT_RIGHT 0x0401
#define GL_BACK_LEFT 0x0402
#define GL_BACK_RIGHT 0x0403
#define GL_FRONT 0x0404
#define GL_BACK 0x0405
#define GL_LEFT 0x0406
#define GL_RIGHT 0x0407
#define GL_FRONT_AND_BACK 0x0408
#define GL_AUX0 0x0409
#define GL_AUX1 0x040A
#define GL_AUX2 0x040B
#define GL_AUX3 0x040C

/* texture */
#define GL_ALPHA4 0x803B
#define GL_ALPHA8 0x803C
#define GL_ALPHA12 0x803D
#define GL_ALPHA16 0x803E
#define GL_LUMINANCE4 0x803F
#define GL_LUMINANCE8 0x8040
#define GL_LUMINANCE12 0x8041
#define GL_LUMINANCE16 0x8042
#define GL_LUMINANCE4_ALPHA4 0x8043
#define GL_LUMINANCE6_ALPHA2 0x8044
#define GL_LUMINANCE8_ALPHA8 0x8045
#define GL_LUMINANCE12_ALPHA4 0x8046
#define GL_LUMINANCE12_ALPHA12 0x8047
#define GL_LUMINANCE16_ALPHA16 0x8048
#define GL_INTENSITY 0x8049
#define GL_INTENSITY4 0x804A
#define GL_INTENSITY8 0x804B
#define GL_INTENSITY12 0x804C
#define GL_INTENSITY16 0x804D
#define GL_R3_G3_B2 0x2A10
#define GL_RGB4 0x804F
#define GL_RGB5 0x8050
#define GL_RGB8 0x8051
#define GL_RGB10 0x8052
#define GL_RGB12 0x8053
#define GL_RGB16 0x8054
#define GL_RGBA2 0x8055
#define GL_RGBA4 0x8056
#define GL_RGB5_A1 0x8057
#define GL_RGBA8 0x8058
#define GL_RGB10_A2 0x8059
#define GL_RGBA12 0x805A
#define GL_RGBA16 0x805B
#define GL_TEXTURE_RED_SIZE 0x805C
#define GL_TEXTURE_GREEN_SIZE 0x805D
#define GL_TEXTURE_BLUE_SIZE 0x805E
#define GL_TEXTURE_ALPHA_SIZE 0x805F
#define GL_TEXTURE_LUMINANCE_SIZE 0x8060
#define GL_TEXTURE_INTENSITY_SIZE 0x8061
#define GL_PROXY_TEXTURE_1D 0x8063
#define GL_PROXY_TEXTURE_2D 0x8064

#ifndef GL_SGIX_shadow
#define GL_TEXTURE_COMPARE_SGIX 0x819A
#define GL_TEXTURE_COMPARE_OPERATOR_SGIX 0x819B
#define GL_TEXTURE_LEQUAL_R_SGIX 0x819C
#define GL_TEXTURE_GEQUAL_R_SGIX 0x819D
#endif

/*
 * GLUT API macro definitions -- the glutSetCursor parameters
 */
#define GLUT_CURSOR_RIGHT_ARROW 0x0000
#define GLUT_CURSOR_LEFT_ARROW 0x0001
#define GLUT_CURSOR_INFO 0x0002
#define GLUT_CURSOR_DESTROY 0x0003
#define GLUT_CURSOR_HELP 0x0004
#define GLUT_CURSOR_CYCLE 0x0005
#define GLUT_CURSOR_SPRAY 0x0006
#define GLUT_CURSOR_WAIT 0x0007
#define GLUT_CURSOR_TEXT 0x0008
#define GLUT_CURSOR_CROSSHAIR 0x0009
#define GLUT_CURSOR_UP_DOWN 0x000A
#define GLUT_CURSOR_LEFT_RIGHT 0x000B
#define GLUT_CURSOR_TOP_SIDE 0x000C
#define GLUT_CURSOR_BOTTOM_SIDE 0x000D
#define GLUT_CURSOR_LEFT_SIDE 0x000E
#define GLUT_CURSOR_RIGHT_SIDE 0x000F
#define GLUT_CURSOR_TOP_LEFT_CORNER 0x0010
#define GLUT_CURSOR_TOP_RIGHT_CORNER 0x0011
#define GLUT_CURSOR_BOTTOM_RIGHT_CORNER 0x0012
#define GLUT_CURSOR_BOTTOM_LEFT_CORNER 0x0013
#define GLUT_CURSOR_INHERIT 0x0064
#define GLUT_CURSOR_NONE 0x0065
#define GLUT_CURSOR_FULL_CROSSHAIR 0x0066

/* GL_NV_register_combiners */
/* Constants */
#define GL_REGISTER_COMBINERS_NV 0x8522

#define GLUquadricObj void

void SOLR_API glLoadIdentity();

void SOLR_API glBegin(GLint);
int SOLR_API glEnd();
void SOLR_API glEnable(GLenum cap);
void SOLR_API glDisable(GLenum cap);
void SOLR_API glClear(GLbitfield mask);
void SOLR_API glFlush();

void SOLR_API glVertex2i(GLint x, GLint y);
void SOLR_API glVertex3f(GLfloat x, GLfloat y, GLfloat z);
void SOLR_API glVertex3fv(const GLfloat *v);
void SOLR_API glNormal3f(GLfloat x, GLfloat y, GLfloat z);
void SOLR_API glNormal3fv(const GLfloat *v);
void SOLR_API glColor3f(GLfloat red, GLfloat green, GLfloat blue);
void SOLR_API glColor4f(GLfloat red, GLfloat green, GLfloat blue,
                        GLfloat alpha);
void SOLR_API glRasterPos2f(GLfloat x, GLfloat y);
void SOLR_API glRasterPos3f(GLfloat x, GLfloat y, GLfloat z);
void SOLR_API glTexParameterf(GLenum target, GLenum pname, GLfloat param);
void SOLR_API glTexCoord2f(GLfloat s, GLfloat t);
void SOLR_API glTexCoord3f(GLfloat x, GLfloat y, GLfloat z);
void SOLR_API glTexEnvf(GLenum target, GLenum pname, GLfloat param);
void SOLR_API glTexImage2D(GLenum target, GLint level, GLint internalformat,
                           GLsizei width, GLsizei height, GLint border,
                           GLenum format, GLenum type, const GLvoid *pixels);
void SOLR_API gluSphere(void *, GLfloat, GLint, GLint);
void SOLR_API glutWireSphere(GLdouble radius, GLint slices, GLint stacks);
SOLR_API GLUquadricObj *gluNewQuadric();
void SOLR_API glClearColor(GLfloat red, GLfloat green, GLfloat blue,
                           GLfloat alpha);

void SOLR_API glViewport(GLint a, GLint b, GLint width, GLint height);

// Materials
void SOLR_API glMaterialfv(GLenum face, GLenum pname, const GLfloat *params);

// Textures
void SOLR_API glGenTextures(GLsizei n, GLuint *textures);
void SOLR_API glBindTexture(GLenum target, GLuint texture);
int SOLR_API gluBuild2DMipmaps(GLenum target, GLint components, GLint width,
                               GLint height, GLenum format, GLenum type,
                               const void *data);
void SOLR_API glTexSubImage2D(GLenum target, GLint level, GLint xoffset,
                              GLint yoffset, GLsizei width, GLsizei height,
                              GLenum format, GLenum type, const GLvoid *data);
void SOLR_API glPushAttrib(GLbitfield mask);
void SOLR_API glPopAttrib();
void SOLR_API glTexParameteri(GLenum target, GLenum pname, GLint param);
void SOLR_API glBlendFunc(GLenum sfactor, GLenum dfactor);
void SOLR_API glMatrixMode(GLenum mode);
void SOLR_API glPushMatrix();
void SOLR_API glPopMatrix();
SOLR_API GLenum glGetError();
void SOLR_API glOrtho(GLdouble left, GLdouble right, GLdouble bottom,
                      GLdouble top, GLdouble nearVal, GLdouble farVal);

void SOLR_API glTranslatef(GLfloat x, GLfloat y, GLfloat z);
void SOLR_API glRotatef(GLfloat angle, GLfloat x, GLfloat y, GLfloat z);
void SOLR_API glPointSize(GLfloat size);
void SOLR_API glReadPixels(GLint x, GLint y, GLsizei width, GLsizei height,
                           GLenum format, GLenum type, GLvoid *data);

// Raytracer specific
void SOLR_API setOpenCLPlatform(const int platform);
void SOLR_API setOpenCLDevice(const int device);
void SOLR_API glCompactBoxes();
void SOLR_API createRandomMaterials(bool update, bool lightsOnly);
void SOLR_API setAngles(GLfloat, GLfloat, GLfloat);
void SOLR_API render();
void SOLR_API gluLookAt(GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ,
                        GLdouble centerX, GLdouble centerY, GLdouble centerZ,
                        GLdouble upX, GLdouble upY, GLdouble upZ);
void SOLR_API gluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear,
                             GLdouble zFar);

// Glut
void SOLR_API glutSpecialFunc(void (*func)(int key, int x, int y));
void SOLR_API glutReshapeFunc(void (*func)(int width, int height));
void SOLR_API glutIdleFunc(void (*func)(void));
void SOLR_API glutSetCursor(int cursor);
void SOLR_API wglSwapIntervalEXT();

// TO BE DONE
#define glColor4fv(...)

#define glRasterPos2i(...)
#define glDrawPixels(...)
#define glGetFloatv(...)
#define glLightModelfv(...)
#define glDeleteTextures(...)
#define glLineWidth(...)
#define glLineStipple(...)
#define glScalef(...)
#define glFogi(...)
#define glFogfv(...)
#define glFogf(...)
#define glShadeModel(...)
#define glHint(...)
#define glClearDepth(...)
#define glDepthFunc(...)
#define glLightfv(...)
#define glLightModeli(...)
#define glMaterialf(...)
#define glFrontFace(...)
#define glMap2f(...)
#define glMapGrid2f(...)
#define glEvalMesh2(...)
#define glDeleteLists(...)
#define glPolygonMode(...)
#define glTexImage1D(...)
#define glColorMaterial(...)
#define glLoadMatrixf(...)
#define glColorMask(...)
#define glPolygonOffset(...)
#define glCullFace(...)
#define glCopyTexSubImage2D(...)
#define glMultMatrixf(...)
#define glutWireCube(...)
#define glTexGeni(...)
#define glTexGenfv(...)
#define glActiveTextureARB(...)
#define glTexEnvi(...)
#define glAlphaFunc(...)
#define glCombinerParameteriNV(...)
#define glCombinerOutputNV(...)
#define glCombinerInputNV(...)
#define glFinalCombinerInputNV(...)
#define glBlendEquationEXT(...)
#define glGenLists(...) 0
#define glNewList(...)
#define glEndList(...)
#define glutSolidTorus(...)
#define glCallList(...)
#define gluUnProject(...)
#define glGetIntegerv(...)
#define glGetDoublev(...)
#define glutSetWindowTitle(...)
#define glutCloseFunc(...)
#define glewInit(...)

#define glTranslated glTranslatef
#define glNormal3d glNormal3f
#define glVertex3d glVertex3f

// Aliases
#define glutSolidSphere(a, b, c) gluSphere(0, a, b, c)

// Extensions
#define SetUpARB_multitexture(...)
#define SetUpEXT_texture_env_combine(...)
#define SetUpNV_register_combiners(...)
#define SetUpSGIX_depth_texture(...)
#define SetUpSGIX_shadow(...)
#define SetUpEXT_blend_minmax(...)

// GLUT

/*
 * GLUT API macro definitions -- the special key codes:
 */
#define GLUT_KEY_F1 0x0001
#define GLUT_KEY_F2 0x0002
#define GLUT_KEY_F3 0x0003
#define GLUT_KEY_F4 0x0004
#define GLUT_KEY_F5 0x0005
#define GLUT_KEY_F6 0x0006
#define GLUT_KEY_F7 0x0007
#define GLUT_KEY_F8 0x0008
#define GLUT_KEY_F9 0x0009
#define GLUT_KEY_F10 0x000A
#define GLUT_KEY_F11 0x000B
#define GLUT_KEY_F12 0x000C
#define GLUT_KEY_LEFT 0x0064
#define GLUT_KEY_UP 0x0065
#define GLUT_KEY_RIGHT 0x0066
#define GLUT_KEY_DOWN 0x0067
#define GLUT_KEY_PAGE_UP 0x0068
#define GLUT_KEY_PAGE_DOWN 0x0069
#define GLUT_KEY_HOME 0x006A
#define GLUT_KEY_END 0x006B
#define GLUT_KEY_INSERT 0x006C

/*
 * GLUT API macro definitions -- mouse state definitions
 */
#define GLUT_LEFT_BUTTON 0x0000
#define GLUT_MIDDLE_BUTTON 0x0001
#define GLUT_RIGHT_BUTTON 0x0002
#define GLUT_DOWN 0x0000
#define GLUT_UP 0x0001
#define GLUT_LEFT 0x0000
#define GLUT_ENTERED 0x0001

/*
 * GLUT API macro definitions -- the display mode definitions
 */
#define GLUT_RGB 0x0000
#define GLUT_RGBA 0x0000
#define GLUT_INDEX 0x0001
#define GLUT_SINGLE 0x0000
#define GLUT_DOUBLE 0x0002
#define GLUT_ACCUM 0x0004
#define GLUT_ALPHA 0x0008
#define GLUT_DEPTH 0x0010
#define GLUT_STENCIL 0x0020
#define GLUT_MULTISAMPLE 0x0080
#define GLUT_STEREO 0x0100
#define GLUT_LUMINANCE 0x0200

/*
 * GLUT API macro definitions -- windows and menu related definitions
 */
#define GLUT_MENU_NOT_IN_USE 0x0000
#define GLUT_MENU_IN_USE 0x0001
#define GLUT_NOT_VISIBLE 0x0000
#define GLUT_VISIBLE 0x0001
#define GLUT_HIDDEN 0x0000
#define GLUT_FULLY_RETAINED 0x0001
#define GLUT_PARTIALLY_RETAINED 0x0002
#define GLUT_FULLY_COVERED 0x0003

/*
 * GLUT API macro definitions -- the glutGet parameters
 */
#define GLUT_WINDOW_X 0x0064
#define GLUT_WINDOW_Y 0x0065
#define GLUT_WINDOW_WIDTH 0x0066
#define GLUT_WINDOW_HEIGHT 0x0067
#define GLUT_WINDOW_BUFFER_SIZE 0x0068
#define GLUT_WINDOW_STENCIL_SIZE 0x0069
#define GLUT_WINDOW_DEPTH_SIZE 0x006A
#define GLUT_WINDOW_RED_SIZE 0x006B
#define GLUT_WINDOW_GREEN_SIZE 0x006C
#define GLUT_WINDOW_BLUE_SIZE 0x006D
#define GLUT_WINDOW_ALPHA_SIZE 0x006E
#define GLUT_WINDOW_ACCUM_RED_SIZE 0x006F
#define GLUT_WINDOW_ACCUM_GREEN_SIZE 0x0070
#define GLUT_WINDOW_ACCUM_BLUE_SIZE 0x0071
#define GLUT_WINDOW_ACCUM_ALPHA_SIZE 0x0072
#define GLUT_WINDOW_DOUBLEBUFFER 0x0073
#define GLUT_WINDOW_RGBA 0x0074
#define GLUT_WINDOW_PARENT 0x0075
#define GLUT_WINDOW_NUM_CHILDREN 0x0076
#define GLUT_WINDOW_COLORMAP_SIZE 0x0077
#define GLUT_WINDOW_NUM_SAMPLES 0x0078
#define GLUT_WINDOW_STEREO 0x0079
#define GLUT_WINDOW_CURSOR 0x007A

#define GLUT_SCREEN_WIDTH 0x00C8
#define GLUT_SCREEN_HEIGHT 0x00C9
#define GLUT_SCREEN_WIDTH_MM 0x00CA
#define GLUT_SCREEN_HEIGHT_MM 0x00CB
#define GLUT_MENU_NUM_ITEMS 0x012C
#define GLUT_DISPLAY_MODE_POSSIBLE 0x0190
#define GLUT_INIT_WINDOW_X 0x01F4
#define GLUT_INIT_WINDOW_Y 0x01F5
#define GLUT_INIT_WINDOW_WIDTH 0x01F6
#define GLUT_INIT_WINDOW_HEIGHT 0x01F7
#define GLUT_INIT_DISPLAY_MODE 0x01F8
#define GLUT_ELAPSED_TIME 0x02BC
#define GLUT_WINDOW_FORMAT_ID 0x007B

/*
 * GLUT API macro definitions -- fonts definitions
 *
 * Steve Baker suggested to make it binary compatible with GLUT:
 */
#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || \
    defined(__WATCOMC__)
#define GLUT_STROKE_ROMAN ((void *)0x0000)
#define GLUT_STROKE_MONO_ROMAN ((void *)0x0001)
#define GLUT_BITMAP_9_BY_15 ((void *)0x0002)
#define GLUT_BITMAP_8_BY_13 ((void *)0x0003)
#define GLUT_BITMAP_TIMES_ROMAN_10 ((void *)0x0004)
#define GLUT_BITMAP_TIMES_ROMAN_24 ((void *)0x0005)
#define GLUT_BITMAP_HELVETICA_10 ((void *)0x0006)
#define GLUT_BITMAP_HELVETICA_12 ((void *)0x0007)
#define GLUT_BITMAP_HELVETICA_18 ((void *)0x0008)
#else
/*
 * I don't really know if it's a good idea... But here it goes:
 */
extern void *glutStrokeRoman;
extern void *glutStrokeMonoRoman;
extern void *glutBitmap9By15;
extern void *glutBitmap8By13;
extern void *glutBitmapTimesRoman10;
extern void *glutBitmapTimesRoman24;
extern void *glutBitmapHelvetica10;
extern void *glutBitmapHelvetica12;
extern void *glutBitmapHelvetica18;
#endif

/*
 * GLUT API macro definitions -- the glutGetModifiers parameters
 */
#define GLUT_ACTIVE_SHIFT 0x0001
#define GLUT_ACTIVE_CTRL 0x0002
#define GLUT_ACTIVE_ALT 0x0004

/*
 * Raytracer
 */

// OpenCL constants
static int gOpenCLPlatform = 0;
static int gOpenCLDevice = 0;

typedef void GLUnurbsObj;

void Initialize(const int width, const int height, const char *openCLKernel);

/*
 * Initialization functions, see fglut_init.c
 */
void SOLR_API glutInit(int *pargc, char **argv);
void SOLR_API glutInitWindowPosition(int x, int y);
void SOLR_API glutInitWindowSize(int width, int height, const char *kernel = 0);
void SOLR_API glutInitDisplayMode(unsigned int displayMode);
void SOLR_API glutReshapeWindow(int width, int height);

/*
 * Process loop function, see freeglut_main.c
 */
void SOLR_API glutMainLoop(void);

/*
 * Window management functions, see freeglut_window.c
 */
int SOLR_API glutCreateWindow(const char *title);
void SOLR_API glutDestroyWindow(int window);
void SOLR_API glutFullScreen(void);

/*
 * State setting and retrieval functions, see freeglut_state.c
 */
int SOLR_API glutGet(GLenum query);
int SOLR_API glutDeviceGet(GLenum query);
int SOLR_API glutGetModifiers(void);
int SOLR_API glutLayerGet(GLenum query);

/*
 * Window-specific callback functions, see freeglut_callbacks.c
 */
void SOLR_API glutKeyboardFunc(void (*callback)(unsigned char, int, int));
void SOLR_API glutDisplayFunc(void (*callback)(void));
void SOLR_API glutMouseFunc(void (*callback)(int, int, int, int));
void SOLR_API glutMotionFunc(void (*callback)(int, int));

/*
 * Global callback functions, see freeglut_callbacks.c
 */
void SOLR_API glutTimerFunc(unsigned int time, void (*callback)(int),
                            int value);

/*
 * Menu stuff, see freeglut_menu.c
 */
int SOLR_API glutCreateMenu(void (*callback)(int menu));
void SOLR_API glutDestroyMenu(int menu);
void SOLR_API glutAddMenuEntry(const char *label, int value);
void SOLR_API glutAttachMenu(int button);

/*
 * Font stuff, see freeglut_font.c
 */
void SOLR_API glutBitmapString(void *font, const unsigned char *string);

/*
 * Display-connected functions, see freeglut_display.c
 */
void SOLR_API glutPostRedisplay(void);
void SOLR_API glutSwapBuffers(void);

/*
 * Nurbs
 */
void *gluNewNurbsRenderer();
#define gluNurbsProperty(...)
#define gluBeginSurface(...)
#define gluNurbsSurface(...)
#define gluEndSurface(...)

#ifdef __cplusplus
};
#endif // #__cplusplus
