/* 
* Copyright (C) 2014 Cyrille Favreau - All Rights Reserved
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
* Written by Cyrille Favreau <cyrille_favreau@hotmail.com>
*/

#include <fstream>
#include <map>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string.h>
#include <math.h>

#include "Logging.h"

#include "Consts.h"
#include "OBJReader.h"

const int NB_MAX_FACES=static_cast<int>(NB_MAX_PRIMITIVES*0.9f); // Max number of faces

OBJReader::OBJReader(void)
{
}

OBJReader::~OBJReader(void)
{
}

Vertex readVertex( const std::string& value )
{
   Vertex returnValue = {0.f,0.f,0.f};
   int item(0);
   std::string tmp;
   for( int i(0); i<value.length(); ++i)
   {
      if( value[i] != ' ') tmp += value[i];

      if( value[i] == ' ' || i==(value.length()-1) )
      {
         switch( item )
         {
         case 0: returnValue.x = static_cast<float>(atof(tmp.c_str())); break;
         case 1: returnValue.y = static_cast<float>(atof(tmp.c_str())); break;
         case 2: returnValue.z = static_cast<float>(atof(tmp.c_str())); break;
         }
         ++item;
         tmp = "";
      }
   }
   return returnValue;
}

int4 readFloat3( const std::string& face )
{
   int4 returnValue = {0,0,0,0};
   int item(0);
   std::string value;
   for( int i(0); i<face.length(); ++i)
   {
      if( face[i] != ' ') value += face[i];

      if( face[i] == ' ' || i==(face.length()-1) )
      {
         switch( item )
         {
         case 0: returnValue.x = atoi(value.c_str()); break;
         case 1: returnValue.y = atoi(value.c_str()); break;
         case 2: returnValue.z = atoi(value.c_str()); break;
         }
         ++item;
         value = "";
      }
   }
   return returnValue;
}

int4 readFace( const std::string& face )
{
   int4 returnValue = {0,0,0,0};
   int item(0);
   std::string value;
   for( int i(0); i<face.length(); ++i)
   {
      if( face[i] != '/') value += face[i];

      if( face[i] == '/' /*|| i==(face.length()-1)*/ )
      {
         switch( item )
         {
         case 0: returnValue.x = (atoi(value.c_str())); break;
         case 1: returnValue.y = (atoi(value.c_str())); break;
         case 2: returnValue.z = (atoi(value.c_str())); break;
         }
         ++item;
         value = "";
      }
   }

   if( value.length() != 0 )
   {
      switch( item )
      {
      case 0: returnValue.x = (atoi(value.c_str())); break;
      case 1: returnValue.y = (atoi(value.c_str())); break;
      case 2: returnValue.z = (atoi(value.c_str())); break;
      }
   }

   //std::cout << returnValue.x <<"," << returnValue.y << "," << returnValue.z << std::endl;
   return returnValue;
}

unsigned int OBJReader::loadMaterialsFromFile(
   const std::string& filename,
   std::map<std::string,MaterialMTL>& materials,
   GPUKernel& kernel,
   int materialId)
{
   LOG_INFO(1,"Loading material library from " << filename);
   const float innerDiffusion=1000.f;
   const float diffusionRatio=4.f;

   std::string id("");
   std::ifstream file(filename.c_str());
   if( file.is_open() )
   {
      while( file.good() )
      {
         std::string line;
         std::getline( file, line );
         line.erase( std::remove(line.begin(), line.end(), '\r'), line.end());

         // remove spaces
         while( line.length() != 0 && line[0] < 32 )
         {
            line = line.substr(1);
         }

         if( line.find("newmtl") == 0 )
         {
            if( id.length() != 0 )
            {
               MaterialMTL& m = materials[id];

               // Grey material is reflective!!! :-)
               //if(m.Kd.x>0.4f && m.Kd.x>0.6f && fabs(m.Kd.x-m.Kd.y)<0.01f && fabs(m.Kd.y-m.Kd.z)<0.01f) m.reflection=0.5f;

               // Add material to kernel
               kernel.setMaterial(
                  m.index,
                  m.Kd.x,m.Kd.y,m.Kd.z,
                  m.noise,m.reflection,m.refraction, false, false, 0,
                  m.transparency, m.opacity,
                  m.diffuseTextureId, m.normalTextureId, m.bumpTextureId, m.specularTextureId, m.reflectionTextureId, m.transparencyTextureId, m.ambientOcclusionTextureId,
                  m.Ks.x, 100.f*m.Ks.y, m.Ks.z,
                  m.illumination, innerDiffusion, innerDiffusion*diffusionRatio,
                  false );
               LOG_INFO(3, "[" << m.index << "] Added material [" << id << "] " <<
                  "( " << m.Kd.x << ", " << m.Kd.y << ", " << m.Kd.z << ") " <<
                  "( " << m.Ks.x << ", " << m.Ks.y << ", " << m.Ks.z << ") " <<
                  "( " << m.illumination << ") " <<
                  ", Textures [" << m.diffuseTextureId << "," << m.bumpTextureId << "]=" << kernel.getTextureFilename(m.diffuseTextureId));
            }
            id = line.substr(7);
            MaterialMTL material;
            memset( &material, 0, sizeof(MaterialMTL));
            material.index = static_cast<unsigned int>(materials.size()+materialId);
            //material.reflection = 0.2f;
            material.diffuseTextureId      = MATERIAL_NONE;
            material.normalTextureId       = MATERIAL_NONE;
            material.bumpTextureId         = MATERIAL_NONE;
            material.specularTextureId     = MATERIAL_NONE;
            material.reflectionTextureId   = MATERIAL_NONE;
            material.transparencyTextureId = MATERIAL_NONE;
            material.ambientOcclusionTextureId = MATERIAL_NONE;
            materials[id].isSketchupLightMaterial = false;
            material.opacity = 0.f;
            material.noise = 0.f;
            materials[id] = material;
            materials[id].Ks.x = 1.f;
            materials[id].Ks.y = 500.f;
         }

         if( line.find("Kd") == 0 )
         {
            // RGB Color
            line = line.substr(3);
            materials[id].Kd = readVertex(line);
            if( materials[id].isSketchupLightMaterial )
            {
               materials[id].illumination = (materials[id].Kd.x+materials[id].Kd.y+materials[id].Kd.z)/3.f;
            }
         }

         if( line.find("Ks") == 0 )
         {
            // Specular values
            line = line.substr(3);
            materials[id].Ks = readVertex(line);
         }

         bool diffuseMap=(line.find("map_Kd")==0);
         bool bumpMap=(line.find("map_bump")==0);
         bool normalMap=(line.find("map_norm")==0);
         bool specularlMap=(line.find("map_spec")==0);
         if( diffuseMap || bumpMap || normalMap || specularlMap )
         {
            if(diffuseMap) line = line.substr(7);
            if(bumpMap || normalMap || specularlMap) line = line.substr(9);
            std::string folder(filename);
            size_t backSlashPos = filename.rfind('/');
            if( backSlashPos==-1 )
            {
               backSlashPos = filename.rfind('\\');
            }
            if( backSlashPos != -1 )
            {
               folder = filename.substr(0, backSlashPos);
            }
            folder += '/';
            folder += line;
            int idx = kernel.getNbActiveTextures();
            if( kernel.loadTextureFromFile(idx, folder) )
            {
               if(diffuseMap) 
               {
                  materials[id].diffuseTextureId = idx;
                  LOG_INFO(3, "[Slot " << idx  << "] Diffuse texture " << folder << " successfully loaded and assigned to material " << id << "(" << materials[id].index << ")" );
               }
               if(bumpMap)
               {
                  materials[id].bumpTextureId = idx;
                  LOG_INFO(3, "[Slot " << idx  << "] Bump texture " << folder << " successfully loaded and assigned to material " << id << "(" << materials[id].index << ")" );
               }
               if(normalMap)
               {
                  materials[id].normalTextureId = idx;
                  LOG_INFO(3, "[Slot " << idx  << "] Mormal texture " << folder << " successfully loaded and assigned to material " << id << "(" << materials[id].index << ")" );
               }
               if(specularlMap)
               {
                  materials[id].specularTextureId = idx;
                  LOG_INFO(3, "[Slot " << idx  << "] Specular texture " << folder << " successfully loaded and assigned to material " << id << "(" << materials[id].index << ")" );
               }
            }
            else
            {
               LOG_ERROR("Failed to load texture " << folder );
            }
         }

         if( /*line.find("d")==0 ||*/ line.find("Tr")==0 )
         {
            // Specular values
            line = line.substr(2);
            float d=static_cast<float>(atof(line.c_str()));
            materials[id].reflection   = 1.f; 
            materials[id].transparency = 0.5f+(d/50.f);
            materials[id].refraction   = 1.1f;
            materials[id].noise = 0.f;
         }

         if( line.find("SoL_R_Light") != -1 )
         {
            materials[id].isSketchupLightMaterial = true;
         }

         if( line.find("illum") == 0 )
         {
            line = line.substr(6);
            int illum=static_cast<int>(atoi(line.c_str()));
            switch(illum)
            {
            case 3:
            case 5:
            case 8:
               // Reflection
               materials[id].transparency = 0.f;
               break;
            case 6:
            case 7:
            case 9:
               // Transparency
               break;
            default:
               materials[id].reflection   = 0.f;
               materials[id].transparency = 0.f;
               materials[id].refraction   = 0.f;
            }
         }
      }

      // Last remaining material
      if( id.length() != 0 )
      {
         MaterialMTL& m = materials[id];

         // Grey material is reflective!!! :-)
         //if(m.Kd.x>0.4f && m.Kd.x>0.6f && fabs(m.Kd.x-m.Kd.y)<0.01f && fabs(m.Kd.y-m.Kd.z)<0.01f) m.reflection=0.5f;

         // Add material to kernel
         kernel.setMaterial(
            m.index,
            m.Kd.x,m.Kd.y,m.Kd.z,
            m.noise,m.reflection,m.refraction, false, false, 0,
            m.transparency, m.opacity,
            m.diffuseTextureId, m.normalTextureId, m.bumpTextureId, m.specularTextureId, m.reflectionTextureId, m.transparencyTextureId, m.ambientOcclusionTextureId,
            m.Ks.x, 100.f*m.Ks.y, m.Ks.z,
            m.illumination, innerDiffusion, innerDiffusion*diffusionRatio,
            false );
         LOG_INFO(3, "[" << m.index << "] Added material [" << id << "] " <<
            "( " << m.Kd.x << ", " << m.Kd.y << ", " << m.Kd.z << ") " <<
            "( " << m.Ks.x << ", " << m.Ks.y << ", " << m.Ks.z << ") " <<
            "( " << m.illumination << ") " <<
            ", Textures [" << m.diffuseTextureId << "," << m.bumpTextureId << "]=" << kernel.getTextureFilename(m.diffuseTextureId));
      }

      file.close();
   }
   return 0;
}

void OBJReader::addLightComponent(
   GPUKernel& kernel,
   std::vector<Vertex>& solrVertices,
   const Vertex& center,
   const Vertex& objectCenter,
   const Vertex& objectScale,
   const int material,
   CPUBoundingBox& aabb)
{
   size_t len=solrVertices.size();
   if( len!=0 )
   {
      Vertex lightCenter={0.f,0.f,0.f};
      aabb.parameters[0].x = 1000000.f;
      aabb.parameters[0].y = 1000000.f;
      aabb.parameters[0].z = 1000000.f;
      aabb.parameters[1].x = -1000000.f;
      aabb.parameters[1].y = -1000000.f;
      aabb.parameters[1].z = -1000000.f;
      for( size_t i(0); i<len;++i)
      {
         aabb.parameters[0].x = std::min(solrVertices[i].x,aabb.parameters[0].x);
         aabb.parameters[1].x = std::max(solrVertices[i].x,aabb.parameters[1].x);
         aabb.parameters[0].y = std::min(solrVertices[i].y,aabb.parameters[0].y);
         aabb.parameters[1].y = std::max(solrVertices[i].y,aabb.parameters[1].y);
         aabb.parameters[0].z = std::min(solrVertices[i].z,aabb.parameters[0].z);
         aabb.parameters[1].z = std::max(solrVertices[i].z,aabb.parameters[1].z);
      }
      lightCenter.x = (aabb.parameters[1].x+aabb.parameters[0].x)/2.f;
      lightCenter.y = (aabb.parameters[1].y+aabb.parameters[0].y)/2.f;
      lightCenter.z = (aabb.parameters[1].z+aabb.parameters[0].z)/2.f;
      Vertex L={aabb.parameters[1].x-aabb.parameters[0].x,aabb.parameters[1].y-aabb.parameters[0].y,aabb.parameters[1].z-aabb.parameters[0].z};
      float radius = sqrt(L.x*L.x+L.y*L.y+L.z*L.z)/2.f;

      int nbPrimitives = kernel.addPrimitive( ptSphere );
      LOG_INFO(1,"Adding SoL-R light [" << material << "] to primitive " << nbPrimitives << " (" << lightCenter.x << "," << lightCenter.y << "," << lightCenter.z << ") r=" << radius );
      kernel.setPrimitive( 
         nbPrimitives,
         center.x+objectScale.x*(-objectCenter.x+lightCenter.x),
         center.y+objectScale.y*(-objectCenter.y+lightCenter.y),
         center.z+objectScale.z*(-objectCenter.z+lightCenter.z),
         radius, 0.f, 0.f,
         material);
      kernel.setPrimitiveBellongsToModel(nbPrimitives,true);
   }
   solrVertices.clear();
}

Vertex OBJReader::loadModelFromFile(
   const std::string& filename,
   GPUKernel& kernel,
   const Vertex& objectPosition,
   const bool autoScale,
   const Vertex& scale,
   bool loadMaterials,
   int materialId,
   bool allSpheres,
   bool autoCenter,
   CPUBoundingBox& aabb,
   const bool& checkInAABB,
   const CPUBoundingBox& inAABB)
{
   LOG_INFO(1,"Loading OBJ file " << filename );

   std::map<int,Vertex> vertices;
   std::map<int,Vertex> normals;
   std::map<int,Vertex> textureCoordinates;
   std::map<std::string,MaterialMTL> materials;

   int index_vertices(1);
   int index_normals(1);
   int index_textureCoordinates(1);

   std::string noExtFilename(filename);
   size_t pos(noExtFilename.find(".obj"));
   if( pos != -1 )
   {
      noExtFilename = filename.substr(0, pos);
   }
   std::replace(noExtFilename.begin(), noExtFilename.end(), '\\', '/');

   // Load model vertices
   std::string modelFilename(noExtFilename);
   modelFilename += ".obj";

   Vertex objectSize = {0.f,0.f,0.f};
#if 0
   std::ifstream file(filename.c_str());
   if( file.is_open() )
   {
      while( file.good() )
      {
         std::string id;
         file >> id;
         if(id[0]=='v' && id[1]==0)
         {
            Vertex v;
            file >> v.x >> v.y >> v.z;
            vertices[index_vertices] = v;
            ++index_vertices;
            LOG_INFO(1, "v = " << v.x << "," << v.y << "," << v.z); 
         }
         if(id[0]=='v' && id[1]=='t')
         {
            Vertex v;
            file >> v.x >> v.y >> v.z;
            textureCoordinates[index_textureCoordinates] = v;
            ++index_textureCoordinates;
            //LOG_INFO(1, "t = " << v.x << "," << v.y << "," << v.z); 
         }
         if(id[0]=='v' && id[1]=='n')
         {
            Vertex v;
            file >> v.x >> v.y >> v.z;
            normals[index_normals] = v;
            ++index_normals;
            //LOG_INFO(1, "n = " << v.x << "," << v.y << "," << v.z); 
         }
         file.clear();
      }
      file.close();
   }
   LOG_INFO(1,"Loaded");

   LOG_INFO(1,"Vertices : " << index_vertices );
   LOG_INFO(1,"Normals  : " << index_normals );
   LOG_INFO(1,"TexCoords: " << index_textureCoordinates );

   return objectSize;
#else
   aabb.parameters[0].x = 100000.f;
   aabb.parameters[0].y = 100000.f;
   aabb.parameters[0].z = 100000.f;
   aabb.parameters[1].x = -100000.f;
   aabb.parameters[1].y = -100000.f;
   aabb.parameters[1].z = -100000.f;

   // Read vertices
   std::ifstream file(modelFilename.c_str());
   if( file.is_open() )
   {
      while( file.good() )
      {
         std::string line;
         std::getline( file, line );
         line.erase( std::remove(line.begin(), line.end(), '\r'), line.end());
         if( line.length() > 1 ) 
         {
            if( loadMaterials && line.find("mtllib")!=std::string::npos )
            {
               // Load materials
               std::string materialFileName=line.substr(7);
               std::string folder=noExtFilename.substr(0,noExtFilename.rfind('/'));
               materialFileName=folder+'/'+materialFileName;
               loadMaterialsFromFile( materialFileName, materials, kernel, materialId );
            }
            if( line[0] == 'v' )
            {
               // Vertices
               Vertex vertex = {0.f,0.f,0.f};
               std::string value("");

               size_t i(1);
               int item(0);
               char previousChar = line[0];
               while( i<line.length() && item<4)
               {
                  if( line[i] == ' '  && previousChar != ' ') 
                  {
                     switch( item )
                     {
                     case 1: vertex.x = static_cast<float>(atof(value.c_str())); break;
                     case 2: vertex.y = static_cast<float>(atof(value.c_str())); break;
                     case 3: vertex.z = static_cast<float>(atof(value.c_str())); break;
                     }
                     ++item;
                     value = "";
                  }
                  else
                  {
                     value += line[i];
                  }

                  previousChar = line[i];
                  ++i;
               }

               // Process last element
               if( value.length() != 0 )
               {
                  switch( item )
                  {
                  case 1: vertex.x = static_cast<float>(atof(value.c_str())); break;
                  case 2: vertex.y = static_cast<float>(atof(value.c_str())); break;
                  case 3: vertex.z = static_cast<float>(atof(value.c_str())); break;
                  }
               }

               if( line[1] == 'n' )
               {  
                  // Normals
                  vertex.z = -vertex.z;
                  normals[index_normals] = vertex;
                  ++index_normals;
               }
               else if( line[1] == 't' )
               {  
                  vertex.x -= static_cast<int>(vertex.x);
                  vertex.y -= static_cast<int>(vertex.y);

                  // Texture coordinates
                  if( vertex.x<0.f )
                  {
                     int Xa = static_cast<int>(fabs(vertex.x));
                     float Xb = fabs(vertex.x)-Xa;
                     vertex.x = Xb;
                  }

                  if( vertex.y<0.f )
                  {
                     int Ya = static_cast<int>(fabs(vertex.y));
                     float Yb = fabs(vertex.y)-Ya;
                     vertex.y = Yb;
                  }

                  if( vertex.z<0.f )
                  {
                     int Za = static_cast<int>(fabs(vertex.z));
                     float Zb = fabs(vertex.z)-Za;
                     vertex.z = Zb;
                  }
                  //LOG_INFO(1,"vt=" << vertex.x << "," << vertex.y << "," << vertex.z );
                  textureCoordinates[index_textureCoordinates] = vertex;
                  ++index_textureCoordinates;
               }
               else
               {
                  // Vertex
                  if( line[1] == ' ' ) 
                  {
                     vertex.z = -vertex.z;
                     vertices[index_vertices] = vertex;
                     ++index_vertices;

                     // min
                     aabb.parameters[0].x = (vertex.x < aabb.parameters[0].x) ? vertex.x : aabb.parameters[0].x;
                     aabb.parameters[0].y = (vertex.y < aabb.parameters[0].y) ? vertex.y : aabb.parameters[0].y;
                     aabb.parameters[0].z = (vertex.z < aabb.parameters[0].z) ? vertex.z : aabb.parameters[0].z;

                     // max
                     aabb.parameters[1].x = (vertex.x > aabb.parameters[1].x) ? vertex.x : aabb.parameters[1].x;
                     aabb.parameters[1].y = (vertex.y > aabb.parameters[1].y) ? vertex.y : aabb.parameters[1].y;
                     aabb.parameters[1].z = (vertex.z > aabb.parameters[1].z) ? vertex.z : aabb.parameters[1].z;
                  }
               }

            }
         }
      }
      file.close();
   }

   if( checkInAABB )
   {
      if( aabb.parameters[0].x<inAABB.parameters[0].x ) return objectSize;
      if( aabb.parameters[0].y<inAABB.parameters[0].y ) return objectSize;
      if( aabb.parameters[0].z<inAABB.parameters[0].z ) return objectSize;

      if( aabb.parameters[1].x>inAABB.parameters[1].x ) return objectSize;
      if( aabb.parameters[1].y>inAABB.parameters[1].y ) return objectSize;
      if( aabb.parameters[1].z>inAABB.parameters[1].z ) return objectSize;
   }

   LOG_INFO(1,"Object contains " << vertices.size() << " vertices");

   // Scale object
   Vertex objectCenter = objectPosition;
   Vertex objectScale  = scale;
   if( autoScale )
   {
      float os = std::max( aabb.parameters[1].x - aabb.parameters[0].x, std::max ( aabb.parameters[1].y - aabb.parameters[0].y, aabb.parameters[1].z - aabb.parameters[0].z ));
      objectScale.x = scale.x/os;
      objectScale.y = scale.y/os;
      objectScale.z = scale.z/os;

      if(autoCenter)
      {
         // Center align object
         objectCenter.x = (aabb.parameters[0].x+aabb.parameters[1].x) / 2.f;
         objectCenter.y = (aabb.parameters[0].y+aabb.parameters[1].y) / 2.f;
         objectCenter.z = (aabb.parameters[0].z+aabb.parameters[1].z) / 2.f;
      }
   }

   // Read faces
   file.open(modelFilename.c_str());
   if( file.is_open() )
   {
      int material(materialId);
      int sketchupMaterial(MATERIAL_NONE);
      bool isSketchupLightMaterial(false);

      std::vector<Vertex> solrVertices;
      std::string component;
      std::string line;
      while( file.good() && kernel.getNbActivePrimitives()<NB_MAX_FACES )
      {
         int nbPrimitives(0);
         std::getline( file, line );
         line.erase( std::remove(line.begin(), line.end(), '\r'), line.end());
         if( line.length() != 0 ) 
         {
            // Compoment
            if( line.find("g")==0 )
            {
               isSketchupLightMaterial=(line.find("SoL_R")!=-1);
               if( isSketchupLightMaterial )
               {
                  if( line!=component )
                  {
                     addLightComponent(kernel,solrVertices,objectPosition,objectCenter,objectScale,sketchupMaterial,aabb);
                  }
                  component=line;
               }
            }

            if( line.find("usemtl") == 0 && line.length()>7)
            {
               std::string value = line.substr(7);
               if( materials.find(value) != materials.end() )
               {
                  MaterialMTL& m=materials[value];
                  material = m.index;
                  if( isSketchupLightMaterial )
                  {
                     LOG_INFO(1,"Sketchup Material " << material);
                     sketchupMaterial=material;
                  }
               }
               else
               {
                  LOG_ERROR("Unknown Material " << value);
               }
            }

            if( line[0] == 'f' )
            {
               std::string value("");

               std::vector<int4> face;
               size_t i(1);
               char previousChar = line[0];
               while( i<line.length() )
               {
                  if(line[i] != ' ') value += line[i];
                  if( i==(line.length()-1) || (line[i] == ' ' && previousChar != ' ')) 
                  {
                     if( value.length() != 0 )
                     {
                        int4 v = readFace(value);
                        face.push_back(v);
                        value = "";
                     }
                  }

                  previousChar = line[i];
                  ++i;
               }

               int f(0);
               if( allSpheres || isSketchupLightMaterial )
               {
                  Vertex sphereCenter;
                  sphereCenter.x = (vertices[face[f].x].x + vertices[face[f+1].x].x + vertices[face[f+2].x].x)/3.f;
                  sphereCenter.y = (vertices[face[f].x].y + vertices[face[f+1].x].y + vertices[face[f+2].x].y)/3.f;
                  sphereCenter.z = (vertices[face[f].x].z + vertices[face[f+1].x].z + vertices[face[f+2].x].z)/3.f;

                  Vertex sphereRadius[3];
                  for( int i=0; i<3; ++i)
                  {
                    sphereRadius[i].x = (sphereCenter.x - vertices[face[f+i].x].x);
                    sphereRadius[i].y = (sphereCenter.y - vertices[face[f+i].x].y);
                    sphereRadius[i].z = (sphereCenter.z - vertices[face[f+i].x].z);
                  }
                  Vertex s;
                  s.x = std::max(sphereRadius[0].x, std::max(sphereRadius[1].x, sphereRadius[2].x ));
                  s.y = std::max(sphereRadius[0].y, std::max(sphereRadius[1].y, sphereRadius[2].y ));
                  s.z = std::max(sphereRadius[0].z, std::max(sphereRadius[1].z, sphereRadius[2].z ));

                  //float radius = objectScale.x*sqrt(s.x*s.x+s.y*s.y+s.z*s.z);

                  if( isSketchupLightMaterial )
                  {
                     solrVertices.push_back(sphereCenter);
                  }
                  else
                  {
                     nbPrimitives = kernel.addPrimitive( ptEllipsoid );
                     kernel.setPrimitive( 
                        nbPrimitives,
                        objectPosition.x+objectScale.x*(-objectCenter.x+sphereCenter.x),
                        objectPosition.y+objectScale.y*(-objectCenter.y+sphereCenter.y),
                        objectPosition.z+objectScale.z*(-objectCenter.z+sphereCenter.z),
                        objectScale.x*s.x, objectScale.y*s.y, objectScale.z*s.z,
                        material);
                     kernel.setPrimitiveBellongsToModel(nbPrimitives,true);
                  }
               }
               else
               {
                  nbPrimitives = kernel.addPrimitive( ptTriangle );
                  kernel.setPrimitive( 
                     nbPrimitives,
                     objectPosition.x+objectScale.x*(-objectCenter.x+vertices[face[f  ].x].x),objectPosition.y+objectScale.y*(-objectCenter.y+vertices[face[f  ].x].y),objectPosition.z+objectScale.z*(-objectCenter.z+vertices[face[f  ].x].z),
                     objectPosition.x+objectScale.x*(-objectCenter.x+vertices[face[f+1].x].x),objectPosition.y+objectScale.y*(-objectCenter.y+vertices[face[f+1].x].y),objectPosition.z+objectScale.z*(-objectCenter.z+vertices[face[f+1].x].z),
                     objectPosition.x+objectScale.x*(-objectCenter.x+vertices[face[f+2].x].x),objectPosition.y+objectScale.y*(-objectCenter.y+vertices[face[f+2].x].y),objectPosition.z+objectScale.z*(-objectCenter.z+vertices[face[f+2].x].z),
                     0.f, 0.f, 0.f,
                     material);
                  kernel.setPrimitiveBellongsToModel(nbPrimitives,true);
               }

               // Texture coordinates
               kernel.setPrimitiveTextureCoordinates( 
                  nbPrimitives, textureCoordinates[face[f].y], textureCoordinates[face[f+1].y], textureCoordinates[face[f+2].y] );

               // Normals
               kernel.setPrimitiveNormals( 
                  nbPrimitives, normals[face[f].z], normals[face[f+1].z], normals[face[f+2].z] );

               if( face.size() == 4 )
               {
                  //LOG_ERROR("Only triangles are currently supported");
                  if( allSpheres )
                  {
                     Vertex sphereCenter;
                     sphereCenter.x = (vertices[face[f+3].x].x + vertices[face[f+2].x].x + vertices[face[f].x].x)/3.f;
                     sphereCenter.y = (vertices[face[f+3].x].y + vertices[face[f+2].x].y + vertices[face[f].x].y)/3.f;
                     sphereCenter.z = (vertices[face[f+3].x].z + vertices[face[f+2].x].z + vertices[face[f].x].z)/3.f;

                     Vertex sphereRadius;
                     sphereRadius.x = sphereCenter.x - vertices[face[f].x].x;
                     sphereRadius.y = sphereCenter.y - vertices[face[f].x].y;
                     sphereRadius.z = sphereCenter.z - vertices[face[f].x].z;

                     float radius = 100.f; //objectScale*sqrt(sphereRadius.x*sphereRadius.x+sphereRadius.y*sphereRadius.y+sphereRadius.z*sphereRadius.z);

                     nbPrimitives = kernel.addPrimitive( ptSphere );
                     kernel.setPrimitive( 
                        nbPrimitives,
                        objectPosition.x+objectScale.x*(-objectCenter.x+sphereCenter.x),
                        objectPosition.y+objectScale.y*(-objectCenter.y+sphereCenter.y),
                        objectPosition.z+objectScale.z*(-objectCenter.z+sphereCenter.z),
                        radius, 0.f, 0.f,
                        material);
                     kernel.setPrimitiveBellongsToModel(nbPrimitives,true);
                  }
                  else
                  {
                     nbPrimitives = kernel.addPrimitive( ptTriangle );
                     kernel.setPrimitive( 
                        nbPrimitives, 
                        objectPosition.x+objectScale.x*(-objectCenter.x+vertices[face[f+3].x].x),objectPosition.y+objectScale.y*(-objectCenter.y+vertices[face[f+3].x].y),objectPosition.z+objectScale.z*(-objectCenter.z+vertices[face[f+3].x].z),
                        objectPosition.x+objectScale.x*(-objectCenter.x+vertices[face[f+2].x].x),objectPosition.y+objectScale.y*(-objectCenter.y+vertices[face[f+2].x].y),objectPosition.z+objectScale.z*(-objectCenter.z+vertices[face[f+2].x].z),
                        objectPosition.x+objectScale.x*(-objectCenter.x+vertices[face[f  ].x].x),objectPosition.y+objectScale.y*(-objectCenter.y+vertices[face[f  ].x].y),objectPosition.z+objectScale.z*(-objectCenter.z+vertices[face[f  ].x].z),
                        0.f, 0.f, 0.f,
                        material);
                     kernel.setPrimitiveBellongsToModel(nbPrimitives,true);
                  }
                  // Texture coordinates
                  kernel.setPrimitiveTextureCoordinates( 
                     nbPrimitives, textureCoordinates[face[f+3].y], textureCoordinates[face[f+2].y], textureCoordinates[face[f].y] );
                  kernel.setPrimitiveNormals( 
                     nbPrimitives, normals[face[f+3].z], normals[face[f+2].z], normals[face[f].z] );
               }
            }
         }
      }
      file.close();

      // Remaining SoL-R lights
      if( solrVertices.size()!=0 )
      {
         addLightComponent(kernel,solrVertices,objectPosition,objectCenter,objectScale,sketchupMaterial,aabb);
      }

   }

   objectSize.x = objectScale.x*(aabb.parameters[1].x - aabb.parameters[0].x);
   objectSize.y = objectScale.y*(aabb.parameters[1].y - aabb.parameters[0].y);
   objectSize.z = objectScale.z*(aabb.parameters[1].z - aabb.parameters[0].z);

   LOG_INFO(1,"--------------------------------------------------------------------------------");
   LOG_INFO(1, "Loaded " << modelFilename.c_str() << " into frame " << kernel.getFrame() << " [" << kernel.getNbActivePrimitives() << " primitives]" );
   LOG_INFO(1, "Nb Vertices: " << kernel.getNbActivePrimitives() );
   LOG_INFO(1, "Min        : " << aabb.parameters[0].x << "," << aabb.parameters[0].y << "," << aabb.parameters[0].z );
   LOG_INFO(1, "Max        : " << aabb.parameters[1].x << "," << aabb.parameters[1].y << "," << aabb.parameters[1].z );
   LOG_INFO(1, "Center     : " << objectCenter.x << "," << objectCenter.y << "," << objectCenter.z );
   LOG_INFO(1, "Scale      : " << objectScale.x << "," << objectScale.y << "," << objectScale.z );
   LOG_INFO(1, "Object size: " << objectSize.x << "," << objectSize.y << "," << objectSize.z );
   LOG_INFO(1, "AABB       : (" << aabb.parameters[0].x << "," << aabb.parameters[0].y << "," << aabb.parameters[0].z << "),(" << aabb.parameters[1].x << "," << aabb.parameters[1].y << "," << aabb.parameters[1].z << ")" );
   LOG_INFO(1, "inAABB     : (" << inAABB.parameters[0].x << "," << inAABB.parameters[0].y << "," << inAABB.parameters[0].z << "),(" << inAABB.parameters[1].x << "," << inAABB.parameters[1].y << "," << inAABB.parameters[1].z << ")" );

   LOG_INFO(1,"--------------------------------------------------------------------------------");
   return objectSize;
#endif 
}
