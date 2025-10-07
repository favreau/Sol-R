# Sol-R ANARI Device

This directory contains the ANARI device implementation for Sol-R, allowing Sol-R to be used as an ANARI-compliant ray tracing library.

## Building with ANARI Support

To build Sol-R with ANARI device support:

1. Install the ANARI SDK:
   ```bash
   git clone https://github.com/KhronosGroup/ANARI-SDK.git
   cd ANARI-SDK
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/anari-install
   make -j && make install
   ```

2. Configure Sol-R with ANARI enabled:
   ```bash
   cd Sol-R
   mkdir build && cd build
   cmake .. -DSOLR_BUILD_ANARI_DEVICE=ON -Danari_DIR=/path/to/anari-install/lib/cmake/anari
   make -j
   ```

## Using the Sol-R ANARI Device

The Sol-R ANARI device can be used with any ANARI-compatible application:

```cpp
#include <anari/anari.h>

// Load the Sol-R device library
ANARILibrary lib = anariLoadLibrary("solr", nullptr, nullptr);
ANARIDevice device = anariNewDevice(lib, "default");

// Use standard ANARI calls
ANARICamera camera = anariNewCamera(device, "perspective");
ANARIRenderer renderer = anariNewRenderer(device, "pathtracer");
// ... etc
```

## Supported Features

The Sol-R ANARI device currently supports:

- **Cameras**: perspective, orthographic
- **Geometries**: triangle, sphere, cylinder, cone
- **Materials**: matte, physically-based
- **Lights**: directional, point, spot
- **Renderers**: raycast, pathtracer

## Device Parameters

- `solr.engine`: Rendering engine to use ("cuda", "opencl", "cpu")
- `solr.samples`: Number of samples per pixel (1-1024)
- `solr.maxDepth`: Maximum ray depth (1-32)

## Status

This is an initial implementation providing basic ANARI device functionality. The implementation currently provides placeholder objects and will need to be extended to fully integrate with Sol-R's existing ray tracing pipeline. 