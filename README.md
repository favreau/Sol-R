[![Build Status](https://travis-ci.org/favreau/Sol-R.svg?branch=master)](https://travis-ci.org/favreau/Sol-R)

# Welcome to Sol-R, the Speed Of Light Ray-tracer

## Note about the Blue Brain BioExplorer, a Sol-R sequel

If you like Sol-R, but you think it's now getting a bit old, you will love my new open-source, the [Blue Brain BioExplorer](https://github.com/BlueBrain/BioExplorer). Check-it out and star the GitHub Repository to support the project.

## Introduction
Sol-R is a CUDA/OpenCL-based realtime ray-tracer compatible with Oculus Rift DK1, Kinect, Razor Hydra and Leap Motion devices.
Sol-R was used by the Interactive Molecular Visualizer project (http://www.molecular-visualization.com)

A number of videos can be found on my channel: https://www.youtube.com/user/CyrilleOnDrums

Sol-R was written as a hobby project in order to understand and learn more about CUDA and OpenCL. Most of the code was written at night and during week-ends, meaning that it's probably not the best quality ever ;-)

The idea was to produce a Ray-Tracer that has its own "personality". Most of the code does not rely on any literature about ray-tracing, but more on a naive approach of what rays could be used for. A simple engine that could produce cool images interactively.

Take it for what it is! Sol-R is a lot of fun to play with if you like coding computer generated images.

May the fun continue with your contributions! :-)

```
usage: solrViewer
```

## Prerequeries

### Mandatory
- CMake 3.5
- Glew 2.x
- Glut 3.7
- Cuda 8.0 or OpenCL 1.2

### Optional
- Kinect SDK 1.8
- Oculus Rift DK1 SDK 0.2.5
- Sixense SDK
- Leap SDK 3.2.0

## Build from source
```
mkdir Build
cd Build
cmake .. -DCMAKE_PREFIX_PATH=<installation-folder>
make install
```
Note that the installation process with deploy extra files that are needed by the Sol-R viewer. Typically, textures, environment maps and OpenCL kernels. Therefore, it is required to run the solrViewer application from the installation folder.

## Run
```
<installation-folder>/bin/solrViewer
```

### Selecting CUDA or OpenCL

By default, the OpenCL engine is selected but this can be changed by modifying the SOLR_ENGINE option, using either ccmake or the following cmake option:
```
cmake .. -DSOLR_ENGINE:STRING=CUDA
```

Optional dependencies can be activated using the following cmake options:
```
cmake .. -DSOLR_KINECT_ENABLED=ON -DSOLR_OCULUS_ENABLED=ON -DSOLR_SIXENSE_ENABLED -DSOLR_LEAPMOTION_ENABLED=ON
```

### Screenshots
![Sol-R_001](doc/images/Sol-R_001.png)
![Sol-R_002](doc/images/Sol-R_002.png)
![Sol-R_003](doc/images/Sol-R_003.png)
![Sol-R_004](doc/images/Sol-R_004.png)
![Sol-R_005](doc/images/Sol-R_005.png)
![Sol-R_006](doc/images/Sol-R_006.png)
![Sol-R_007](doc/images/Sol-R_007.png)
![Sol-R_008](doc/images/Sol-R_008.png)
![Sol-R_009](doc/images/Sol-R_009.png)
![Sol-R_009](doc/images/Sol-R_010.png)

### Supported platforms

Sol-R has currently been tested on:
- Windows 7 with Visual Studio 2015 Community edition
- Mac OS X Sierra 10.12.15
- Ubuntu 16.04
