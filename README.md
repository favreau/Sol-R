### Introduction
Sol-R is a CUDA/OpenCL-based realtime ray-tracer compatible with Oculus Rift DK1, Kinect, Razor Hydra and Leap Motion devices.
Sol-R was used to for the Interactive Molecular Visualiser project (http://www.molecular-visualization.com)

A number of videos can be found on my channel: https://www.youtube.com/user/CyrilleOnDrums

Sol-R was written as a hobby project in order to understand and learn more about CUDA and OpenCL. Most of the code was written overnight and at week-ends, meaning that it's probably not the best quality ever ;-)

The idea was to produce a Ray-Tracer that has its own "personality". Most of the code does not rely on any litterature about ray-tracing, but more on a naive approach of what rays could be used for. The idea was not to produce a physically based ray-tracer, but a simple engine that could produce cool images interactively.

Take it for what it is! Sol-R is a lot of fun to play with if you like coding computer generated images.

May the fun continue with your contributions! :-)

```
usage: solrViewer
```

### Build
```
mkdir Build
cd Build
cmake ..
```
