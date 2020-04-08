# Zippies

Zippies are a project developed to create a tiny, open source robot which uses the HTC Vive Lighthouse for 2D position and orientation tracking. Picture below with nickel for scale.

![Fully Assembled Zippy](https://solinvictus21.github.io/FullyAssembledZippy.png)

This repository includes all the details you need to get a Zippy up and running, including the following...

- A complete set of [3D models](BodyModels) created in [SketchUp](https://www.sketchup.com/) that are required to build the outer body of a Zippy. Each of these parts is intended to be printed on a 3D printer and has been exported to a set of [STL files](BodyModels/STL) for convenience.
- A [schematic](LighthouseCircuit) and [complete bill of materials]() for a circuit designed to sense IR signals from a single HTC Vive Lighthouse to enable highly accurate, sub-millimeter localized positioning and orientation. The circuit is built in [AutoDesk Eagle](https://www.autodesk.com/products/eagle/free-download) and the PCB is available for immediate purchase directly [from Osh Park]().
- A complete [list of the additional hardware required]() to assemble a Zippy. 
- The [software](PlatformIO) required to allow your Zippy robot to follow any predefined path you wish to create. A sample path is included in the codebase for testing new Zippy builds.
- An [iPhone app](iOSClient) (work in progress) that will allow you to control the Zippy directly and aid in debugging over Bluetooth.
- A [path building app](Builder) (work in progress) using [Unity 3D]() designed to allow you to build and simulate routines for your Zippies to follow and, on supported platforms, allow you to upload them directly to the Zippy via Bluetooth LE for immediate testing.

See each of the links above for more details about each aspect of this project and feel free to contact me for any assistance you may need if you are interested in building your own Zippy.
