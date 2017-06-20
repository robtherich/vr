# A package for Max/MSP/Jitter to support virtual reality head-mounted displays

This Max package adds support for the Oculus Rift and the HTC Vive head-mounted displays (HMD) hardware. 

## Installing

Make sure you have the prerequisite hardware and drivers (see below).

Download the package and place into your Max Packages folder. (On Windows, My Documents/Max 7/Packages, on Mac, ~/Documents/Max 7/Packages), then restart Max.

Take a look at patchers in the `help` and `examples` folders to get to know the package.

If you have trouble loading externals you might need to install the [visual studio 2015 redistributable package](https://www.microsoft.com/en-us/download/details.aspx?id=48145). It is free.

## Supported systems

To use an HMD you will need the appropriate drivers installed, and a high end PC and graphics card. For these particular consumer head-mounted displays, it requires the equivalent of two HD or better resolution screens rendering at 90 frames per second. 

At present the recommended PC requirements to cover both the Oculus Rift and the HTC Vive are:

- **GPU:** NVIDIA GTX 970 / AMD R9 290 equivalent or greater
- **Video Output:** HDMI 1.4 or DisplayPort 1.2 or newer
- **CPU:** Intel i5-4590 / AMD FX 8350 equivalent or greater
- **Memory:** 8GB+ RAM
- **USB:** 3x USB 3.0 ports plus 1x USB 2.0 port
- **OS:** Windows 7 SP1 64 bit or newer (No OSX support at present)

**Oculus Rift**

The [Oculus Home](http://www.oculus.com/en-us/setup/) software should be installed & calibrated. It is free, but you need to create an Oculus account (free). Once installed, you may need to go into Oculus Home settings and enable the option to allow apps not downloaded from the Oculus Home store.

**HTC Vive** 

The [Steam and SteamVR](http://store.steampowered.com/steamvr) should be installed & calibrated. They are free, but you need to create a Steam account (also free).

## Development

See the ```/source``` folder. You may need to ```git submodules init && git submodules update``` to get all dependencies.


