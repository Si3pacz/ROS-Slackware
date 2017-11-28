# Possible runtime issues:

## Gazebo:
The gazebo has the following optional dependencies that you likely
don't have installed. If you want them, you will need to rebuild gazebo and ROS:

### Available on SlackBuilds:

 - hdf5
 - gts (GNU Triangulated Surface Library) - enables CSG support
 - Bullet - enables Oculus Rift support
 - GDAL - enables support for Digital elevation terrains
 - graphviz - enables support for Model editor's schematic view and OpenSource Virtual Reality (OSVR)

### Not yet available on SlackBuilds:

 - Simbody - https://github.com/simbody/simbody
 - DART - http://dartsim.github.io/index.html
 - Player - http://playerstage.sourceforge.net/
 - ronn - https://github.com/rtomayko/ronn ; enables manpages


## PCL:
The gazebo has the following optional dependencies that you likely
don't have installed. If you want them, you will need to rebuild PCL and ROS:


### Available on SlackBuilds: 
 - qhull
 - cudatoolkit

### Not yet available on SlackBuilds:

 - OpenNI - enables OpenNI grabber support - https://github.com/OpenNI/OpenNI ; http://structure.io/openni

 - OpenNI2 - enables OpenNI2 grabber support - https://github.com/OpenNI/OpenNI2 ; http://structure.io/openni

 - FZAPI - enables fotonic camera support

 - ensenso - enables IDS-Imaging Ensenso camera support - http://www.ensenso.com/support/sdk-download/

 - davidSDK - enables David Vision Systems SDK support 
    - official http://www.david-3d.com/en/support/downloads
    - free fork https://github.com/InstitutMaupertuis/davidSDK

 - Depth Sense SDK (DSSDK) - enables DepthSense SDK support - https://github.com/huningxin/DSSDK

 - RealSense SDK (RSSDK) - enables RealSense SDK support - https://github.com/IntelRealSense/realsense_sdk








