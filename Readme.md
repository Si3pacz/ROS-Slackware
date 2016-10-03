# Install ROS Kinetic on Slackware 14.2

This guide has been written and tested for Slackware 14.2 and a ROS Kinetic Desktop Install. If you 
have a different version of Slackware, steps might be slightly different.

Some other useful references are:
* Install ROS from source: http://wiki.ros.org/Installation/Source
* Another guide for installing on Slackware 14.2: http://barelywalking.com/?p=13
* A guide for installing on Slackware 14.1: http://barelywalking.com/?p=5

Note: ROS still primarily uses Python 2.7 and you should use that version of python for the installation.

##1. Prerequisites

###1.1 Install pip if you don't already have it
```
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
```

###1.2 Install ros tools

```
sudo pip install rosdep rosintall rosinall_generator wstool
```

###1.3 Upgrade rosdep
To current date, the latest release of rosdep does not include the definitions for Slackware.
The master branch of rosdep on github includes it. 
If you have a version of rosdep above `0.11.4` go to step 1.4. If you have version `0.11.4` or earlier
you need to update rosdep with the master branch on github:


```
git clone https://github.com/ros-infrastructure/rosdep.git
cd rosdep
sudo pip install -U .
```

###1.4 Initialize rosdep 
```
sudo rosdep init
rosdep update
```


##2. Installing dependencies

###2.1 Install sbotools
If you already have sbotools go to the next step

The Slackware installer that ROS uses is sbotools. This installer automates the process of installing
packages from SlackBuilds.org - a third party repository for Slackware Linux. If you don't wish to isntall
sbotools you will have to manually install all dependencies.

To install sbotools:

```
wget https://pink-mist.github.io/sbotools/downloads/sbotools-2.0.tar.gz
sudo installpkg sbotools-2.0.tar.gz
```

###2.2 Update your list of pacakges

Make sure to execute this step as some of the dependencies were submitted on SlackBuilds.org recently.
```
sudo sbocheck
```

###2.3 Install some of the dependencies

We are going to preinstall some of the ROS dependencies because they are more specific.

###2.3.1 Install VTK

VTK requires to be built with JAVA support which needs to be set manually.
First we need to install `jdk`. Installing it directly from sbotools fails because it cannot automatically
download the source from Oracle's website. Go to https://slackbuilds.org/repository/14.2/development/jdk/
and install jdk manually. Make sure to select the right architecture (32 or 64-bit).

After you are done, make sure to **reopen terminal** - we need to update environment variables that jdk set.
Otherwise VTK installation will fail. We are now ready to install VTK:

```
sudo sboinstall VTK
```

When asked whether you want to set options when building VTK, say yes and pass the option:

```
JAVA=yes
```

###2.3.2 Install log4cxx

log4cxx is not available on SlackBuilds.org so we need to install it manually. It is available as part of this
repository:

```
git clone https://github.com/nikonikolov/ROS-Slackware.git
cd ROS-Slackware/log4cxx
sudo ./log4cxx.SlackBuild
sudo installpkg log4cxx-0.10.0-x86_64-1root.txz
```

###2.3.3 Install urdfdom_headers

Although this package is available on SlackBuilds.org we still need to manually install it beforehand due 
to reasons I don't wanna go into:

```
sudo sboinstall urdfdom_headers
```


##3. Initialize rosdep
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

####Desktop-Full Install: 
*ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception* 

Note that if you choose this option you will need to install more dependencies on your own. 
The guide on this repo will not fully work.

```
rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-full-wet.rosinstall
wstool init -j8 src kinetic-desktop-full-wet.rosinstall
```


####Desktop Install (recommended): 
*ROS, rqt, rviz, and robot-generic libraries* 

Note that this guide has been written and tested for this kind of installation

```
rosinstall_generator desktop --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-wet.rosinstall
wstool init -j8 src kinetic-desktop-wet.rosinstall
```

####ROS-Comm: 
*(Bare Bones) ROS package, build, and communication libraries. No GUI tools.*

Although not tested for this kind of installation, this guide should work for it as all dependencies should be satisfied.

```
rosinstall_generator desktop --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-wet.rosinstall
wstool init -j8 src kinetic-desktop-wet.rosinstall
```

##4. Install remaining dependencies
We should now install the rest of the dependencies for Slackware. We can do that using rosdep which will 
automatically install the missing dependencies using sbotools and pip.

```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

Now wait for ages until it is done.
If you get an error while installing it is very likely to be because the pacakges take a lot of time 
to build and the cache has updated and no longer will execute sudo commands without a password. In this 
case just execute the above command again - the script will continue with the unsatisfied dependencies only.


If you don't want to use sbotools and pip you can execute

```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -s
```

This will output a list of commands that contains that rosdep would have otherwise executed. In that list you
can find the dependencies that you are missing. For me it looks like:

```
#[sbotools] Installation commands:
  sudo -H sboinstall -r protobuf
  sudo -H sboinstall -r collada-dom
  sudo -H sboinstall -r matplotlib
  sudo -H sboinstall -r hddtemp
  sudo -H sboinstall -r urdfdom
  sudo -H sboinstall -r urlgrabber
  sudo -H sboinstall -r sbcl
  sudo -H sboinstall -r nose
#[pip] Installation commands:
  sudo -H pip install -U empy
#[sbotools] Installation commands:
  sudo -H sboinstall -r PyYAML
  sudo -H sboinstall -r PyOpenGL
  sudo -H sboinstall -r paramiko
  sudo -H sboinstall -r lz4
  sudo -H sboinstall -r PyQt5
  sudo -H sboinstall -r urdfdom-headers
  sudo -H sboinstall -r assimp
  sudo -H sboinstall -r gtest
  sudo -H sboinstall -r netifaces
  sudo -H sboinstall -r coverage
  sudo -H sboinstall -r numpy
  sudo -H sboinstall -r mock
#[pip] Installation commands:
  sudo -H pip install -U defusedxml
#[sbotools] Installation commands:
  sudo -H sboinstall -r ffmpeg
  sudo -H sboinstall -r pydot
  sudo -H sboinstall -r qhull
  sudo -H sboinstall -r graphviz
  sudo -H sboinstall -r psutil
  sudo -H sboinstall -r yaml-cpp
  sudo -H sboinstall -r tinyxml
  sudo -H sboinstall -r poco
  sudo -H sboinstall -r console_bridge
  sudo -H sboinstall -r pygraphviz
  sudo -H sboinstall -r cppunit
  sudo -H sboinstall -r ogre
```

For you it might be a bit different.

NOTE: If you get an output similar to `No definition of [package] for OS version [Slackware_version]`, then 
the package is not actually defined in the rosdep definitions. You can submit an issue to this repo and I
will try to fix it ASAP or you can submit a pull request to https://github.com/ros/rosdistro with the
implemented fixes. If you are going for the Desktop-Full install you are very likely to get the above message.
It will tell you which dependencies you must satisfy on your own.


##5. Build the catking Workspace

Now that we have satisfied all dependencies we are almost ready to go. There are a few issues we need to fix
before building. 

*Sidenote: If for some reason building fails, try rebooting your system. I was getting strange errors as I had 
just installed all dependencies. Strangely, rebooting fixed them.*

###5.1 Set PYTHONPATH

```
echo $PYTHONPATH
```

If the above command does not output anything, place this at the end of your ~/.bashrc file:
```
export PYTHONPATH="~/ros_catkin_ws/src/catkin/python/catkin"
```

Otherwise place this line:
```
export PYTHONPATH="${PYTHONPATH}:~/ros_catkin_ws/src/catkin/python/catkin"
```

Make the changes take effect:

```
source ~/.bashrc
```

###5.2 Fix the version of qmake

Slackware uses qt4 by default. We need to make a fix in the `python_qt_bindings` package.
Open `~/ros_catkin_ws/src/python_qt_binding/cmake/sip_configure.py` with your favourite editor. Find the
lines:
```
env['QT_SELECT'] = '5'
qtconfig = subprocess.check_output(
    ['qmake', '-query'], env=env, universal_newlines=True)
```
Change `qmake` to `qmake-qt5` and save.

If you don't do this you will get an error while building:
```
/usr/share/sip/PyQt5/QtCore/qregularexpression.sip:26:32: fatal error: qregularexpression.h: No such file or directory
compilation terminated.
```


NOTE: Making the symlink in `/usr/bin/qmake` point to `qmake-qt5` for some reason did not fix the above error for me.


###5.3 Fix eigen3 linking problem in geomertic_shapes

Open `~/ros_catkin_ws/src/geometric_shapes/CMakeLists.txt` with your favourite editor.
Find the line 

```
find_package(Eigen3 REQUIRED)
```

and substitute it with

```
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )	
```
Save and exit. Not fixing the above caused an error:

```
CMake Error at CMakeLists.txt:24 (find_package):
  By not providing "FindEigen3.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "Eigen3", but
  CMake did not find one.

  Could not find a package configuration file provided by "Eigen3" with any
  of the following names:

    Eigen3Config.cmake
    eigen3-config.cmake

  Add the installation prefix of "Eigen3" to CMAKE_PREFIX_PATH or set
  "Eigen3_DIR" to a directory containing one of the above files.  If "Eigen3"
  provides a separate development package or SDK, be sure it has been
  installed.
```

The reason for that is that Slackware's eigen3 package does not come with a `FindEigen3.cmake`.

###5.4 Build the workspace
We are now ready to build our workspace

```
cd ~/ros_catkin_ws
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```

Wait for ages until it is done. Then
```
source ~/ros_catkin_ws/install_isolated/setup.bash
```


##6. Enjoy ROS




	sudo sboinstall ninja


	HAVE TO USE LATEST VERSION OF CATKIN 0.7.4 - see if that works if you fetch normally
	ninja might not be needed




. Desktop-Full Install: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

You will need to install more dependencies before you can install the Desktop-Full version of ROS. Most of these
packages are not readily available for Slackware and you will need to install them from source. I did not have
enough time to provide a SlackBuild for all of them, but if there are many requests, I will do it. Here is a list

### 1. PCL
	Point Cloud Library - http://pointclouds.org/

	Note you will also need a higher version of FLANN than the one available on SlackBuilds.org

### 2. FLTK
	Fulltick - http://www.fltk.org/index.php

### 3. Gazebo
	Gazebo - http://gazebosim.org/

	You will need a lot of dependencies which are to this date not readily available on SlackBuilds.org. These are:
	sdformat - http://sdformat.org/
	orge3d - 
	iginition-math2
