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
sudo pip install -U rosdep rosinstall rosinstall_generator wstool
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

###2.2 Update your list of packages

Make sure to execute this step as some of the dependencies were submitted on SlackBuilds.org very recently.
```
sudo sbocheck
```

###2.3 Install some of the dependencies

We are going to preinstall some of the ROS dependencies because they are more special.

####2.3.1 Install VTK

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

####2.3.2 Install log4cxx

log4cxx is not available on SlackBuilds.org so we need to install it manually. It is available as part of this
repository:

```
git clone https://github.com/nikonikolov/ROS-Slackware.git
cd ROS-Slackware/log4cxx
sudo ./log4cxx.SlackBuild
sudo installpkg log4cxx-0.10.0-x86_64-1root.txz
```

####2.3.3 Install urdfdom

Although this package is available on SlackBuilds.org we still need to manually beforehand due to a strange way that sbotools handles dependencies:

```
sudo sboinstall urdfdom_headers
sudo sboinstall urdfdom
```


##3. Initialize rosdep
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

####Desktop-Full Install: 
*ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception* 

Note that if you choose this option **you will need to install more dependencies on your own**. 
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

If you don't want to use sbotools and pip to install dependencies, instead of the above you can execute

```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -s
```

This will output a list of commands that contains that rosdep would have otherwise executed. In that list you
can find the dependencies that you are missing. It will look like:

```
#[sbotools] Installation commands:
  sudo -H sboinstall -r protobuf
  sudo -H sboinstall -r sbcl
#[pip] Installation commands:
  sudo -H pip install -U empy
  ...
```

##5. Build the catkin Workspace

Now that we have satisfied all dependencies we are almost ready to go. There are a few issues we need to fix
before building. 


###5.1 Set PYTHONPATH


Execute these commands in your current terminal:

```
PYTHONPATH=$PYTHONPATH:~/ros_catkin_ws/src/catkin/python/catkin
PYTHONPATH=$PYTHONPATH:~/ros_catkin_ws/install_isolated/lib64/python2.7/site-packages
PYTHONPATH=$PYTHONPATH:~/ros_catkin_ws/install_isolated/lib/python2.7/site-packages
export PYTHONPATH
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


###5.3 Fix eigen3 linking problem

Open `~/ros_catkin_ws/src/geometric_shapes/CMakeLists.txt` and `~/ros_catkin_ws/src/eigen_stl_containers/CMakeLists.txt` 
with your favourite editor.
In both files find the line 

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

You can place this command in your `~/.bashrc` to make the changes permanent

###5.5 Fix a possible build bug

In my case there were packages with the same name but different modules that built in 
`install_isolated/lib64/python2.7/site-packages` and `install_isolated/lib/python2.7/site-packages`.
This is not correct behavior and is a possible ROS bug that is being fixed. For now we need to fix 
this manually as it will cause trouble in the future when we use ROS. 

Clone this repo if you have not done it already:
```
git clone https://github.com/nikonikolov/ROS-Slackware.git
```

and execute a script which will move all the modules under `install_isolated/lib64/python2.7/site-packages`

```
cd ROS-Slackware && ./fixmodules.sh
```

##6. Enjoy ROS



#Possible Errors

###Error installing dependencies via rosdep

If you get an error while installing Slackware using rosdep it is very likely because the packages take a lot of time 
to build. Meanwhile the cache has updated and no longer will execute sudo commands without a password. In this 
case just execute again

```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

If that also fails, try to manually install the corresponding package that fails using 

```
sudo sboinstall <package>
```

The reason is that `rosdep` does not seem to pass a `y` flag for all dependencies that a Slackware package needs


###No definition of [package] for OS version [Slackware_version] 

If you get an output similar to `No definition of [package] for OS version [Slackware_version]`, then 
the package is not actually defined in the rosdep definitions. You can submit an issue to my repo 
https://github.com/nikonikolov/ROS-Slackwarethis repo and I will try to fix it ASAP or you can manually handle
it following the instructions on http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html
If you are going for the Desktop-Full install you are very likely to get the above message.

