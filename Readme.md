# Install ROS Kinetic on Slackware 14.2

Official Instructions for ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Slackware 

## Missing package definitions
If during installation you get an error which looks like 
```
<ros-package-x>: No definition of [<package-x>] for OS [slackware]
```
then read on.

The package dependency list gets updated very often and it might not contain
all the definitions for Slackware at all times (if new packages have been added).
Sometimes I will fail to keep up and some packages might be missing.
Do not worry, there is an easy fix. You need to add a definition for each 
`<package-x>` in the above error message. That is, you need to add a mapping 
between the general name of `<package-x>` (i.e. the name with which ROS has 
it defined internally) and the name with which the package is defined in 
Slackware. This is needed in order for `rosdep` to be able to automatically
resolve the system dependencies that need to be satisfied in order for the ROS
installation to be successful.  

To add the definitions, follow these steps:

1. Fork https://github.com/ros/rosdistro
2. Clone it to your own machine
3. Find where the missing package is defined in ROS. Specifically, search for
  the name of `<package-x>` in one of the following files
- `rosdistro/rosdep/base.yaml`
- `rosdistro/rosdep/python.yaml`
4. Find the name of the package as Slackware knows it. Note that you must search
  the official Slackware mirrors or Slackbuilds.org to find the equivalent pacakge.
  If you cannot find the package in one of these lists, then it is probably a pure
  python package and you will need to search for it via `pip`. Note that sometimes,
  one ROS package might correspond to several Slackware packages.
  NOTE: ROS uses the Ubuntu package definitions. When searching have in mind that 
  on Slackware one package might correspond to several Ubuntu packages and thus 
  the names might not match exactly. The cases in which there will be no Slackware
  package will be very very unlikely.
  Note that if you add the wrong Slackware equivalent, then the ROS installation will
  later fail.
5. Add the package definition. There are three ways to do so, depending on whether the
  package is from the official Slackware mirror, Slackbuilds.org or pip:
  - Slackbuilds.org. The definition should look like
```
<package-x>: [<slackware-package-x->]
```

  - Official Slackware package. The definition should look like
```
<package-x>:
  slackware:
    slackpkg:
      packages: [<slackware-package-x->]
```

  - `pip` package. The definition should look like
```
<package-x>:
  slackware:
    pip:
      packages: [<slackware-package-x->]
```
Just try to follow the syntax in the file.

6. Commit your changes and push them to your fork on GitHub.
7. Redirect your sources list. You need to edit the file
`/etc/ros/rosdep/sources.list.d/20-default.list` and make the links
inside point to your fork on GitHub. For example if you modified
`rosdistro/rosdep/base.yaml`, you need to change
```
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
```
to
```
yaml https://raw.githubusercontent.com/<your-username>/rosdistro/master/rosdep/base.yaml
```
Save and exit
8. After you are done, run 
```
rosdep update
```
This step will make your installation of `rosdep` fetch the sources as defined
in your fork. Note that you might need to clear the `rosdep`'s cache for the
update to take place. To clear the cache, run
```
rm -rf ~/.ros/rosdep/*
```

9. If everything is successful, you can submit a pull request to
https://github.com/ros/rosdistro so that others can benefit.

NOTE: You might also find https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md
to be helpful.

## Known Issues:

1. Compiling the OpenCV3 ROS package when CUDA is available runs into errors. 
A messy fix at the moment. Seems like an upstream problem

2. A clash between a library compiled by ROS which is also called `cv2.so` and
the system OpenCV library. Details here: https://answers.ros.org/question/272355/opencv-python-system-conflict/

3. PCL works only with QT4. See `perception_pcl.pathc` and PCL package on SlackBuilds.org.
