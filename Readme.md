# Install ROS Kinetic on Slackware 14.2

Official Instructions for ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Slackware 

## Known Issues:

1. Compiling the OpenCV3 ROS package when CUDA is available runs into errors. 
A messy fix at the moment. Seems like an upstream problem

2. A clash between a library compiled by ROS which is also called `cv2.so` and
the system OpenCV library. Details here: https://answers.ros.org/question/272355/opencv-python-system-conflict/

3. PCL works only with QT4. See `perception_pcl.pathc` and PCL package on SlackBuilds.org.
