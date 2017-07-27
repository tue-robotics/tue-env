#!/bin/bash
set -e

echo_and_run() { echo "$@" ; "$@" ; }

if dpkg-query -l opencv* libopencv* > /dev/null 2>&1
then
	echo_and_run sudo apt remove opencv* libopencv* # Explicitly ask for confirmation
fi

if ! dpkg-query -l ros-kinetic-opencv3 > /dev/null
then
	echo_and_run sudo apt install --assume-yes ros-kinetic-opencv3
fi

if ! dpkg-query -l libopenblas-dev > /dev/null
then
    echo_and_run sudo apt install --assume-yes libopenblas-dev
fi

symlink() { if [ ! -h "$2" ] ; then sudo ln -fsv "$@"; fi; }

symlink /opt/ros/kinetic/lib/libopencv_core3.so /usr/lib/libopencv_core.so
symlink /opt/ros/kinetic/lib/libopencv_highgui3.so /usr/lib/libopencv_highgui.so
symlink /opt/ros/kinetic/lib/libopencv_imgcodecs3.so /usr/lib/libopencv_imgcodecs.so
symlink /opt/ros/kinetic/lib/libopencv_imgproc3.so /usr/lib/libopencv_imgproc.so
symlink /opt/ros/kinetic/lib/libopencv_videoio3.so /usr/lib/libopencv_videoio.so
symlink /opt/ros/kinetic/include/opencv-3.2.0-dev/opencv2 /usr/include/opencv2

if cd ~/openpose
then
	git pull
else
	git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose ~/openpose
fi

# Modify scripts to use opencv3
sed -i 's/# OPENCV_VERSION := 3/OPENCV_VERSION := 3/g' ~/openpose/ubuntu/Makefile.config.Ubuntu16_cuda8.example
sed -i 's/# OPENCV_VERSION := 3/OPENCV_VERSION := 3/g' ~/openpose/3rdparty/caffe/Makefile.config.Ubuntu16_cuda8.example

# Switch architecture if desired
if [ ! -d /usr/lib/x86_64-linux-gnu ]
then
    sed -i 's/x86_64-linux-gnu/aarch64-linux-gnu/g' ~/openpose/ubuntu/Makefile.config.Ubuntu16_cuda8.example
fi

# Do not update, takes long, not required
sed -i 's/sudo apt-get --assume-yes update/# sudo apt-get --assume-yes update/g' ~/openpose/3rdparty/caffe/install_caffe_if_cuda8.sh

# sudo pip needs capital H
sed -i 's/sudo -h pip install numpy protobuf/sudo -H pip install numpy protobuf/g' ~/openpose/3rdparty/caffe/install_caffe_if_cuda8.sh

# copy Makefile
cp ~/openpose/3rdparty/caffe/Makefile.config.Ubuntu16_cuda8.example ~/openpose/3rdparty/caffe/Makefile.config
cp ~/openpose/ubuntu/Makefile.config.Ubuntu16_cuda8.example ~/openpose/Makefile.config

if [ ! -d ~/openpose/build ]
then
	echo "

	Great, now run these four commands yourself:
	cd ~/openpose
	bash ./ubuntu/install_caffe_and_openpose_if_cuda8.sh
	roscd openpose_ros
	ln -s ~/openpose

	"
fi
