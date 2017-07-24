#!/bin/bash
set -ev

if dpkg-query -l opencv* libopencv* > /dev/null
then
	sudo apt remove opencv* libopencv* # Explicitly ask for confirmation
fi

if ! dpkg-query -l opencv* libopencv* > /dev/null
then
	sudo apt install --assume-yes ros-kinetic-opencv3
fi

sudo ln -fs /opt/ros/kinetic/lib/libopencv_core3.so /usr/lib/libopencv_core.so
sudo ln -fs /opt/ros/kinetic/lib/libopencv_highgui3.so /usr/lib/libopencv_highgui.so
sudo ln -fs /opt/ros/kinetic/lib/libopencv_imgcodecs3.so /usr/lib/libopencv_imgcodecs.so
sudo ln -fs /opt/ros/kinetic/lib/libopencv_imgproc3.so /usr/lib/libopencv_imgproc.so
sudo ln -fs /opt/ros/kinetic/lib/libopencv_videoio3.so /usr/lib/libopencv_videoio.so
sudo ln -fs /opt/ros/kinetic/include/opencv-3.2.0-dev/opencv2 /usr/include/opencv2

if cd ~/openpose
then
	git pull
else
	git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose ~/openpose
fi

# Modify scripts to use opencv3
sed -i.bak 's/# OPENCV_VERSION := 3/OPENCV_VERSION := 3/g' ~/openpose/ubuntu/Makefile.config.Ubuntu16_cuda8.example
sed -i.bak 's/# OPENCV_VERSION := 3/OPENCV_VERSION := 3/g' ~/openpose/3rdparty/caffe/Makefile.config.Ubuntu16_cuda8.example

# Do not update, takes long, not required
sed -i.bak 's/sudo apt-get --assume-yes update/# sudo apt-get --assume-yes update/g' ~/openpose/3rdparty/caffe/install_caffe_if_cuda8.sh

# sudo pip needs capital H
sed -i.bak 's/sudo -h pip install numpy protobuf/sudo -H pip install numpy protobuf/g' ~/openpose/3rdparty/caffe/install_caffe_if_cuda8.sh


set +ev

echo "

Great, now run these four commands yourself:
cd ~/openpose
bash ./ubuntu/install_caffe_and_openpose_if_cuda8.sh
roscd openpose_ros
ln -s ~/openpose

"
