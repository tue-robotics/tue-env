if dpkg -s libmedia1 | grep 'Architecture: amd64' &> /dev/null
then
    echo "Removing wrong version of libmedial"
    sudo apt remove libmedia1 -y
fi

deps=$(apt-cache depends ros-$ROS_DISTRO-audio-capture)
targets=
for dep in $deps
do
    if [ $dep != "Depends:" ] && [ $dep != "ros-$ROS_DISTRO-audio-capture" ] && [ $dep != "ros-$ROS_DISTRO-audio-common-msgs" ] && [ $dep != "ros-$ROS_DISTRO-roscpp" ]
    then
        #echo $dep
        targets="$targets $dep"
    fi
done
#echo $targets
tue-install-system $targets
