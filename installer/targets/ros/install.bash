if [ -z "$TUE_ROS_DISTRO" ]
then
    TUE_ROS_DISTRO=hydro
fi

if [ ! -d /opt/ros/$TUE_ROS_DISTRO ]
then
    sudo sh -c 'echo "deb http://ros.informatik.uni-freiburg.de/packages/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

    sudo apt-get update

    # Install basic ROS packages. All other packages will be installed using tue-rosdep
    sudo apt-get install -y ros-$TUE_ROS_DISTRO-ros-base cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential

    sudo rosdep init || true # make sure it always succeeds, even if rosdep init was already called

    rosdep update
fi

source /opt/ros/$TUE_ROS_DISTRO/setup.bash

TUE_SYSTEM_DIR=~/ros/$TUE_ROS_DISTRO/system

if [ ! -f $TUE_SYSTEM_DIR/devel/setup.bash ]
then
    mkdir -p $TUE_SYSTEM_DIR/src
    sudo apt-get install g++
    cd $TUE_SYSTEM_DIR
    catkin_make
    source $TUE_SYSTEM_DIR/devel/setup.bash
fi

TUE_DEV_DIR=~/ros/$TUE_ROS_DISTRO/dev

if [ ! -f $TUE_DEV_DIR/devel/setup.bash ]
then
    mkdir -p $TUE_DEV_DIR/src
    sudo apt-get install g++
    cd $TUE_DEV_DIR
    catkin_make
    source $TUE_DEV_DIR/devel/setup.bash
fi

