if [ -z "$TUE_ROS_DISTRO" ]
then
    echo "[tue ros install] TUE_ROS_DISTRO was not set"
    return
fi

if [ ! -d /opt/ros/$TUE_ROS_DISTRO ]
then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

    sudo apt-get update

    # Install basic ROS packages. All other packages will be installed using tue-rosdep
    sudo apt-get install -y ros-$TUE_ROS_DISTRO-ros-base cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential python-catkin-tools

    sudo rosdep init || true # make sure it always succeeds, even if rosdep init was already called

    rosdep update
fi

source /opt/ros/$TUE_ROS_DISTRO/setup.bash

TUE_SYSTEM_DIR=$TUE_ENV_DIR/system
TUE_DEV_DIR=$TUE_ENV_DIR/dev

if [ ! -f $TUE_SYSTEM_DIR/devel/setup.bash ]
then
    mkdir -p $TUE_SYSTEM_DIR/src
    sudo apt-get install g++
    cd $TUE_SYSTEM_DIR
    catkin_make
    source $TUE_SYSTEM_DIR/devel/setup.bash
fi

if [ ! -f $TUE_DEV_DIR/devel/setup.bash ]
then
    mkdir -p $TUE_DEV_DIR/src
    sudo apt-get install g++
    cd $TUE_DEV_DIR
    catkin_make
    source $TUE_DEV_DIR/devel/setup.bash
fi
