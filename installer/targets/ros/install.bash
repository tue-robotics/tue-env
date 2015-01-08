if [ -z "$TUE_ROS_DISTRO" ]
then
    echo "[tue ros install] TUE_ROS_DISTRO was not set"
    return
fi

if [ ! -d /opt/ros/$TUE_ROS_DISTRO ]
then
    sudo sh -c 'echo "deb http://ros.informatik.uni-freiburg.de/packages/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

    sudo apt-get update

    # Install basic ROS packages. All other packages will be installed using tue-rosdep
    sudo apt-get install -y ros-$TUE_ROS_DISTRO-ros-base cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential

    sudo rosdep init || true # make sure it always succeeds, even if rosdep init was already called

    rosdep update
fi

source /opt/ros/$TUE_ROS_DISTRO/setup.bash

TUE_SYSTEM_DIR=~/ros/$TUE_ROS_DISTRO/system
TUE_DEV_DIR=~/ros/$TUE_ROS_DISTRO/dev

if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    # create system catkin workspace
    if [ ! -d $TUE_SYSTEM_DIR/devel ]; then
        sudo apt-get install python-catkin-pkg -y
        mkdir -p $TUE_SYSTEM_DIR/src
        catkin_init_workspace $TUE_SYSTEM_DIR/src
        cd $TUE_SYSTEM_DIR && catkin_make
        source $TUE_SYSTEM_DIR/devel/setup.bash
    fi

    # create dev catkin workspace
    if [ ! -d $TUE_DEV_DIR/devel ]; then
        sudo apt-get install python-catkin-pkg -y
        mkdir -p $TUE_DEV_DIR/src
        catkin_init_workspace $TUE_DEV_DIR/src
        cd $TUE_DEV_DIR && catkin_make
        source $TUE_DEV_DIR/devel/setup.bash
    fi

    TUE_ROSBUILD_DIR=~/ros/$TUE_ROS_DISTRO/rosbuild

    # create rosbuild workspace
    if [ ! -f $TUE_ROSBUILD_DIR/.rosinstall ]; then
        # install rosws (part of rosinstall)
        sudo apt-get install python-rosinstall -y

        mkdir -p $TUE_ROSBUILD_DIR/trunk
        rosws init $TUE_ROSBUILD_DIR $TUE_SYSTEM_DIR/devel
        cd $TUE_ROSBUILD_DIR && rosws set trunk -y

        # make sure system and dev catkin workspaces are favored over the rosbuild_ws
        cd $TUE_ROSBUILD_DIR && rosws set $TUE_SYSTEM_DIR/src -y
        cd $TUE_ROSBUILD_DIR && rosws set $TUE_DEV_DIR/src -y
    fi

    # Check-out user folder
    if [ ! -d $TUE_ROSBUILD_DIR/user ]
    then
        svn co https://roboticssrv.wtb.tue.nl/svn/ros/user $TUE_ROSBUILD_DIR/user --depth immediates --trust-server-cert --non-interactive
    else
        svn up $TUE_ROSBUILD_DIR/user
    fi

    source $TUE_ROSBUILD_DIR/setup.bash

else
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
fi
