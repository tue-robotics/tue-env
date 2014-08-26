function tue-make
{
    cd ~/ros/$ROS_DISTRO/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=Release $@
    cd -
}

function tue-make-isolated
{
    cd ~/ros/$ROS_DISTRO/catkin_ws
    catkin_make_isolated -DCMAKE_BUILD_TYPE=Release $@
    cd -
}

function tue-install
{
    ~/.tue/installer/scripts/tue-install $@
}
