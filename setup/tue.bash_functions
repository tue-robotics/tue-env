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

_tue-install()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`ls ~/.tue/installer/targets`" -- $cur) )
}
complete -F _tue-install tue-install
