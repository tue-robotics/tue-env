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

function randid
{
    </dev/urandom tr -dc '0123456789abcdef' | head -c16; echo ""
}

function tue-setup
{
    if [ -n "$2" ]
    then
        local tue_setup_dir=$2
    else
        local tue_setup_dir=/tmp/tue-setup-`randid`
        mkdir -p $tue_setup_dir
    fi

    if [ ! -f $tue_setup_dir/$1 ]
    then
        source ~/.tue/setup/$1
        touch $tue_setup_dir/$1
    fi
}
