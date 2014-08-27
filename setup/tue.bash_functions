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
    local first_time=

    if [ -z "$1" ]
    then
        TUE_SETUP_TARGETS=" "
        first_time=true

        # No argument given
        local installed_targets=`ls ~/.tue/installed`
        for t in $installed_targets
        do
            tue-setup $t
        done
    else    
        if [ -z "$TUE_SETUP_TARGETS" ]
        then
            TUE_SETUP_TARGETS=" "
            first_time=true
        fi

       if [[ "$TUE_SETUP_TARGETS" != *" $1 "* ]];
        then
            local tue_setup_file=~/.tue/installer/targets/$1/setup
            if [ -f $tue_setup_file ]
            then
                source $tue_setup_file
                TUE_SETUP_TARGETS=" $1$TUE_SETUP_TARGETS"
            else
                echo "[tue-setup] WARNING: Target '$1' does not have a setup."
            fi
        fi

    fi
 
    if [ -n "$first_time" ]
    then
        TUE_SETUP_TARGETS=
    fi
}
