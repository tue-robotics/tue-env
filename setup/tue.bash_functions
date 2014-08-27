# ----------------------------------------------------------------------------------------------------
#                                            TUE-MAKE
# ----------------------------------------------------------------------------------------------------


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

# ----------------------------------------------------------------------------------------------------
#                                            TUE-INSTALL
# ----------------------------------------------------------------------------------------------------


function tue-install
{
    ~/.tue/installer/scripts/tue-install $@
    source ~/.bashrc
}

function _tue-install
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

# ----------------------------------------------------------------------------------------------------
#                                            TUE-SETUP
# ----------------------------------------------------------------------------------------------------

function tue-setup
{
    local first_time=

    if [ -z "$1" ]
    then
        if [ -d ~/.tue/installed ]
        then
            return
        fi

        TUE_SETUP_TARGETS=" "
        first_time=true

        # No argument given
        local installed_targets=`ls ~/.tue/installed`
        for t in $installed_targets
        do
            if [ -f ~/.tue/installer/targets/$t/setup ]
            then
                tue-setup $t
            fi
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


function _tue-setup-complete
{
    local targets=`ls ~/.tue/installer/targets`
    for t in $targets
    do
        if [ -f ~/.tue/installer/targets/$t/setup ]
        then
            echo $t
        fi
    done
}


function _tue-setup
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_tue-setup-complete`" -- $cur) )
}
complete -F _tue-setup tue-setup
