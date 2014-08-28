TUE_DEV_DIR=~/ros/hydro/dev
TUE_SYSTEM_DIR=~/ros/hydro/system

# ----------------------------------------------------------------------------------------------------
#                                        HELPER FUNCTIONS
# ----------------------------------------------------------------------------------------------------

function _list_subdirs
{
    fs=`ls $1`
    for f in $fs
    do
        if [ -d $1/$f ]
        then
            echo $f
        fi
    done
}

# ----------------------------------------------------------------------------------------------------
#                                            TUE-MAKE
# ----------------------------------------------------------------------------------------------------

function tue-make
{
    cd $TUE_SYSTEM_DIR
    catkin_make -DCMAKE_BUILD_TYPE=Release $@
    cd -
}

function tue-make-system
{
    cd $TUE_SYSTEM_DIR
    catkin_make_isolated -DCMAKE_BUILD_TYPE=Release $@
    cd -
}

function tue-make-dev
{
    cd $TUE_DEV_DIR
    catkin_make -DCMAKE_BUILD_TYPE=Release $@
    cd -
}

function tue-make-dev-isolated
{
    cd $TUE_DEV_DIR
    catkin_make_isolated -DCMAKE_BUILD_TYPE=Release $@
    cd -
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-DEV
# ----------------------------------------------------------------------------------------------------

function tue-dev
{
    if [ -z "$1" ]
    then
        _list_subdirs $TUE_DEV_DIR/src
        return 0
    fi

    for pkg in $@
    do     
        if [ ! -d $TUE_SYSTEM_DIR/src/$pkg ]
        then
            echo "[tue-dev] '$pkg' does not exist in the system workspace."
        elif [ -d $TUE_DEV_DIR/src/$pkg ]
        then
            echo "[tue-dev] '$pkg' is already in the dev workspace."
        else
            ln -s $TUE_SYSTEM_DIR/src/$pkg $TUE_DEV_DIR/src/$pkg
        fi
    done

    # Call rospack such that the linked directories are indexed
    local tmp=`rospack profile`
}

function tue-dev-clean
{
    for f in `_list_subdirs $TUE_DEV_DIR/src`
    do
        # Test if f is a symbolic link
        if [[ -L $TUE_DEV_DIR/src/$f ]]
        then
            echo "Cleaned '$f'"
            rm $TUE_DEV_DIR/src/$f
        fi
    done

    rm -rf $TUE_DEV_DIR/devel/share
    rm -rf $TUE_DEV_DIR/build
}

function _tue-dev
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_list_subdirs $TUE_SYSTEM_DIR/src`" -- $cur) )
}
complete -F _tue-dev tue-dev

# ----------------------------------------------------------------------------------------------------
#                                             TUE-STATUS
# ----------------------------------------------------------------------------------------------------

function tue-status
{
    fs=`ls $TUE_SYSTEM_DIR/src`
    for f in $fs
    do
        pkg_dir=$TUE_SYSTEM_DIR/src/$f

        status=
        vctype=

        if [ -d $pkg_dir/.svn ]
        then
            status=`svn status -q $pkg_dir`
            vctype=svn
        elif [ -d $pkg_dir/.git ]
        then
            cd $pkg_dir
            status=`git status --porcelain`
            cd - &> /dev/null
            vctype=git
        else
            show=false
        fi

        if [ -n "$vctype" ]
        then
            if [ -n "$status" ]; then
                echo ""
                #                echo -e "\033[1m$f (svn) \033[0m \033[38;5;1mMODIFIED\033[39m"
                echo -e "\033[38;5;1mM  \033[0m($vctype) \033[1m$f\033[0m"
                echo "--------------------------------------------------"
                echo -e "$status"
                echo "--------------------------------------------------"
                echo ""
            else
                echo -e "\033[38;5;2mOK\033[39m \033[0m($vctype) \033[1m$f\033[0m"
            fi 
        fi
   done
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
        if [ ! -d ~/.tue/installed ]
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
