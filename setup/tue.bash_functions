#!/bin/bash

TUE_DEV_DIR=~/ros/$TUE_ROS_DISTRO/dev
TUE_SYSTEM_DIR=~/ros/$TUE_ROS_DISTRO/system

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
#                                           TUE-CREATE
# ----------------------------------------------------------------------------------------------------

function tue-create
{
    if [ -z "$1" ]
    then
        echo "Usage: tue-create TYPE [ ARG1 ARG2 ... ]"
        return 1
    fi

    creation_type=$1

    # remove the first argument (which contained the creation type)
    shift

    if [ -f ~/.tue/create/$creation_type/create.bash ]
    then
        source ~/.tue/create/$creation_type/create.bash
    elif [ ~/.tue/create/$creation_type/create ]
    then
        ~/.tue/create/$creation_type/create $@
    else
        echo "tue-create: invalid creation type: '$creation_type'."
        return 1
    fi
}

function _tue-create
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "`_list_subdirs ~/.tue/create`" -- $cur) )
    fi
}
complete -F _tue-create tue-create

# ----------------------------------------------------------------------------------------------------
#                                            TUE-ADD
# ----------------------------------------------------------------------------------------------------

function tue-add
{
    if [ -z $1 ]
    then
        echo "Adds a given folder to the tue repository."
        echo ""
        echo "Usage: tue-add DIRECTORY"
        return 1
    fi

    if [ ! -d $1 ]
    then
        echo "'$1' is not a directory."
        return 1
    fi

    if [ -d $1/.svn ]
    then
        echo "'$1' is already under version control."
        return 1
    fi

    base=$(basename $1)  

    rm -rf /tmp/tue-svn
    upres=`svn co https://roboticssrv.wtb.tue.nl/svn/ros/trunk /tmp/tue-svn --depth immediates`

    if [ -d /tmp/tue-svn/$base ]
    then
        echo "'$base' already exists on the server."
        return 1
    fi

    mkdir -p /tmp/tue-svn/$base
    svn add /tmp/tue-svn/$base    

    svn ci /tmp/tue-svn/$base -m "tue-add: Added package '$base'"
    if [ $? -eq 0 ]
    then
        mv /tmp/tue-svn/$base/.svn $1
    else
        echo "Could not add '$base' to the tue repository."
        return 1
    fi
}

# ----------------------------------------------------------------------------------------------------
#                                            TUE-MAKE
# ----------------------------------------------------------------------------------------------------

function tue-make
{
    catkin_make --directory $TUE_SYSTEM_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
}

function tue-make-system
{
    catkin_make_isolated --directory $TUE_SYSTEM_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@	
}

function tue-make-dev
{
    catkin_make --directory $TUE_DEV_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
}

function tue-make-dev-isolated
{
    catkin_make_isolated --directory $TUE_DEV_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
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
    rm -rf $TUE_DEV_DIR/devel/etc
    rm -rf $TUE_DEV_DIR/devel/include
    rm -rf $TUE_DEV_DIR/devel/lib
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

function _tue-repo-status
{
    local name=$1
    local pkg_dir=$2

    if [ ! -d $pkg_dir ]
    then
        return
    fi

    local status=
    local vctype=

    if [ -d $pkg_dir/.svn ]
    then
        status=`svn status $pkg_dir`
        vctype=svn
    else
        # Try git

        cd $pkg_dir
        res=$(git status . --short --branch 2>&1)
        if [ $? -eq 0 ]
        then
            # Is git
            if echo "$res" | grep -q '\['   # Check if ahead of branch
            then
                status=$res
            else
                status=`git status . --short`
            fi

            local current_branch=`git rev-parse --abbrev-ref HEAD`
            if [ $current_branch != "master" ] && [ $current_branch != "hydro-devel" ]
            then
                echo -e "\033[1m$name\033[0m is on branch '$current_branch'"
            fi

        fi  

        cd - &> /dev/null
        vctype=git
    #else
    #    show=false
    fi

    if [ -n "$vctype" ]
    then
        if [ -n "$status" ]; then
            echo ""
            echo -e "\033[38;5;1mM  \033[0m($vctype) \033[1m$name\033[0m"
            echo "--------------------------------------------------"
            echo -e "$status"
            echo "--------------------------------------------------"
        fi 
    fi    
}

# ----------------------------------------------------------------------------------------------------

function tue-status
{
    fs=`ls $TUE_SYSTEM_DIR/src`
    for f in $fs
    do
        pkg_dir=$TUE_SYSTEM_DIR/src/$f
        _tue-repo-status $f $pkg_dir        
    done

    _tue-repo-status $TUE_DIR $TUE_DIR
}

# ----------------------------------------------------------------------------------------------------

function tue-git-status
{
    local output=""

    fs=`ls $TUE_SYSTEM_DIR/src`
    for pkg in $fs
    do
        pkg_dir=$TUE_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            branch=$(git rev-parse --abbrev-ref HEAD 2>&1)
            if [ $? -eq 0 ]
            then
                hash=$(git rev-parse --short HEAD)
                printf "\e[0;36m%-20s\033[0m %-15s %s\n" "$branch" "$hash" "$pkg"
            fi
        fi
    done
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-GET
# ----------------------------------------------------------------------------------------------------

function _tue_depends1
{
    local tue_dep_dir=$TUE_ENV_DIR/.env/dependencies

    if [ -z "$1" ]
    then
        echo "Usage: tue-depends PACKAGE"
        return 1
    fi

    if [ ! -f $tue_dep_dir/$1 ]
    then
        echo "Package '$1' not installed"
        return 1
    fi

    cat $tue_dep_dir/$1
}

function randid
{
    </dev/urandom tr -dc '0123456789abcdef' | head -c16; echo ""
}

function tue-get
{
    if [ -z "$1" ]
    then
        echo """tue-get is a tool for installing and removing packages that are under version control.

    Usage: tue-get COMMAND [ARG1 ARG2 ...]

    Possible commands:

        dep            - Shows target dependencies
	    install        - Installs a package
        update         - Updates currently installed packages
        remove         - Removes installed package
        list-installed - Lists all installed packages

"""
        return 1
    fi

    local tue_dep_dir=$TUE_ENV_DIR/.env/dependencies
    local tue_installed_dir=$TUE_ENV_DIR/.env/installed

    cmd=$1
    shift

    if [[ $cmd == "install" ]]
    then        
        if [ -z "$1" ]
        then
            echo "Usage: tue-get install TARGET [TARGET2 ...]"
            return 1
        fi

        ~/.tue/installer/scripts/tue-install $@
        error_code=$?

        if [ $error_code -eq 0 ]
        then
            # Mark targets as installed
            TUE_INSTALL_INSTALLED_DIR=$TUE_ENV_DIR/.env/installed
            mkdir -p $TUE_INSTALL_INSTALLED_DIR

            for target in $@
            do            
                touch $TUE_INSTALL_INSTALLED_DIR/$1
            done
        fi

        source ~/.bashrc
        return $error_code
    elif [[ $cmd == "update" ]]
    then
        error_code=0
        for target in $@
        do
            if [ ! -f $TUE_ENV_DIR/.env/dependencies/$target ]
            then
                echo "[tue-get] Package '$target' is not installed."
                error_code=1
            fi
        done

        if [ $error_code -eq 0 ]
        then
            ~/.tue/installer/scripts/tue-install $@
            error_code=$?
            source ~/.bashrc 
        fi
    
        return $error_code       
    elif [[ $cmd == "remove" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-get remove TARGET [TARGET2 ...]"
            return 1
        fi

        error=0
        for target in $@
        do
            if [ ! -f $tue_installed_dir/$target ]
            then
                echo "[tue-get] Package '$target' is not installed."
                error=1
            fi
        done        

        if [ $error -gt 0 ];
        then
            echo ""
            echo "[tue-get] No packages where removed."
            return $error;
        fi

        for target in $@
        do
            rm $tue_installed_dir/$target 
        done

        echo ""
        if [ -n "$2" ]; then
            echo "The packages were removed from the 'installed list' but still need to be deleted from your workspace."
        else
            echo "The package was removed from the 'installed list' but still needs to be deleted from your workspace."
        fi
    elif [[ $cmd == "list-installed" ]]
    then
        ls $tue_dep_dir
    elif [[ $cmd == "dep" ]]
    then
        ~/.tue/installer/scripts/tue-get-dep $@
    else
        echo "[tue-get] Unknown command: '$cmd'"
        return 1
    fi
}

function _tue-get
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "dep install update remove list-installed" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "install" ]]
        then
            COMPREPLY=( $(compgen -W "`ls ~/.tue/installer/targets`" -- $cur) )        
        elif [[ $cmd == "dep" ]]
        then
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_DIR/.env/dependencies`" -- $cur) ) 
        elif [[ $cmd == "update" ]]
        then
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_DIR/.env/dependencies`" -- $cur) ) 
        elif [[ $cmd == "remove" ]]
        then
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_DIR/.env/installed`" -- $cur) )  
        else
            COMREPLY=""
        fi
    fi
}
complete -F _tue-get tue-get

# ----------------------------------------------------------------------------------------------------
#                                             TUE-BRANCH
# ----------------------------------------------------------------------------------------------------

function tue-branch
{
    if [ -z "$1" ]
    then
        echo """Switches all packages to the given branch, if such a branch exists in that package. Usage:

    tue-branch BRANCH-NAME

"""
        return 1
    fi

    local branch=$1

    fs=`ls $TUE_SYSTEM_DIR/src`
    for pkg in $fs
    do
        pkg_dir=$TUE_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            local memd=$PWD
            cd $pkg_dir
            test_branch=$(git branch --list $branch 2>&1)
            if [ $? -eq 0 ] && [ "$test_branch" ]
            then
                local current_branch=`git rev-parse --abbrev-ref HEAD`
                if [[ "$current_branch" == "$branch" ]]
                then
                    echo -e "\033[1m$pkg\033[0m: Already on branch $branch"
                elsebash fu
                    res=$(git checkout $branch 2>&1)
                    if [ $? -eq 0 ]                
                    then
                        echo -e "\033[1m$pkg\033[0m: checked-out $branch"
                    else
                        echo ""
                        echo -e "    \033[1m$pkg\033[0m"
                        echo "--------------------------------------------------"
                        echo -e "\033[38;5;1m$res\033[0m"
                        echo "--------------------------------------------------"
                    fi
                fi
            fi
            cd $memd
        fi
    done
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-ENV
# ----------------------------------------------------------------------------------------------------

function tue-env
{
    if [ -z "$1" ]
    then
        echo """tue-env is a tool for switching between different installation environments.

    Usage: tue-env COMMAND [ARG1 ARG2 ...]

    Possible commands:

        switch         - Switch to a different environment
        list           - List all possible environments
"""
        return 1
    fi

    cmd=$1
    shift

    if [[ $cmd == "switch" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env switch ENVIRONMENT"
            return 1
        fi

        if [ ! -d ~/.tue/envs/$1 ]
        then
            echo "No such environment: '$1'."
            return 1
        fi

        rm -rf ~/.tue/env
        ln -s ~/.tue/envs/$1 ~/.tue/env        
    elif [[ $cmd == "list" ]]
    then
        for env in `ls ~/.tue/envs`
        do
            echo $env
        done
    elif [[ $cmd == "current" ]]
    then
        echo $(basename `readlink ~/.tue/env`)
    else
        echo "[tue-env] Unknown command: '$cmd'"
        return 1
    fi
}

function _tue-env
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "list switch current" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "switch" ]]
        then
            if [ $COMP_CWORD -eq 2 ]
                then
                COMPREPLY=( $(compgen -W "`ls ~/.tue/envs`" -- $cur) )        
            fi
        fi
    fi
}
complete -F _tue-env tue-env

