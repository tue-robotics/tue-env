#!/bin/bash

_TUE_CATKIN_DEV_DIR=$TUE_ENV_DIR/dev
_TUE_CATKIN_SYSTEM_DIR=$TUE_ENV_DIR/system

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

    if [ -f $TUE_DIR/create/$creation_type/create.bash ]
    then
        source $TUE_DIR/create/$creation_type/create.bash
    elif [ $TUE_DIR/create/$creation_type/create ]
    then
        $TUE_DIR/create/$creation_type/create $@
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
        COMPREPLY=( $(compgen -W "`_list_subdirs $TUE_DIR/create`" -- $cur) )
    fi
}
complete -F _tue-create tue-create

# ----------------------------------------------------------------------------------------------------
#                                            TUE-MAKE
# ----------------------------------------------------------------------------------------------------

function tue-make
{
    # compile non-ros packages if needed
    if [ -d $TUE_ENV_DIR/pkgs ]
    then
        $TUE_DIR/make/pre-configure.bash
        $TUE_DIR/make/configure.bash
        $TUE_DIR/make/make.bash
        $TUE_DIR/make/post-make.bash
    fi

    if [ -n "$TUE_ROS_DISTRO" ] && [ -d $_TUE_CATKIN_SYSTEM_DIR ]
    then
		case $(cat $_TUE_CATKIN_SYSTEM_DIR/devel/.built_by) in
		'catkin_make')
			catkin_make --directory $_TUE_CATKIN_SYSTEM_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
			;;
		'catkin build')
			catkin build --workspace $_TUE_CATKIN_SYSTEM_DIR $@
			;;
		'')
			catkin init --workspace $_TUE_CATKIN_SYSTEM_DIR $@
			catkin build --workspace $_TUE_CATKIN_SYSTEM_DIR $@
			;;
		esac
    fi
}

function tue-make-system
{
	case $(cat $_TUE_CATKIN_SYSTEM_DIR/devel/.built_by) in
	'catkin_make')
		catkin_make --directory $_TUE_CATKIN_SYSTEM_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
		;;
	'catkin build')
		catkin build --workspace $_TUE_CATKIN_SYSTEM_DIR $@
		;;
	'')
		catkin init --workspace $_TUE_CATKIN_SYSTEM_DIR $@
		catkin build --workspace $_TUE_CATKIN_SYSTEM_DIR $@
		;;
	esac
}

function tue-make-dev
{
	case $(cat $_TUE_CATKIN_SYSTEM_DIR/devel/.built_by) in
	'catkin_make')
		catkin_make --directory $_TUE_CATKIN_DEV_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
		;;
	'catkin build')
		catkin build --workspace $_TUE_CATKIN_DEV_DIR $@
		;;
	'')
		catkin init --workspace $_TUE_CATKIN_DEV_DIR $@
		catkin build --workspace $_TUE_CATKIN_DEV_DIR $@
		;;
	esac
}

function tue-make-dev-isolated
{
	case $(cat $_TUE_CATKIN_SYSTEM_DIR/devel/.built_by) in
	'catkin_make')
		catkin_make_isolated --directory $_TUE_CATKIN_DEV_DIR -DCMAKE_BUILD_TYPE=RelWithDebInfo $@
		;;
	'catkin build')
		catkin build --workspace $_TUE_CATKIN_DEV_DIR $@
		;;
	'')
		catkin init --workspace $_TUE_CATKIN_DEV_DIR $@
		catkin build --workspace $_TUE_CATKIN_DEV_DIR $@
		;;
	esac
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-DEV
# ----------------------------------------------------------------------------------------------------

function tue-dev
{
    if [ -z "$1" ]
    then
        _list_subdirs $_TUE_CATKIN_DEV_DIR/src
        return 0
    fi

    for pkg in $@
    do
        if [ ! -d $_TUE_CATKIN_SYSTEM_DIR/src/$pkg ]
        then
            echo "[tue-dev] '$pkg' does not exist in the system workspace."
        elif [ -d $_TUE_CATKIN_DEV_DIR/src/$pkg ]
        then
            echo "[tue-dev] '$pkg' is already in the dev workspace."
        else
            ln -s $_TUE_CATKIN_SYSTEM_DIR/src/$pkg $_TUE_CATKIN_DEV_DIR/src/$pkg
        fi
    done

    # Call rospack such that the linked directories are indexed
    local tmp=`rospack profile`
}

function tue-dev-clean
{
    for f in `_list_subdirs $_TUE_CATKIN_DEV_DIR/src`
    do
        # Test if f is a symbolic link
        if [[ -L $_TUE_CATKIN_DEV_DIR/src/$f ]]
        then
            echo "Cleaned '$f'"
            rm $_TUE_CATKIN_DEV_DIR/src/$f
        fi
    done

    rm -rf $_TUE_CATKIN_DEV_DIR/devel/share
    rm -rf $_TUE_CATKIN_DEV_DIR/devel/etc
    rm -rf $_TUE_CATKIN_DEV_DIR/devel/include
    rm -rf $_TUE_CATKIN_DEV_DIR/devel/lib
    rm -rf $_TUE_CATKIN_DEV_DIR/build
}

function _tue-dev
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_list_subdirs $_TUE_CATKIN_SYSTEM_DIR/src`" -- $cur) )
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
            if [ $current_branch != "master" ] && [ $current_branch != "develop" ] && [ $current_branch != "indigo-devel" ] && [ $current_branch != "kinetic-devel" ] && [ $current_branch != "toolchain-2.9" ]
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

function _tue-dir-status
{
    [ -d "$1" ] || return

    local fs=`ls $1`
    for f in $fs
    do
        pkg_dir=$1/$f
        _tue-repo-status $f $pkg_dir
    done
}

# ----------------------------------------------------------------------------------------------------

function tue-status
{
    _tue-dir-status $_TUE_CATKIN_SYSTEM_DIR/src
    _tue-dir-status $TUE_ENV_DIR/pkgs
    _tue-repo-status $TUE_DIR $TUE_DIR
}

# ----------------------------------------------------------------------------------------------------

function tue-git-status
{
    local output=""

    fs=`ls $_TUE_CATKIN_SYSTEM_DIR/src`
    for pkg in $fs
    do
        pkg_dir=$_TUE_CATKIN_SYSTEM_DIR/src/$pkg

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
#                                              NOBLEO-REVERT
# ----------------------------------------------------------------------------------------------------

function tue-revert
{
	human_time="$*"

    fs=`ls $_TUE_CATKIN_SYSTEM_DIR/src`
    for pkg in $fs
    do
        pkg_dir=$_TUE_CATKIN_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            branch=$(git rev-parse --abbrev-ref HEAD 2>&1)
            if [ $? -eq 0 ] && [ $branch != "HEAD" ]
            then
                new_hash=$(git rev-list -1 --before="$human_time" $branch)
                current_hash=$(git rev-parse HEAD)
                git diff -s --exit-code $new_hash $current_hash
                if [ $? -eq 0 ]
                then
                    newtime=$(git show -s --format=%ci)
                    printf "\e[0;36m%-20s\033[0m %-15s \e[1m%s\033[0m %s\n" "$branch is fine" "$new_hash" "$newtime" "$pkg"
                else
                    git checkout -q $new_hash
                    newbranch=$(git rev-parse --abbrev-ref HEAD 2>&1)
                    newtime=$(git show -s --format=%ci)
                    echo $branch > .do_not_commit_this
                    printf "\e[0;36m%-20s\033[0m %-15s \e[1m%s\033[0m %s\n" "$newbranch based on $branch" "$new_hash" "$newtime" "$pkg"
                fi
            else
                echo "Package $pkg could not be reverted, current state: $branch"
            fi
        fi
    done
}

# ----------------------------------------------------------------------------------------------------
#                                              NOBLEO-REVERT-UNDO
# ----------------------------------------------------------------------------------------------------

function tue-revert-undo
{
    fs=`ls $_TUE_CATKIN_SYSTEM_DIR/src`
    for pkg in $fs
    do
        pkg_dir=$_TUE_CATKIN_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            if [ -f .do_not_commit_this ]
            then
                echo $pkg
                git checkout `cat .do_not_commit_this`
                rm .do_not_commit_this
            fi
        fi
    done
    tue-git-status
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
        echo """tue-get is a tool for installing and removing packages.

    Usage: tue-get COMMAND [ARG1 ARG2 ...]

    Possible commands:

        dep            - Shows target dependencies
        install        - Installs a package
        update         - Updates currently installed packages
        remove         - Removes installed package
        list-installed - Lists all manually installed packages

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

        $TUE_DIR/installer/scripts/tue-install $@
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

        [ $error_code -eq 0 ] && source ~/.bashrc

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
            $TUE_DIR/installer/scripts/tue-install $@
            error_code=$?
            [ $error_code -eq 0 ] && source ~/.bashrc
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
        if [[ "$1" == "-a" ]]
        then
            ls $tue_dep_dir
        else
            ls $TUE_ENV_DIR/.env/installed
        fi
    elif [[ $cmd == "dep" ]]
    then
        $TUE_DIR/installer/scripts/tue-get-dep $@
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
            COMPREPLY=( $(compgen -W "`ls $TUE_DIR/installer/targets`" -- $cur) )
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
#                                             TUE-CHECKOUT
# ----------------------------------------------------------------------------------------------------

function tue-checkout
{
    if [ -z "$1" ]
    then
        echo """Switches all packages to the given branch, if such a branch exists in that package. Usage:

    tue-checkout BRANCH-NAME

"""
        return 1
    fi

    local branch=$1
    cd $TUE_DIR
    pkg=tue-env
    test_branch=$(git branch -a 2> /dev/null | grep -q $branch)
    if [ $? -eq 0 ]
    then
    	local current_branch=`git rev-parse --abbrev-ref HEAD`
        if [[ "$current_branch" == "$branch" ]]
        then
        	echo -e "\033[1m$pkg\033[0m: Already on branch $branch"
		else
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

    fs=`ls $_TUE_CATKIN_SYSTEM_DIR/src`
    for pkg in $fs
    do
        pkg_dir=$_TUE_CATKIN_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            local memd=$PWD
            cd $pkg_dir
            test_branch=$(git branch -a 2> /dev/null | grep -q $branch)
            if [ $? -eq 0 ]
            then
                local current_branch=`git rev-parse --abbrev-ref HEAD`
                if [[ "$current_branch" == "$branch" ]]
                then
                    echo -e "\033[1m$pkg\033[0m: Already on branch $branch"
                else
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

# ----------------------------------------------------------------------------------------------------

# Change directory to a package
function pcd
{
    if [ -z "$1" ]
    then
        cd $TUE_ENV_DIR/pkgs
        return
    fi

    if [ ! -d "$TUE_ENV_DIR/pkgs/$1" ]
    then
        echo "[pcd] No such package: '$1'"
        return 1
    fi

    cd $TUE_ENV_DIR/pkgs/$1
}

function _pcd
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]
    then
        local pkgs=
        [ -d $TUE_ENV_DIR/pkgs ] && pkgs=`_list_subdirs $TUE_ENV_DIR/pkgs`
        COMPREPLY=( $(compgen -W "$pkgs" -- $cur) )
    fi
}
complete -F _pcd pcd

# ----------------------------------------------------------------------------------------------------

source $TUE_DIR/setup/tue-data.bash

# ----------------------------------------------------------------------------------------------------

function tue-set-git-remote
{
    local remote=$1
    local server=$2

    if [ -z $2 ]
    then
        echo "Usage: tue-set-git-remote REMOTE SERVER

For example:

    tue-set-git-remote origin https://github.com
        "
        return 1
    fi

    local mem_pwd=$PWD

    cd ~/.tue
    git remote set-url $remote ${server}tue-robotics/tue-env

    pkgs_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics
    # replace spaces with underscores
    pkgs_dir=${pkgs_dir// /_}
    # now, clean out anything that's not alphanumeric or an underscore
    pkgs_dir=${pkgs_dir//[^a-zA-Z0-9\/\.-]/_}

    local fs=`ls $pkgs_dir`
    for pkg in $fs
    do
        local pkg_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            local current_url=`git config --get remote.origin.url`

            if echo "$current_url" | grep -q "tue-robotics"
            then
                git remote set-url origin ${server}tue-robotics/$pkg
                echo "Set origin url of '$pkg' to '${server}tue-robotics/$pkg'"
            fi
        fi
    done

    cd $mem_pwd
}

# Temporarily for RoboCup

function tue-robocup-set-github-origin
{
    tue-set-git-remote origin amigo@192.168.44.10:
}

function tue-robocup-reset-github-origin
{
    tue-set-git-remote origin https://github.com/
}

function tue-robocup-install-package
{
    local pkgs_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics
    # replace spaces with underscores
    pkgs_dir=${pkgs_dir// /_}
    # now, clean out anything that's not alphanumeric or an underscore
    pkgs_dir=${pkgs_dir//[^a-zA-Z0-9\/\.-]/_}

    local pkg_dir=$pkgs_dir/${1}.git


    # If directory already exists, return
    [ -d $pkg_dir ] && return

    git clone amigo@192.168.44.10:tue-robotics/${1}.git $pkg_dir

    ln -s $pkg_dir $TUE_ENV_DIR/system/src/$1
}

function tue-robocup-update
{
    local mem_pwd=$PWD

    cd ~/.tue
    git pull --ff-only

    # Copy rsettings file
    if [ "$ROBOT_REAL" != "true" ]
    then
        cp ~/.tue/installer/targets/tue-common/rsettings_file ~/.tue/.rsettings
    fi

    local fs=`ls $_TUE_CATKIN_SYSTEM_DIR/src`
    for pkg in $fs
    do
        local pkg_dir=$_TUE_CATKIN_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            local current_url=`git config --get remote.origin.url`

            if echo "$current_url" | grep -q "192.168.44.10"
            then
                echo -n "$pkg: "
                git pull --ff-only
            fi
        fi
    done

    if [ ! -d $TUE_ENV_DIR/system/src/robocup_knowledge ]; then
        ln -s $TUE_ENV_DIR/repos/https_/github.com/tue-robotics/tue_robocup.git/robocup_knowledge $TUE_ENV_DIR/system/src/robocup_knowledge
    fi

    cd $mem_pwd
}

function tue-robocup-set-apt-get-proxy
{
    sudo bash -c "echo 'Acquire::http::Proxy \"http://roboticssrv.wtb.tue.nl:3142\";' > /etc/apt/apt.conf.d/01proxy"
}

function tue-robocup-unset-apt-get-proxy
{
    sudo rm /etc/apt/apt.conf.d/01proxy
}

