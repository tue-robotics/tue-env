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

function _tue-make
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_list_subdirs $_TUE_CATKIN_SYSTEM_DIR/src`" -- $cur) )
}

complete -F _tue-make tue-make
complete -F _tue-make tue-make-system

function tue-make-dev
{
	case $(cat $_TUE_CATKIN_DEV_DIR/devel/.built_by) in
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

function _tue-make-dev
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_list_subdirs $_TUE_CATKIN_DEV_DIR/src`" -- $cur) )
}
complete -F _tue-make-dev tue-make-dev
complete -F _tue-make-dev tue-make-dev-isolated

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

function _robocup_branch_allowed
{
    local branch=$1
    if [ "$branch" == "robocup" ] && [ -f $TUE_DIR/user/config/robocup ]
    then
        return 0
    fi
    return 1
}

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
            if echo "$res" | grep -q -E 'behind|ahead' # Check if behind or ahead of branch
            then
                status=$res
            else
                status=`git status . --short`
            fi

            local current_branch=`git rev-parse --abbrev-ref HEAD`
            if [ $current_branch != "master" ]  && [ $current_branch != "develop" ] && [ $current_branch != "indigo-devel" ] && [ $current_branch != "kinetic-devel" ] && [ $current_branch != "toolchain-2.9" ] && ! _robocup_branch_allowed $current_branch
            then
                echo -e "\033[1m$name\033[0m is on branch '$current_branch'"
            fi

        fi

        cd - &> /dev/null
        vctype=git
    fi

    if [ -n "$vctype" ]
    then
        if [ -n "$status" ]; then
            echo -e ""
            echo -e "\033[38;5;1mM  \033[0m($vctype) \033[1m$name\033[0m"
            echo -e "--------------------------------------------------"
            echo -e "$status"
            echo -e "--------------------------------------------------"
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
    _tue-repo-status $TUE_DIR $TUE_DIR
}

# ----------------------------------------------------------------------------------------------------

function tue-git-status
{
    local mem_pwd=$PWD
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
    cd $mem_pwd
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-REVERT
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
#                                              TUE-REVERT-UNDO
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

function _tue_depends
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

    Possible options:
        --debug        - Shows more debugging information
        --branch=name  - Try to checkout this branch if exists

"""
        return 1
    fi

    local tue_dep_dir=$TUE_ENV_DIR/.env/dependencies
    local tue_installed_dir=$TUE_ENV_DIR/.env/installed

    local cmd=$1
    shift

    #Create btrfs snapshot if possible and usefull:
    if [[ "$cmd" =~ ^(install|update|remove)$ ]] && $(df --print-type / | grep -q btrfs)
    then
        sudo mkdir -p /snap/root
        sudo btrfs subvolume snapshot / /snap/root/$(date +%Y-%m-%d_%H:%M:%S)
    fi

    if [[ "$cmd" =~ ^(install|remove)$ && -z "$1" ]]
    then
       echo "Usage: tue-get $cmd TARGET [TARGET2 ...]"
       return 1
    fi

    if [[ $cmd == "install" ]]
    then
        $TUE_DIR/installer/scripts/tue-install $cmd $@
        error_code=$?

        [ $error_code -eq 0 ] && source ~/.bashrc

        return $error_code
    elif [[ $cmd == "update" ]]
    then
        error_code=0
        for target in $@
        do
            #Skip options
            [[ $target = '--'* ]] && continue

            if [ ! -f $TUE_ENV_DIR/.env/dependencies/$target ]
            then
                echo "[tue-get] Package '$target' is not installed."
                error_code=1
            fi
        done

        if [ $error_code -eq 0 ]
        then
            $TUE_DIR/installer/scripts/tue-install $cmd $@
            error_code=$?
            [ $error_code -eq 0 ] && source ~/.bashrc
        fi

        return $error_code
    elif [[ $cmd == "remove" ]]
    then
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
        local IFS=$'\n'
        options="'dep '\n'install '\n'update '\n'remove '\n'list-installed '"
        COMPREPLY=( $(compgen -W "$(echo -e "$options")" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "install" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_DIR/installer/targets | sed "s/.*/'& '/g")\n'--debug '\n'--branch='")" -- $cur) )
        elif [[ $cmd == "dep" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_ENV_DIR/.env/dependencies | sed "s/.*/'& '/g")\n'--plain '\n'--verbose '\n'--all '\n'--level='")" -- $cur) )
        elif [[ $cmd == "update" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_ENV_DIR/.env/dependencies | sed "s/.*/'& '/g")\n'--debug '\n'--branch='")" -- $cur) )
        elif [[ $cmd == "remove" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_DIR/.env/installed | sed "s/.*/'& '/g"`" -- $cur) )
        else
            COMREPLY=""
        fi
    fi
}
complete -o nospace -F _tue-get tue-get

# ----------------------------------------------------------------------------------------------------
#                                             TUE-CHECKOUT
# ----------------------------------------------------------------------------------------------------

function tue-checkout
{
    if [ -z "$1" ]
    then
        echo """Switches all packages to the given branch, if such a branch exists in that package. Usage:

    tue-checkout BRANCH-NAME [option]

    options:
    --only-pks: tue-env is not checkedout to the specified branch

"""
        return 1
    fi

    while test $# -gt 0
    do
        case "$1" in
            --only-pkgs) local NO_TUE_ENV="true"
            ;;
            --*) echo "unknown option $1"; exit 1;
            ;;
            *) local branch=$1
            ;;
        esac
        shift
    done

    fs=`ls -d -1 $_TUE_CATKIN_SYSTEM_DIR/src/**`
    if [ -z "$NO_TUE_ENV" ]
    then
        fs="$TUE_DIR $fs"
    fi
    for pkg_dir in $fs
    do
        pkg=${pkg_dir#$_TUE_CATKIN_SYSTEM_DIR/src/}
        if [ -z "$NO_TUE_ENV" ]
        then
            if [[ $pkg =~ ".tue" ]]
            then
                pkg="tue-env"
            fi
        fi

        if [ -d $pkg_dir ]
        then
            test_branch=$(git -C $pkg_dir branch -a 2> /dev/null | grep -q $branch)
            if [ $? -eq 0 ]
            then
                local current_branch=`git -C $pkg_dir rev-parse --abbrev-ref HEAD`
                if [[ "$current_branch" == "$branch" ]]
                then
                    echo -e "\033[1m$pkg\033[0m: Already on branch $branch"
                else
                    res=$(git -C $pkg_dir checkout $branch 2>&1)
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
        fi
    done
}

# ----------------------------------------------------------------------------------------------------

source $TUE_DIR/setup/tue-data.bash

# ----------------------------------------------------------------------------------------------------
#                                             TUE-ROBOCUP
# ----------------------------------------------------------------------------------------------------

function _tue-add-git-remote
{
    local remote=$1
    local server=$2

    if [ -z $2 ]
    then
        echo "Usage: _tue-add-git-remote REMOTE SERVER

For example:

    _tue-add-git-remote roboticssrv amigo@roboticssrv.local:
        "
        return 1
    fi

    if [ "$remote" == "origin" ]
    then
        echo "You are not allowed to change the remote: 'origin'"
        return 1
    fi

    local github_url="$(git config --get remote.origin.url)"
    local url_extension=${github_url#https://github.com/}
    local pkg=${url_extension#tue-robotics/}

    if [[ "$(git remote)" == *"$remote"* ]]
    then
        local current_url=$(git config --get remote.${remote}.url)
        if [[ "$current_url" == "$server$url_extension" ]]
        then
            echo -e "\033[1m[${pkg%.git}]\033[0m remote '$remote' exists with the same url"
            return 0
        fi

        git remote set-url $remote $server$url_extension
        echo -e "\033[1m[${pkg%.git}]\033[0m url of remote '$remote' is changed
    from: $current_url
    to: $server$url_extension"
    return 0
    fi
    git remote add $remote $server$url_extension

    echo -e "\033[1m[${pkg%.git}]\033[0m remote '$remote' added with url: $server$url_extension"
}

function tue-add-git-remote
{
    if [ -z $2 ]
    then
        echo "Usage: tue-add-git-remote REMOTE SERVER

For example:

    tue-add-git-remote origin https://github.com/
        "
        return 1
    fi

    local remote=$1
    local server=$2

    local mem_pwd=$PWD

    cd $TUE_DIR
    _tue-add-git-remote $remote $server

    local pkgs_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics

    local fs=`ls $pkgs_dir`
    for pkg in $fs
    do
        local pkg_dir=$pkgs_dir/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            _tue-add-git-remote $remote $server
        fi
    done

    cd $mem_pwd
}

function _tue-remote-checkout
{
    if [ -z $2 ]
    then
        echo "Usage: _tue-remote-checkout REMOTE BRANCH

For example:

    _tue-remote-checkout roboticssrv robocup
        "
        return 1
    fi

    local remote=$1
    local branch=$2

    local pkgs_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics

    local mem_pwd=$PWD

    cd $TUE_DIR
    echo -e "\033[1m[tue-env]\033[0m"
    git fetch $remote
    git checkout $branch

    local fs=`ls $pkgs_dir`
    for pkg in $fs
    do
        local pkg_dir=$pkgs_dir/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            echo -e "\033[1m[${pkg%.git}]\033[0m"
            git fetch $remote
            git checkout $branch
        fi
    done

    cd $mem_pwd
}

function _tue-robocup-default-branch
{
    local pkgs_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics

    local mem_pwd=$PWD

    cd $TUE_DIR
    echo -e "\033[1m[tue-env]\033[0m"
    default_branch=$(git symbolic-ref refs/remotes/origin/HEAD | sed 's@^refs/remotes/origin/@@')
    git checkout $default_branch

    local fs=`ls $pkgs_dir`
    for pkg in $fs
    do
        local pkg_dir=$pkgs_dir/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            echo -e "\033[1m[${pkg%.git}]\033[0m"
            default_branch=$(git symbolic-ref refs/remotes/origin/HEAD | sed 's@^refs/remotes/origin/@@')
            git checkout $default_branch
        fi
    done

    cd $mem_pwd
}

function tue-robocup-set-github
{
    _tue-robocup-default-branch
    # disallow robocup as branch in tue-status
    if [ -f $TUE_DIR/user/config/robocup ]
    then
        rm $TUE_DIR/user/config/robocup
    fi
}

function tue-robocup-set-roboticssrv
{
    tue-add-git-remote roboticssrv amigo@roboticssrv.local:
    _tue-remote-checkout roboticssrv robocup
    # allow robocup as branch in tue-status
    if [ ! -f $TUE_DIR/user/config/robocup ]
    then
        touch $TUE_DIR/user/config/robocup
    fi
}

function tue-robocup-set-timezone-robocup
{
    sudo timedatectl set-timezone America/Toronto
}

function tue-robocup-set-timezone-home
{
    sudo timedatectl set-timezone Europe/Amsterdam
}

function _ping_bool
{
    ping -c 1 "$1" 1>/dev/null 2>/dev/null
    if [ "$?" == "0" ]
    then
        return 0
    else
        return 1
    fi
}

function tue-robocup-install-package
{
    local repos_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics
    local repo_dir=$repos_dir/${1}.git

    local mem_pwd=$PWD

    local remote="roboticssrv"
    local server="amigo@roboticssrv.local:"
    local branch="robocup"

    # If directory already exists, return
    [ -d $repo_dir ] && return

    # If there is internet, use it for first clone. Otherwise an error
    # will be raised during switching back to default branch of origin
    local internet=$(echo _ping_bool github.com)
    if $internet
    then
        git clone https://github.com/tue-robotics/${1}.git $repo_dir
    else
        git clone amigo@roboticssrv.local:tue-robotics/${1}.git $repo_dir
    fi

    cd $repo_dir

    if $internet
    then
        _tue-add-git-remote $remote $server
        git fetch $remote
        git checkout $branch
    else
        git remote rename origin $remote
        git remote add origin https://github.com/tue-robotics/${1}.git
    fi

    cd $mem_pwd

    if [ -f "$repo_dir/package.xml" ]
    then
        if [ ! -h $TUE_ENV_DIR/system/src/$1 ]
        then
            ln -s $repo_dir $TUE_ENV_DIR/system/src/$1
        fi
    else
        # multiple packages in one repo
        local fs=$(ls -l | grep "^d" | awk '{print $9}')
        for pkg in $fs
        do
            local pkg_dir=$repo_dir/$pkg
            if [ -f "$pkg_dir/package.xml" ]
            then
                if [ ! -h $TUE_ENV_DIR/system/src/$pkg ]
                then
                    ln -s $pkg_dir $TUE_ENV_DIR/system/src/$pkg
                fi
            fi
        done
    fi

    # mark target as installed
    touch $TUE_ENV_DIR/.env/installed/ros-${1}
}

function tue-robocup-update
{
    local mem_pwd=$PWD

    cd $TUE_DIR
    git pull --ff-only

    # Copy rsettings file
    if [ "$ROBOT_REAL" != "true" ]
    then
        cp $TUE_DIR/installer/targets/tue-common/rsettings_file $TUE_DIR/.rsettings
    fi

    local fs=`ls $_TUE_CATKIN_SYSTEM_DIR/src`
    for pkg in $fs
    do
        local pkg_dir=$_TUE_CATKIN_SYSTEM_DIR/src/$pkg

        if [ -d $pkg_dir ]
        then
            cd $pkg_dir
            local current_url=`git config --get remote.origin.url`

            if echo "$current_url" | grep -q "roboticssrv.local"
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

