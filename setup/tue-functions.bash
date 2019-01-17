#! /usr/bin/env bash

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
#                                             TUE-ROBOCUP
# ----------------------------------------------------------------------------------------------------

export TUE_ROBOCUP_BRANCH="robocup"

function _tue-repos-do
{
    # Evaluates the command of the input for tue-env and all repo's of tue-robotics.
    # The input can be multiple arguments, but if the input consists of multiple commands
    # seperated by ';' or '&&' the input needs to be captured in a string.

    local mem_pwd=$PWD

    cd $TUE_DIR
    echo -e "\033[1m[tue-env]\033[0m"
    eval "$@"

    local repos_dir=$TUE_ENV_DIR/repos/https_/github.com/tue-robotics

    local fs=`ls $repos_dir`
    for repo in $fs
    do
        local repo_dir=$repos_dir/$repo

        if [ -d $repo_dir ]
        then
            cd $repo_dir
            echo -e "\033[1m[${repo%.git}]\033[0m"
            eval "$@"
        fi
    done

    cd $mem_pwd
}

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
        echo -e "\033[1mYou are not allowed to change the remote: 'origin'\033[0m"
        return 1
    fi

    local github_url="$(git config --get remote.origin.url)"
    local url_extension=${github_url#https://github.com/}

    if [[ "$(git remote)" == *"$remote"* ]]
    then
        local current_url=$(git config --get remote.${remote}.url)
        if [[ "$current_url" == "$server$url_extension" ]]
        then
            echo -e "remote '$remote' exists with the same url"
            return 0
        fi

        git remote set-url $remote $server$url_extension
        echo -e "url of remote '$remote' is changed
    from: $current_url
    to: $server$url_extension"
    return 0
    fi
    git remote add $remote $server$url_extension

    echo -e "remote '$remote' added with url: $server$url_extension"
}

function tue-add-git-remote
{
    if [ -z $2 ]
    then
        echo "Usage: tue-add-git-remote REMOTE SERVER

For example:

    tue-add-git-remote roboticssrv amigo@roboticssrv.local:
        "
        return 1
    fi

    local remote=$1
    local server=$2

    if [ "$remote" == "origin" ]
    then
        echo -e "\033[1mYou are not allowed to change the remote: 'origin'\033[0m"
        return 1
    fi

    _tue-repos-do "_tue-add-git-remote $remote $server"
}

function __tue-remove-git-remote
{
    local remote=$1

    if [ -z $1 ]
    then
        echo "Usage: __tue-remove-git-remote REMOTE

For example:

    __tue-remove-git-remote roboticssrv
        "
        return 1
    fi

    if [ "$remote" == "origin" ]
    then
        echo -e "\033[1mYou are not allowed to remove the remote: 'origin'\033[0m"
        return 1
    fi

    if [[ "$(git remote)" == *"$remote"* ]]
    then
        git remote remove $remote
        echo -e "remote '$remote' is removed"
        return 0
    fi

    echo -e "remote '$remote' doesn't exist"
}

function _tue-remove-git-remote
{
    if [ -z $1 ]
    then
        echo "Usage: _tue-remove-git-remote REMOTE

For example:

    _tue-remove-git-remote roboticssrv
        "
        return 1
    fi

    local remote=$1

    if [ "$remote" == "origin" ]
    then
        echo -e "\033[1mYou are not allowed to remove the remote: 'origin'\033[0m"
        return 1
    fi

    _tue-repos-do "__tue-remove-git-remote $remote"
}

function _git_remote_checkout
{
    if [ -z $2 ]
    then
        echo "Usage: _git_remote_checkout REMOTE BRANCH

For example:

    _git_remote_checkout roboticssrv robocup
        "
        return 1
    fi

    local remote=$1
    local branch=$2
    local exists=$(git show-ref refs/heads/$branch)
    if [ -n "$exists" ]
    then
        git checkout $branch
        git branch -u $remote/$branch $branch
    else
        git checkout --track -b $branch $remote/$branch
    fi
}

function tue-remote-checkout
{
    if [ -z $2 ]
    then
        echo "Usage: tue-remote-checkout REMOTE BRANCH

For example:

    tue-remote-checkout roboticssrv robocup
        "
        return 1
    fi

    local remote=$1
    local branch=$2

    _tue-repos-do "git fetch $remote; _git_remote_checkout $remote $branch"
}

function _tue-robocup-remote-checkout
{
    if [ -z $2 ]
    then
        echo "Usage: _tue-robocup-remote-checkout REMOTE BRANCH

For example:

    _tue-robocup-remote-checkout roboticssrv robocup
        "
        return 1
    fi

    local remote=$1
    local branch=$2

    git fetch $remote
    local current_remote=$(git for-each-ref --format='%(upstream:short)' $(git symbolic-ref -q HEAD) | awk -F/ '{print $1}')
    if [ "$current_remote" != "$remote" ]
    then
        _git_remote_checkout $remote $branch
    fi
}

function tue-robocup-remote-checkout
{
    # same functionality as tue-remote-checkout, but no arguments needed
    # doesn't perform a checkout, when current branch is already setup
    # to the roboticssrv
    local remote="roboticssrv"
    local branch=$TUE_ROBOCUP_BRANCH

    _tue-repos-do "_tue-robocup-remote-checkout $remote $branch"
}

function __tue-robocup-default-branch
{
    local default_branch=$(git remote show origin | grep HEAD | awk '{print $3}')
    _git_remote_checkout origin $default_branch
}

function _tue-robocup-default-branch
{
    _tue-repos-do "__tue-robocup-default-branch"
}

function _tue-robocup-change-remote
{
    if [ -z $2 ]
    then
        echo "Usage: _tue-robocup-change-remote BRANCH REMOTE

For example:

    _tue-robocup-change-remote robocup origin
        "
        return 1
    fi

    local branch=$1
    local remote=$2

    if [ -n "$(git show-ref refs/heads/$branch)" ]
    then
        if [[ "$(git remote)" == *"$remote"* ]]
        then
            git fetch  $remote
            if [[ "$(git branch -a)" == *"${remote}/${branch}"* ]]
            then
                git branch -u $remote/$branch $branch
            else
                echo -e "no branch: $branch on remote: $remote"
            fi
        else
            echo -e "no remote: $remote"
        fi
    else
        echo -e "no local branch: $branch"
    fi
}

function tue-robocup-change-remote
{
    # This changes the remote of the 'BRANCH' branch to 'REMOTE'
    # After this, you local working copies may be behind what was fetched from REMOTE, so run a $ tue-get update

    # for packages that have a REMOTE as a remote:
        # do a git fetch origin: git fetch
        # Change remote of branch 'BRANCH' to REMOTE: git branch -u REMOTE/BRANCH BRANCH

    if [ -z $2 ]
    then
        echo "Usage: tue-robocup-change-remote BRANCH REMOTE

For example:

    tue-robocup-change-remote robocup origin
        "
        return 1
    fi

    local branch=$1
    local remote=$2

    _tue-repos-do "_tue-robocup-change-remote $branch $remote"
}

function tue-robocup-ssh-copy-id
{
    ssh-copy-id amigo@roboticssrv.local
}

function tue-robocup-set-github
{
    tue-robocup-change-remote $TUE_ROBOCUP_BRANCH origin
    _tue-robocup-default-branch
    # disallow TUE_ROBOCUP_BRANCH as branch in tue-status
    if [ -f $TUE_DIR/user/config/robocup ]
    then
        rm $TUE_DIR/user/config/robocup
    fi
}

function tue-robocup-set-roboticssrv
{
    tue-add-git-remote roboticssrv amigo@roboticssrv.local:
    tue-robocup-remote-checkout
    # allow TUE_ROBOCUP_BRANCH as branch in tue-status
    if [ ! -f $TUE_DIR/user/config/robocup ]
    then
        echo $TUE_ROBOCUP_BRANCH > $TUE_DIR/user/config/robocup
    fi
}

function tue-robocup-set-timezone-robocup
{
    sudo timedatectl set-timezone Australia/Sydney
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
    local branch=$TUE_ROBOCUP_BRANCH

    # If directory already exists, return
    [ -d $repo_dir ] && return

    git clone ${server}tue-robotics/${1}.git $repo_dir

    cd $repo_dir

    git remote rename origin $remote
    git remote add origin https://github.com/tue-robotics/${1}.git

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
    _tue-repos-do "git pull --ff-only"

    # Copy rsettings file
    if [ "$ROBOT_REAL" != "true" ]
    then
        rsettings_file=$TUE_ENV_TARGETS_DIR/tue-common/rsettings_file
        if [ -f $rsettings_file]
        then
            cp $rsettings_file $TUE_DIR/.rsettings
        fi
    fi
}

function tue-robocup-set-apt-get-proxy
{
    sudo bash -c "echo 'Acquire::http::Proxy \"http://roboticssrv.wtb.tue.nl:3142\";' > /etc/apt/apt.conf.d/01proxy"
}

function tue-robocup-unset-apt-get-proxy
{
    sudo rm /etc/apt/apt.conf.d/01proxy
}

