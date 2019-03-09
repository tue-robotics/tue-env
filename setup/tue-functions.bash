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
#                                              SSH
# ----------------------------------------------------------------------------------------------------

function _github_https
{
    local input_url=$1
    echo ${input_url/git@github.com:/https:\/\/github.com\/}
}
export -f _github_https # otherwise not available in sourced files

function _github_ssh
{
    local input_url=$1
    echo ${input_url/https:\/\/github.com\//git@github.com:}
}
export -f _github_ssh # otherwise not available in sourced files

function _github_https_or_ssh
{
    local input_url=$1
    if [[ "$TUE_USE_SSH" == "true" ]]
    then
        local output_url=$(_github_ssh $input_url)
    else
        local output_url=$(_github_https $input_url)
    fi
    echo "$output_url"
}
export -f _github_https_or_ssh # otherwise not available in sourced files

# ----------------------------------------------------------------------------------------------------
#                                            TUE-MAKE
# ----------------------------------------------------------------------------------------------------

function tue-make
{
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
    if [ -f $TUE_DIR/user/config/robocup ] && [ "$branch" == "$(cat $TUE_DIR/user/config/robocup)" ]
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

        res=$(git -C $pkg_dir status . --short --branch 2>&1)
        if [ $? -eq 0 ]
        then
            # Is git
            if echo "$res" | grep -q -E 'behind|ahead' # Check if behind or ahead of branch
            then
                status=$res
            else
                status=$(git -C $pkg_dir status . --short)
            fi

            local current_branch=$(git -C $pkg_dir rev-parse --abbrev-ref HEAD)
            case $current_branch in
                master|develop|indigo-devel|hydro-devel|jade-devel|kinetic-devel|toolchain-2.9) ;;
                *) _robocup_branch_allowed $current_branch || echo -e "\033[1m$name\033[0m is on branch '$current_branch'";;
            esac
        fi

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
        _tue-repo-status "$f" "$pkg_dir"
    done
}

# ----------------------------------------------------------------------------------------------------

function tue-status
{
    _tue-dir-status $_TUE_CATKIN_SYSTEM_DIR/src
    _tue-repo-status "tue-env" "$TUE_DIR"
    _tue-repo-status "tue-env-targets" "$TUE_ENV_TARGETS_DIR"
}

# ----------------------------------------------------------------------------------------------------

function tue-git-status
{
    for pkg_dir in $_TUE_CATKIN_SYSTEM_DIR/src/*/
    do
        pkg=$(basename $pkg_dir)

        branch=$(git -C $pkg_dir rev-parse --abbrev-ref HEAD 2>&1)
        if [ $? -eq 0 ]
        then
            hash=$(git -C $pkg_dir rev-parse --short HEAD)
            printf "\e[0;36m%-20s\033[0m %-15s %s\n" "$branch" "$hash" "$pkg"
        fi
    done
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-REVERT
# ----------------------------------------------------------------------------------------------------

function tue-revert
{
    human_time="$*"

    for pkg_dir in $_TUE_CATKIN_SYSTEM_DIR/src/*/
    do
        pkg=$(basename $pkg_dir)

        branch=$(git -C $pkg_dir rev-parse --abbrev-ref HEAD 2>&1)
        if [ $? -eq 0 ] && [ $branch != "HEAD" ]
        then
            new_hash=$(git -C $pkg_dir  rev-list -1 --before="$human_time" $branch)
            current_hash=$(git -C $pkg_dir  rev-parse HEAD)
            git -C $pkg_dir  diff -s --exit-code $new_hash $current_hash
            if [ $? -eq 0 ]
            then
                newtime=$(git -C $pkg_dir  show -s --format=%ci)
                printf "\e[0;36m%-20s\033[0m %-15s \e[1m%s\033[0m %s\n" "$branch is fine" "$new_hash" "$newtime" "$pkg"
            else
                git -C $pkg_dir  checkout -q $new_hash
                newbranch=$(git -C $pkg_dir  rev-parse --abbrev-ref HEAD 2>&1)
                newtime=$(git -C $pkg_dir  show -s --format=%ci)
                echo $branch > "$pkg_dir/.do_not_commit_this"
                printf "\e[0;36m%-20s\033[0m %-15s \e[1m%s\033[0m %s\n" "$newbranch based on $branch" "$new_hash" "$newtime" "$pkg"
            fi
        else
            echo "Package $pkg could not be reverted, current state: $branch"
        fi
    done
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-REVERT-UNDO
# ----------------------------------------------------------------------------------------------------

function tue-revert-undo
{
    for pkg_dir in $_TUE_CATKIN_SYSTEM_DIR/src/*/
    do
        pkg=$(basename $pkg_dir)

        if [ -f "$pkg_dir/.do_not_commit_this" ]
        then
            echo $pkg
            git -C $pkg_dir checkout $(cat $pkg_dir/.do_not_commit_this)
            rm "$pkg_dir/.do_not_commit_this"
        fi
    done
    tue-git-status
}

# ----------------------------------------------------------------------------------------------------
#                                              TUE-GET
# ----------------------------------------------------------------------------------------------------

function _show_file
{
    if [ ! -z $2 ]
    then
        echo -e "\033[1m[$1] $2\033[0m"
        echo "--------------------------------------------------"
        if hash pygmentize 2> /dev/null
        then
            pygmentize -g $TUE_ENV_TARGETS_DIR/$1/$2
        else
            cat $TUE_ENV_TARGETS_DIR/$1/$2
        fi
        echo "--------------------------------------------------"
    else
        echo -e "_show_file requires target_name and relative file_path in target"
        return 1
    fi
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
        show           - Show the contents of (a) package(s)

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
        $TUE_DIR/installer/tue-install.bash $cmd $@
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
            $TUE_DIR/installer/tue-install.bash $cmd $@
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
            echo "[tue-get] The packages were removed from the 'installed list' but still need to be deleted from your workspace."
        else
            echo "[tue-get] The package was removed from the 'installed list' but still needs to be deleted from your workspace."
        fi
    elif [[ $cmd == "list-installed" ]]
    then
        if [[ "$1" == "-a" ]]
        then
            ls $tue_dep_dir
        else
            ls $TUE_ENV_DIR/.env/installed
        fi
    elif [[ $cmd == "show" ]]
    then
        if [ -z "$1" ]
        then
            echo "[tue-get](show) Provide at least one target name"
            return 1
        fi
        local firsttarget=true
        for target in $@
        do
            if [[ $firsttarget == false ]]
            then
                echo ""
            fi
            if [ ! -d $TUE_ENV_TARGETS_DIR/$target ]
            then
                echo "[tue-get](show) '$target' is not a valid target"
                firsttarget=false
                continue
            fi

            local firstfile="true"
            local files=($(find $TUE_ENV_TARGETS_DIR/$target -type f))

            # First show the common target files
            local main_target_files="install.yaml install.bash setup"
            for file in $main_target_files
            do
                for key in ${!files[@]}
                do
                    if [ ${files[$key]} == $TUE_ENV_TARGETS_DIR/$target/$file ]
                    then
                        if [[ $firstfile == false ]]
                        then
                            echo ""
                        fi
                        _show_file $target $file
                        firstfile=false
                        unset files[$key]
                        files=(${files[@]})
                        break
                    fi
                done
            done

            # Show all remaining files
            for file in ${files[@]}
            do
                if [[ $firstfile == false ]]
                then
                    echo ""
                fi
                _show_file $target ${file#*$TUE_ENV_TARGETS_DIR/$target/}
                firstfile=false
            done
            firsttarget=false
        done

    elif [[ $cmd == "dep" ]]
    then
        $TUE_DIR/installer/tue-get-dep.bash $@
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
        options="'dep '\n'install '\n'update '\n'remove '\n'list-installed '\n'show '"
        COMPREPLY=( $(compgen -W "$(echo -e "$options")" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "install" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_ENV_TARGETS_DIR | sed "s/.*/'& '/g")\n'--debug '\n'--branch='")" -- $cur) )
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
        elif [[ $cmd == "show" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_TARGETS_DIR | sed "s/.*/'& '/g"`" -- $cur) )
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
    --only-pks: tue-env is not checked-out to the specified branch

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
        fs="$TUE_DIR $TUE_ENV_TARGETS_DIR $fs"
    fi
    for pkg_dir in $fs
    do
        pkg=${pkg_dir#$_TUE_CATKIN_SYSTEM_DIR/src/}
        if [ -z "$NO_TUE_ENV" ]
        then
            if [[ $pkg =~ ".tue" ]]
            then
                pkg="tue-env"
            elif [[ $pkg =~ "targets" ]]
            then
                pkg="tue-env-targets"
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
#                                              TUE-DATA
# ----------------------------------------------------------------------------------------------------

source $TUE_DIR/setup/tue-data.bash

# ----------------------------------------------------------------------------------------------------
#                                             TUE-ROBOCUP
# ----------------------------------------------------------------------------------------------------

export TUE_ROBOCUP_BRANCH="rgo2019"

function _tue-repos-do
{
    # Evaluates the command of the input for tue-env and all repo's of tue-robotics.
    # The input can be multiple arguments, but if the input consists of multiple commands
    # seperated by ';' or '&&' the input needs to be captured in a string.

    local mem_pwd=$PWD

    cd $TUE_DIR
    echo -e "\033[1m[tue-env]\033[0m"
    eval "$@"

    cd $TUE_ENV_TARGETS_DIR
    echo -e "\033[1m[tue-env-targets]\033[0m"
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

    local github_url="$(_github_https "$(git config --get remote.origin.url)")"
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

