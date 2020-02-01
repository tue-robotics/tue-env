#! /usr/bin/env bash

TUE_INSTALL_DEPENDENCIES_DIR=$TUE_ENV_DIR/.env/dependencies
TUE_INSTALL_DEPENDENCIES_ON_DIR=$TUE_ENV_DIR/.env/dependencies-on
TUE_INSTALL_INSTALLED_DIR=$TUE_ENV_DIR/.env/installed

mkdir -p "$TUE_INSTALL_DEPENDENCIES_DIR"
mkdir -p "$TUE_INSTALL_DEPENDENCIES_ON_DIR"
mkdir -p "$TUE_INSTALL_INSTALLED_DIR"

TUE_INSTALL_TARGETS_DIR=$TUE_ENV_TARGETS_DIR

TUE_REPOS_DIR=$TUE_ENV_DIR/repos

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function date_stamp
{
    date +%Y_%m_%d_%H_%M_%S
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function version_gt()
{
    test "$(printf '%s\n' "$@" | sort -V | head -n 1)" != "$1";
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-error
{
    echo -e "\033[38;5;1m
Error while installing target '$TUE_INSTALL_CURRENT_TARGET':

    $1
\033[0m" | tee --append "$INSTALL_DETAILS_FILE"
    exit 1
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-warning
{
    echo -e "\033[33;5;1m[$TUE_INSTALL_CURRENT_TARGET] WARNING: $*\033[0m" | tee --append "$INSTALL_DETAILS_FILE"
    TUE_INSTALL_WARNINGS="    [$TUE_INSTALL_CURRENT_TARGET] $*\n${TUE_INSTALL_WARNINGS}"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-info
{
    echo -e "\e[0;36m[$TUE_INSTALL_CURRENT_TARGET] INFO: $*\033[0m"  | tee --append "$INSTALL_DETAILS_FILE"
    TUE_INSTALL_INFOS="    [$TUE_INSTALL_CURRENT_TARGET] $*\n${TUE_INSTALL_INFOS}"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-debug
{
    if [ "$DEBUG" = "true" ]
    then
        echo -e "\e[0;34m[$TUE_INSTALL_CURRENT_TARGET] DEBUG: $*\033[0m"  | tee --append "$INSTALL_DETAILS_FILE"
    else
        echo -e "\e[0;34m[$TUE_INSTALL_CURRENT_TARGET] DEBUG: $*\033[0m"  | tee --append "$INSTALL_DETAILS_FILE" 1> /dev/null
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-target
{
    tue-install-debug "tue-install-target $*"

    local target=$1

    tue-install-debug "Installing $target"

    # Check if valid target received as input
    if [ ! -d "$TUE_INSTALL_TARGETS_DIR"/"$target" ]
    then
        tue-install-debug "Target '$target' does not exist."
        return 1
    fi

    local parent_target=$TUE_INSTALL_CURRENT_TARGET

    # If the target has a parent target, add target as a dependency to the parent target
    if [ -n "$parent_target" ] && [ "$parent_target" != "main-loop" ]
    then
        if [ "$parent_target" != "$target" ]
        then
            echo "$target" >> "$TUE_INSTALL_DEPENDENCIES_DIR"/"$parent_target"
            echo "$parent_target" >> "$TUE_INSTALL_DEPENDENCIES_ON_DIR"/"$target"
            sort "$TUE_INSTALL_DEPENDENCIES_DIR"/"$parent_target" -u -o "$TUE_INSTALL_DEPENDENCIES_DIR"/"$parent_target"
            sort "$TUE_INSTALL_DEPENDENCIES_ON_DIR"/"$target" -u -o "$TUE_INSTALL_DEPENDENCIES_ON_DIR"/"$target"
        fi
    fi

    if [ ! -f "$TUE_INSTALL_STATE_DIR"/"$target" ]
    then
        tue-install-debug "File $TUE_INSTALL_STATE_DIR/$target does not exist, going to installation procedure"


        local install_file=$TUE_INSTALL_TARGETS_DIR/$target/install

        TUE_INSTALL_CURRENT_TARGET_DIR=$TUE_INSTALL_TARGETS_DIR/$target
        TUE_INSTALL_CURRENT_TARGET=$target

        # Empty the target's dependency file
        tue-install-debug "Emptying $TUE_INSTALL_DEPENDENCIES_DIR/$target"
        truncate -s 0 "$TUE_INSTALL_DEPENDENCIES_DIR"/"$target"
        local target_processed=false

        if [ -f "$install_file".yaml ]
        then
            tue-install-debug "Parsing $install_file.yaml"
            # Do not use 'local cmds=' because it does not preserve command output status ($?)
            local cmds
            if cmds=$("$TUE_INSTALL_SCRIPTS_DIR"/parse-install-yaml.py "$install_file".yaml)
            then
                for cmd in $cmds
                do
                    tue-install-debug "Running following command: $cmd"
                    ${cmd//^/ }
                done
                target_processed=true
            else
                tue-install-error "Invalid install.yaml: $cmds"
            fi
        fi

        if [ -f "$install_file".bash ]
        then
            tue-install-debug "Sourcing $install_file.bash"
            # shellcheck disable=SC1090
            source "$install_file".bash
            target_processed=true
        fi

        if [ "$target_processed" == false ]
        then
            tue-install-warning "Target $target does not contain a valid install.yaml/bash file"
        fi

        touch "$TUE_INSTALL_STATE_DIR"/"$target"

    fi

    TUE_INSTALL_CURRENT_TARGET=$parent_target
    TUE_INSTALL_CURRENT_TARGET_DIR=$TUE_INSTALL_TARGETS_DIR/$parent_target

    tue-install-debug "Finished installing $target"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _show_update_message
{
    # shellcheck disable=SC2086,SC2116
    if [ -n "$(echo $2)" ]
    then
        echo -e "\n    \033[1m$1\033[0m"                          | tee --append "$INSTALL_DETAILS_FILE"
        echo "--------------------------------------------------" | tee --append "$INSTALL_DETAILS_FILE"
        echo -e "$2"                                              | tee --append "$INSTALL_DETAILS_FILE"
        echo "--------------------------------------------------" | tee --append "$INSTALL_DETAILS_FILE"
        echo ""                                                   | tee --append "$INSTALL_DETAILS_FILE"
    else
        echo -e "\033[1m$1\033[0m: up-to-date"                    | tee --append "$INSTALL_DETAILS_FILE"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-svn
{
    tue-install-debug "tue-install-svn $*"

    tue-install-system-now subversion
    local res
    if [ ! -d "$2" ]
    then
        res=$(svn co "$1" "$2" --trust-server-cert --non-interactive 2>&1)
    else
        res=$(svn up "$2" --trust-server-cert --non-interactive 2>&1)
        if echo "$res" | grep -q "At revision";
        then
            res=
        fi
    fi

    _show_update_message "$TUE_INSTALL_CURRENT_TARGET" "$res"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _try_branch
{
    tue-install-debug "_try_branch $*"

    if [ -z "$2" ]
    then
        tue-install-error "Invalid _try_branch: needs two arguments (repo and branch)."
    fi

    tue-install-debug "git -C $1 checkout $2"
    _try_branch_res=$(git -C "$1" checkout "$2" 2>&1)
    tue-install-debug "$_try_branch_res"
    if [[ $_try_branch_res == "Already on "* || $_try_branch_res == "error: pathspec"* ]]
    then
        _try_branch_res=
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-git
{
    tue-install-debug "tue-install-git $*"

    local repo=$1
    local targetdir=$2
    local version=$3

    # Change url to https/ssh
    repo=$(_github_https_or_ssh "$repo")

    if [ ! -d "$targetdir" ]
    then
        tue-install-debug "git clone --recursive $repo $targetdir"
        res=$(git clone --recursive "$repo" "$targetdir" 2>&1)
        TUE_INSTALL_GIT_PULL_Q+=$targetdir
    else
        # Check if we have already pulled the repo
        if [[ $TUE_INSTALL_GIT_PULL_Q =~ $targetdir ]]
        then
            tue-install-debug "Repo previously pulled, skipping"
            # We have already pulled this repo, skip it
            res=
        else
            # Switch url of origin to use https/ssh if different
            # Get current remote url
            local current_url
            current_url=$(git -C "$targetdir" config --get remote.origin.url)

            # If different, switch url
            if [ ! "$current_url" == "$repo" ]
            then
                tue-install-debug "git -C $targetdir remote set-url origin $repo"
                git -C "$targetdir" remote set-url origin "$repo"
                tue-install-info "URL has switched to $repo"
            fi

            tue-install-debug "git -C $targetdir pull --ff-only --prune"

            local res
            res=$(git -C "$targetdir" pull --ff-only --prune 2>&1)

            tue-install-debug "$res"

            TUE_INSTALL_GIT_PULL_Q+=$targetdir

            if [[ $res == "Already up to date"* ]]
            then
                res=
            fi
        fi
    fi

    tue-install-debug "Desired version: $version"
    if [ -n "$version" ];
    then
        _try_branch "$targetdir" "$version"
        res="$res $_try_branch_res"
    fi

    tue-install-debug "Desired branch: $BRANCH"
    if [ -n "$BRANCH" ]; #Cannot be combined with version-if because this one might not exist
    then
        _try_branch "$targetdir" "$BRANCH"
        res="$res $_try_branch_res"
    fi

    _show_update_message "$TUE_INSTALL_CURRENT_TARGET" "$res"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-apply-patch
{
    tue-install-debug "tue-install-apply-patch $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: needs patch file as argument."
    fi

    if [ -z "$TUE_INSTALL_PKG_DIR" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: package directory is unknown."
    fi

    patch_file=$TUE_INSTALL_CURRENT_TARGET_DIR/$1

    if [ ! -f "$patch_file" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: patch file '$1' does not exist."
    fi

    patch -s -N -r - -p0 -d "$TUE_INSTALL_PKG_DIR" < "$patch_file"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-cp
{
    tue-install-debug "tue-install-cp $*"

    if [ -z "$2" ]
    then
        tue-install-error "Invalid tue-install-cp call: needs two arguments (source and target). The source must be relative to the installer target directory
        Command: tue-install-cp $*"
    fi

    local source_files="$TUE_INSTALL_CURRENT_TARGET_DIR"/"$1"

    # Check if user is allowed to write on target destination
    local root_required=true
    if namei -l "$2" | grep -q "$(whoami)"
    then
        root_required=false
    fi

    local cp_target=
    local cp_target_parent_dir=

    if [ -d "$2" ]
    then
        cp_target_parent_dir="${2%%/}"
    else
        cp_target_parent_dir="$(dirname "$2")"
    fi

    for file in $source_files
    do
        if [ ! -f "$file" ]
        then
            tue-install-error "Invalid tue-install-cp call: file '$file' does not exist."
        fi

        if [ -d "$2" ]
        then
            cp_target="$cp_target_parent_dir"/$(basename "$file")
        else
            cp_target="$2"
        fi

        if ! cmp --quiet "$file" "$cp_target"
        then
            tue-install-debug "File $file and $cp_target are different, copying..."
            if "$root_required"
            then
                tue-install-debug "Using elevated privileges (sudo)"
                sudo mkdir --parents --verbose "$cp_target_parent_dir" && sudo cp --verbose "$file" "$cp_target"
            else
                mkdir --parents --verbose "$cp_target_parent_dir" && cp --verbose "$file" "$cp_target"
            fi
        else
            tue-install-debug "File $file and $cp_target are the same, no action needed"
        fi

    done
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# Reads SOURCE_FILE and looks in TARGET_FILE for the first and last line of SOURCE_FILE. If these
# are not found, SOURCE_FILE is appended to TARGET_FILE. Otherwise, the appearance of the first and
# last line of SOURCE_FILE in TARGET_FILE, and everything in between, is replaced by the contents
# of SOURCE_FILE.
# This is useful for adding text blocks to files and allowing to change only that part of the file
# on a next update. It is advised to start and end SOURCE_FILE with unique tags, e.g.:
#
#    # BEGIN TU/E BLOCK
#    .... some text ...
#    # END TU/E BLOCK
#
function tue-install-add-text
{
    tue-install-debug "tue-install-add-text $*"

    if [ -z "$2" ]
    then
        tue-install-error "Invalid tue-install-add-text call. Usage: tue-install-add-text SOURCE_FILE TARGET_FILE"
    fi

    tue-install-debug "tue-install-add-text $*"

    local source_file=$1
    # shellcheck disable=SC2088
    if [[ "$source_file" == "/"* ]] || [[ "$source_file" == "~/"* ]]
    then
        tue-install-error "tue-install-add-text: Only relative source files to the target directory are allowed"
    else
        source_file="$TUE_INSTALL_CURRENT_TARGET_DIR"/"$source_file"
    fi
    local target_file=$2
    # shellcheck disable=SC2088
    if [[ "$target_file" != "/"* ]] && [[ "$source_file" != "~/"* ]]
    then
        tue-install-error "tue-install-add-text: target file needs to be absolute or relative to the home directory"
    fi

    local root_required=true
    if namei -l "$target_file" | grep -q "$(whoami)"
    then
        tue-install-debug "tue-install-add-text: NO root required"
        root_required=false
    else
        tue-install-debug "tue-install-add-text: root required"
    fi

    if [ ! -f "$source_file" ]
    then
        tue-install-error "tue-install-add-text: No such source file: $source_file"
    fi

    if [ ! -f "$target_file" ]
    then
        tue-install-error "tue-install-add-text: No such target file: $target_file"
    fi

    local begin_tag end_tag text
    begin_tag=$(head -n 1 "$source_file")
    end_tag=$(awk '/./{line=$0} END{print line}' "$source_file")
    text=$(sed -e :a -e '/^\n*$/{$d;N;};/\n&/ba' "$source_file")
    tue-install-debug "tue-install-add-text: Lines to be added: \n$text"

    if ! grep -q "$begin_tag" "$target_file"
    then
        tue-install-debug "tue-install-add-text: Appending $target_file"
        if $root_required
        then
            echo -e "$text" | sudo tee --append "$target_file" 1> /dev/null
        else
            echo -e "$text" | tee --append "$target_file" 1> /dev/null
        fi
    else
        tue-install-debug "tue-install-add-text: Begin tag already in $target_file, so comparing the files for changed lines"
        local tmp_source_file="/tmp/tue-install-add-text_source_temp_${USER}_${TUE_INSTALL_CURRENT_TARGET}_${stamp}"
        local tmp_target_file="/tmp/tue-install-add-text_target_temp_${USER}_${TUE_INSTALL_CURRENT_TARGET}_${stamp}"

        echo "$text" | tee "$tmp_source_file" > /dev/null
        sed -e "/^$end_tag/r $tmp_source_file" -e "/^$begin_tag/,/^$end_tag/d" "$target_file" | tee "$tmp_target_file" 1> /dev/null

        if ! cmp --quiet "$tmp_target_file" "$target_file"
        then
            tue-install-debug "tue-install-add-text: Lines are changed, so copying"
            if $root_required
            then
                sudo mv "$tmp_target_file" "$target_file"
            else
                mv "$tmp_target_file" "$target_file"
            fi
        else
            tue-install-debug "tue-install-add-text: Lines have not changed, so not copying"
        fi
        rm "$tmp_source_file" "$tmp_target_file"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-system
{
    tue-install-debug "tue-install-system $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-system call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to apt list"
    TUE_INSTALL_SYSTEMS="$1 $TUE_INSTALL_SYSTEMS"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-system-now
{
    tue-install-debug "tue-install-system-now $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-system-now call: needs package as argument."
    fi

    local pkgs_to_install=""
    # shellcheck disable=SC2048
    for pkg in $*
    do
        # Check if pkg is not already installed dpkg -S does not cover previously removed packages
        # Based on https://stackoverflow.com/questions/1298066
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "ok installed"
        then
            pkgs_to_install="$pkgs_to_install $pkg"
        else
            tue-install-debug "$pkg is already installed"
        fi
    done

    if [ -n "$pkgs_to_install" ]
    then
        echo -e "Going to run the following command:\n"
        echo -e "sudo apt-get install --assume-yes -qq --no-install-recommends $pkgs_to_install\n"

        # Wait for apt-lock first (https://askubuntu.com/a/375031)
        i=0
        tput sc
        while fuser /var/lib/dpkg/lock >/dev/null 2>&1
        do
            case $((i % 4)) in
                0 ) j="-" ;;
                1 ) j="\\" ;;
                2 ) j="|" ;;
                3 ) j="/" ;;
            esac
            tput rc
            echo -en "\r[$j] Waiting for other software managers to finish..."
            sleep 0.5
            ((i=i+1))
        done

        # shellcheck disable=SC2086
        sudo apt-get install --assume-yes -qq --no-install-recommends $pkgs_to_install || sudo apt-get update  # Update and try again
        # shellcheck disable=SC2086
        sudo apt-get install --assume-yes -qq --no-install-recommends $pkgs_to_install || exit 1
        tue-install-debug "Installed $pkgs_to_install ($?)"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ppa
{
    tue-install-debug "tue-install-ppa $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-ppa call: needs ppa as argument."
    fi
    ppa="$*"

    if [[ $ppa != ppa:* && $ppa != deb* ]]
    then
        tue-install-error "Invalid tue-install-ppa call: needs to start with 'ppa:' or 'deb '"
    fi
    tue-install-debug "Adding $ppa to PPA list"
    TUE_INSTALL_PPA="${TUE_INSTALL_PPA} ${ppa// /#}"  # Replace space by # to support for-loops later
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-pip
{
    tue-install-debug "tue-install-pip $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-pip call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to pip list"
    TUE_INSTALL_PIPS="$1 $TUE_INSTALL_PIPS"
}

function tue-install-pip3
{
    tue-install-debug "tue-install-pip3 $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-pip3 call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to pip3 list"
    TUE_INSTALL_PIP3S="$1 $TUE_INSTALL_PIP3S"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-snap
{
    tue-install-debug "tue-install-snap $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-snap call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to snap list"
    TUE_INSTALL_SNAPS="$1 $TUE_INSTALL_SNAPS"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-snap-now
{
    tue-install-debug "tue-install-snap-now $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-snap call: needs package as argument."
    fi

    tue-install-system-now snapd


    local snaps_to_install snaps_installed
    snaps_to_install=""
    snaps_installed=$(snap list)
    # shellcheck disable=SC2068
    for pkg in $@
    do
        if [[ ! $snaps_installed == *$pkg* ]] # Check if pkg is not already installed
        then
            snaps_to_install="$snaps_to_install $pkg"
            tue-install-debug "snap pkg: $pkg is not yet installed"
        else
            tue-install-debug "snap pkg: $pkg is already installed"
        fi
    done

    if [ -n "$snaps_to_install" ]
    then
        echo -e "Going to run the following command:\n"
        for pkg in $snaps_to_install
        do
            echo -e "yes | sudo snap install --classic $pkg\n"
            tue-install-debug "yes | sudo snap install --classic $pkg"
            yes | sudo snap install --classic "$pkg"
        done
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-dpkg
{
    tue-install-debug "tue-install-dpkg $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-dpkg call: needs package as argument."
    fi
    tue-install-debug "Installing dpkg $1"
    sudo dpkg --install "$1"
    tue-install-debug "sudo apt-get --fix-broken --assume-yes install --no-install-recommends"
    sudo apt-get --fix-broken --assume-yes install --no-install-recommends
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ros
{
    tue-install-debug "tue-install-ros $*"

    local install_type=$1
    local src=$2
    local sub_dir=$3
    local version=$4

    tue-install-debug "Installing ros package: type = $install_type, source = $src"

    [ -n "$TUE_ROS_DISTRO" ] || tue-install-error "Environment variable 'TUE_ROS_DISTRO' is not set."

    local ros_pkg_name=${TUE_INSTALL_CURRENT_TARGET#ros-}
    if [[ $ros_pkg_name == *-* ]]
    then
        tue-install-error "A rospackage cannot contain dashes (${ros_pkg_name})"
        return 1
    fi

    # First of all, make sure ROS itself is installed
    tue-install-target ros || tue-install-error "Failed to install target 'ROS'"

    if [ "$install_type" = "system" ]
    then
        tue-install-debug "tue-install-system ros-$TUE_ROS_DISTRO-$src"
        tue-install-system ros-"$TUE_ROS_DISTRO"-"$src"
        return 0
    fi

    if [ -z "$ROS_PACKAGE_INSTALL_DIR" ]
    then
        tue-install-error "Environment variable ROS_PACKAGE_INSTALL_DIR not set."
    fi

    # Make sure the ROS package install dir exists
    tue-install-debug "Creating ROS package install dir: $ROS_PACKAGE_INSTALL_DIR"
    mkdir -p "$ROS_PACKAGE_INSTALL_DIR"

    local ros_pkg_dir="$ROS_PACKAGE_INSTALL_DIR"/"$ros_pkg_name"
    local repos_dir="$TUE_REPOS_DIR"/"$src"
    # replace spaces with underscores
    repos_dir=${repos_dir// /_}
    # now, clean out anything that's not alphanumeric or an underscore
    repos_dir=${repos_dir//[^a-zA-Z0-9\/\.-]/_}

    # For backwards compatibility: if the ros_pkg_dir already exists and is NOT
    # a symbolic link, then update this direcory instead of creating a symbolic
    # link from the repos directory. In other words, the ros_pkg_dir becomes the
    # repos_dir
    if [[ -d $ros_pkg_dir && ! -L $ros_pkg_dir ]]
    then
        repos_dir=$ros_pkg_dir
    fi
    tue-install-debug "repos_dir = $repos_dir"

    if [ "$install_type" = "git" ]
    then
        tue-install-git "$src" "$repos_dir" "$version"
        tue-install-debug "git clone $src"
    elif [ "$install_type" = "svn" ]
    then
        tue-install-svn "$src" "$repos_dir" "$version"
    else
        tue-install-error "Unknown ros install type: '${install_type}'"
    fi

    if [ -d "$repos_dir" ]
    then
        if [ ! -d "$repos_dir"/"$sub_dir" ]
        then
            tue-install-error "Subdirectory '$sub_dir' does not exist for URL '$src'."
        fi

        if [ -L "$ros_pkg_dir" ]
        then
            # Test if the current symbolic link points to the same repository dir. If not, give a warning
            # because it means the source URL has changed
            if [ ! "$ros_pkg_dir" -ef "$repos_dir"/"$sub_dir" ]
            then
                tue-install-info "URL has changed to $src/$sub_dir"
                rm "$ros_pkg_dir"
                ln -s "$repos_dir"/"$sub_dir" "$ros_pkg_dir"
            fi
        elif [ ! -d "$ros_pkg_dir" ]
        then
            # Create a symbolic link to the system workspace
            ln -s "$repos_dir"/"$sub_dir" "$ros_pkg_dir"
        fi

        if [[ "$NO_ROS_DEPS" = "true" ]]
        then
            tue-install-debug "No need to parse package.xml for dependencies"
            return 0
        fi

        if [ -f "$ros_pkg_dir"/package.xml ]
        then
            # Catkin
            deps=$("$TUE_INSTALL_SCRIPTS_DIR"/parse-ros-package-deps.py "$ros_pkg_dir"/package.xml)
            tue-install-debug "Parsed package.xml \n$deps"

            for dep in $deps
            do
                # Preference given to target name starting with ros-
                tue-install-target ros-"$dep" || tue-install-target "$dep" || \
                    tue-install-error "Targets 'ros-$dep' and '$dep' does not exist."
            done

        else
            tue-install-warning "Does not contain a valid ROS package.xml."
        fi

    else
        tue-install-error "Checking out $src was not successful."
    fi

    TUE_INSTALL_PKG_DIR=$ros_pkg_dir
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _missing_targets_check
{
    tue-install-debug "_missing_targets_check $*"

    # Check if valid target received as input
    local targets="$1"
    local missing_targets=""
    local target

    for target in $targets
    do
        if [ ! -d "$TUE_INSTALL_TARGETS_DIR"/"$target" ]
        then
            missing_targets="$target${missing_targets:+ ${missing_targets}}"
        fi
    done

    if [ -n "$missing_targets" ]
    then
        missing_targets=$(echo "$missing_targets" | tr " " "\n" | sort)
        echo -e "\e[31mThe following installed targets don't exist (anymore):\n$missing_targets \e[0m"
        exit 1
    fi

    return 0
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                           MAIN LOOP
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

TUE_INSTALL_CURRENT_TARGET="main-loop"

tue_get_cmd=$1
shift

# idiomatic parameter and option handling in sh
targets=""
while test $# -gt 0
do
    case "$1" in
        --debug) DEBUG=true
            ;;
        --no-ros-deps) NO_ROS_DEPS=true
            ;;
        --branch*)
            # shellcheck disable=SC2001
            BRANCH=$(echo "$1" | sed -e 's/^[^=]*=//g')
            ;;
        --*) echo "unknown option $1"
            ;;
        *) targets="$targets $1"
            ;;
    esac
    shift
done


# Create log file
stamp=$(date_stamp)
INSTALL_DETAILS_FILE=/tmp/tue-get-details-$stamp
touch "$INSTALL_DETAILS_FILE"

# Make sure tools used by this installer are installed
tue-install-system-now python-yaml git python-pip python3-pip

# CATKIN PACKAGES
ROS_PACKAGE_INSTALL_DIR=$TUE_SYSTEM_DIR/src

TUE_INSTALL_SCRIPTS_DIR=$TUE_DIR/installer

TUE_INSTALL_GENERAL_STATE_DIR=/tmp/tue-installer
if [ ! -d $TUE_INSTALL_GENERAL_STATE_DIR ]
then
    tue-install-debug "mkdir $TUE_INSTALL_GENERAL_STATE_DIR"
    mkdir "$TUE_INSTALL_GENERAL_STATE_DIR"
    tue-install-debug "chmod a+rwx $TUE_INSTALL_GENERAL_STATE_DIR"
    chmod a+rwx "$TUE_INSTALL_GENERAL_STATE_DIR"
fi

TUE_INSTALL_STATE_DIR=$TUE_INSTALL_GENERAL_STATE_DIR/$stamp
mkdir -p "$TUE_INSTALL_STATE_DIR"

TUE_INSTALL_GIT_PULL_Q=()

TUE_INSTALL_SYSTEMS=
TUE_INSTALL_PPA=
TUE_INSTALL_PIPS=
TUE_INSTALL_PIP3S=
TUE_INSTALL_SNAPS=

TUE_INSTALL_WARNINGS=
TUE_INSTALL_INFOS=


if [[ -z "${targets// }" ]] #If only whitespace
then
    # If no targets are provided, update all installed targets
    targets=$(ls "$TUE_INSTALL_INSTALLED_DIR")
fi

# Check if all installed targets exist in the targets repo
_missing_targets_check "$targets"

for target in $targets
do
    tue-install-debug "Main loop: installing $target"
    # Next line shouldn't error anymore with _missing_targets_check
    tue-install-target "$target" || tue-install-error "Installed target: '$target' doesn't exist (anymore)"

    if [[ "$tue_get_cmd" == "install" ]]
    then
        # Mark as installed
        tue-install-debug "[$target] marked as installed after a successful install"
        touch "$TUE_INSTALL_INSTALLED_DIR"/"$target"
    else
        tue-install-debug "[$target] succesfully updated"
    fi
done


# Display infos
if [ -n "$TUE_INSTALL_INFOS" ]
then
    echo -e "\e[0;36m\nSome information you may have missed:\n\n$TUE_INSTALL_INFOS\033[0m"
fi

# Display warnings
if [ -n "$TUE_INSTALL_WARNINGS" ]
then
    echo -e "\033[33;5;1m\nOverview of warnings:\n\n$TUE_INSTALL_WARNINGS\033[0m"
fi


# Remove temp directories
rm -rf "$TUE_INSTALL_STATE_DIR"


# Installing all the ppa repo's, which are collected during install
if [ -n "$TUE_INSTALL_PPA" ]
then
    TUE_INSTALL_CURRENT_TARGET="PPA-ADD"

    PPA_ADDED=""
    for ppa in ${TUE_INSTALL_PPA}
    do
        ppa=${ppa//#/ }  # Replace # back to spaces
        if ! grep -h "^deb.*${ppa#ppa:}" /etc/apt/sources.list.d/* 1>/dev/null 2>&1 && ! grep -q "$ppa" /etc/apt/sources.list
        then
            tue-install-system-now software-properties-common
            tue-install-info "Adding ppa: $ppa"
            sudo add-apt-repository --yes "$ppa"
            PPA_ADDED=true
        else
            tue-install-debug "$ppa is already added previously"
        fi
    done
    if [ -n "$PPA_ADDED" ]
    then
        tue-install-debug "Updating apt-get"
        sudo apt-get update -qq
    fi
fi


# Installing all system (apt-get) targets, which are collected during the install
if [ -n "$TUE_INSTALL_SYSTEMS" ]
then
    TUE_INSTALL_CURRENT_TARGET="APT-GET"

    tue-install-debug "tue-install-system-now $TUE_INSTALL_SYSTEMS"
    tue-install-system-now "$TUE_INSTALL_SYSTEMS"
fi


# Installing all python (pip) targets, which are collected during the install
if [ -n "$TUE_INSTALL_PIPS" ]
then
    TUE_INSTALL_CURRENT_TARGET="PIP"

    pip_version=$(pip2 --version | awk '{print $2}')
    if version_gt "9" "$pip_version"
    then
        tue-install-debug "pip2 not yet version >=9, but $pip_version"
        sudo -H pip2 install --upgrade pip
        hash -r
    else
        tue-install-debug "Already pip2>=9\n"
    fi

    # Just install the packages because checking for installation is not faster
    echo -e "Going to run the following command:\n"
    echo -e "yes | pip2 install --user $TUE_INSTALL_PIPS\n"
    yes | pip2 install --user $TUE_INSTALL_PIPS
fi


# Installing all python3 (pip3) targets, which are collected during the install
if [ -n "$TUE_INSTALL_PIP3S" ]
then
    TUE_INSTALL_CURRENT_TARGET="PIP3"

    pip3_version=$(pip3 --version | awk '{print $2}')
    if version_gt "9" "$pip3_version"
    then
        tue-install-debug "pip3 not yet version >=9, but $pip3_version"
        sudo -H pip3 install --upgrade pip
        hash -r
    else
        tue-install-debug "Already pip3>=9\n"
    fi

    # Just install the packages because checking for installation is not faster
    echo -e "Going to run the following command:\n"
    echo -e "yes | pip3 install --user $TUE_INSTALL_PIP3S\n"
    yes | pip3 install --user $TUE_INSTALL_PIP3S
fi


# Installing all snap targets, which are collected during the install
if [ -n "$TUE_INSTALL_SNAPS" ]
then
    TUE_INSTALL_CURRENT_TARGET="SNAP"

    tue-install-debug "tue-install-snap-now $TUE_INSTALL_SNAPS"
    tue-install-snap-now "$TUE_INSTALL_SNAPS"
fi

return 0
