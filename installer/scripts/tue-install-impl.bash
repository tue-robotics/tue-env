#!/bin/bash

TUE_INSTALL_DEPENDENCIES_DIR=$TUE_ENV_DIR/.env/dependencies
TUE_INSTALL_DEPENDENCIES_ON_DIR=$TUE_ENV_DIR/.env/dependencies-on
TUE_INSTALL_INSTALLED_DIR=$TUE_ENV_DIR/.env/installed

mkdir -p $TUE_INSTALL_DEPENDENCIES_DIR
mkdir -p $TUE_INSTALL_DEPENDENCIES_ON_DIR
mkdir -p $TUE_INSTALL_INSTALLED_DIR

TUE_INSTALL_TARGETS_DIR=$TUE_DIR/installer/targets

TUE_SYSTEM_DIR=$TUE_ENV_DIR/system
TUE_REPOS_DIR=$TUE_ENV_DIR/repos

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function date_stamp
{
    echo $(date +%Y_%m_%d_%H_%M_%S)
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
\033[0m"
    return 1
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-warning
{
    echo -e "\033[33;5;1m[$TUE_INSTALL_CURRENT_TARGET] WARNING: $1\033[0m"
    TUE_INSTALL_WARNINGS="    [$TUE_INSTALL_CURRENT_TARGET] $1\n${TUE_INSTALL_WARNINGS}"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-info
{
    echo -e "\e[0;36m[$TUE_INSTALL_CURRENT_TARGET] INFO: $1\033[0m"
    TUE_INSTALL_INFOS="    [$TUE_INSTALL_CURRENT_TARGET] $1\n${TUE_INSTALL_INFOS}"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-debug
{
    if [ "$DEBUG" = "true" ]; then
        echo -e "\e[0;34m[$TUE_INSTALL_CURRENT_TARGET] DEBUG: $1\033[0m"
    fi
}


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-target
{
    local target=$1

    tue-install-debug "Installing $target"

    # Check if valid target received as input
    if [ ! -d $TUE_INSTALL_TARGET_DIR/$target ]
    then
        tue-install-debug "Target '$target' does not exist."
        return 1
    fi

    local parent_target=$TUE_INSTALL_CURRENT_TARGET

    # If the target has a parent target, add target as a dependency to the parent target
    if [ -n "$parent_target" ]
    then
        if [ "$parent_target" != "$target" ]
        then
            echo "$target" >> $TUE_INSTALL_DEPENDENCIES_DIR/$parent_target
            echo "$parent_target" >> $TUE_INSTALL_DEPENDENCIES_ON_DIR/$target
            sort $TUE_INSTALL_DEPENDENCIES_DIR/$parent_target -u -o $TUE_INSTALL_DEPENDENCIES_DIR/$parent_target
            sort $TUE_INSTALL_DEPENDENCIES_ON_DIR/$target -u -o $TUE_INSTALL_DEPENDENCIES_ON_DIR/$target
        fi
    fi

    if [ ! -f $TUE_INSTALL_STATE_DIR/$target ]; then
        tue-install-debug "File $TUE_INSTALL_STATE_DIR/$target does not exist, going to installation procedure"


        local install_file=$TUE_INSTALL_TARGET_DIR/$target/install

        TUE_INSTALL_CURRENT_TARGET_DIR=$TUE_INSTALL_TARGET_DIR/$target
        TUE_INSTALL_CURRENT_TARGET=$target

        # Empty the target's dependency file
        tue-install-debug "Emptying $TUE_INSTALL_DEPENDENCIES_DIR/$target"
        truncate -s 0 $TUE_INSTALL_DEPENDENCIES_DIR/$target

        if [ -f $install_file.yaml ]
        then
            tue-install-debug "Parsing $install_file.yaml"
            # Do not use 'local cmds=' because it does not preserve command output status ($?)
            cmds=$($TUE_INSTALL_SCRIPTS_DIR/parse-install-yaml.py $install_file.yaml)
            if [ $? -eq 0 ]; then
                for cmd in $cmds
                do
                    tue-install-debug "Running following command: $cmd"
                    ${cmd//^/ }
                done
            else
                tue-install-error "Invalid install.yaml: $cmd"
            fi
        fi

        if [ -f $install_file.bash ]
        then
            tue-install-debug "Sourcing $install_file.bash"
            source $install_file.bash
        fi

        touch $TUE_INSTALL_STATE_DIR/$target
    fi

    TUE_INSTALL_CURRENT_TARGET=$parent_target
    TUE_INSTALL_CURRENT_TARGET_DIR=$TUE_INSTALL_TARGET_DIR/$parent_target

    tue-install-debug "Finished installing $target"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _show_update_message
{
    if [ -n "$(echo $2)" ]
    then
        echo -e "\n    \033[1m$1\033[0m"
        echo "--------------------------------------------------"
        echo -e "$2"
        echo "--------------------------------------------------"
        echo ""
    else
        echo -e "\033[1m$1\033[0m: up-to-date"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-svn
{
    if [ ! -d $2 ]; then
        res=$(svn co $1 $2 --trust-server-cert --non-interactive 2>&1)
    else
        res=$(svn up $2 --trust-server-cert --non-interactive 2>&1)
        if echo "$res" | grep -q "At revision";
        then
            res=
        fi
    fi

    _show_update_message $TUE_INSTALL_CURRENT_TARGET "$res"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _try_branch
{
    tue-install-debug "_try_branch $1 $2"
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
    tue-install-debug "tue-install-git $@"
    local repo=$1
    local targetdir=$2
    local version=$3

    if [ ! -d $2 ]; then
        tue-install-debug "git clone --recursive $repo $targetdir"
        res=$(git clone --recursive $repo $targetdir 2>&1)
        TUE_INSTALL_GIT_PULL_Q+=$targetdir
    else
        # Check if we have already pulled the repo
        if [[ $TUE_INSTALL_GIT_PULL_Q =~ $targetdir ]]
        then
            tue-install-debug "Repo previously pulled, skipping"
            # We have already pulled this repo, skip it
            res=
        else
            tue-install-debug "git -C $targetdir pull --ff-only --prune"

            res=$(git -C $targetdir pull --ff-only --prune 2>&1)

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
        _try_branch $targetdir $version
        res="$res $_try_branch_res"
    fi

    tue-install-debug "Desired branch: $BRANCH"
    if [ -n "$BRANCH" ]; #Cannot be combined with version-if because this one might not exist
    then
        _try_branch $targetdir "$BRANCH"
        res="$res $_try_branch_res"
    fi

    _show_update_message $TUE_INSTALL_CURRENT_TARGET "$res"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-apply-patch
{
    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: needs patch file as argument."
    fi

    if [ -z "$TUE_INSTALL_PKG_DIR" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: package directory is unknown."
    fi

    patch_file=$TUE_INSTALL_CURRENT_TARGET_DIR/$1

    if [ ! -f $patch_file ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: patch file '$1' does not exist."
    fi

    patch -s -N -r - -p0 -d $TUE_INSTALL_PKG_DIR < $patch_file
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-cp
{
    if [ -z "$2" ]
    then
        tue-install-error "Invalid tue-install-cp call: needs two arguments (source and target)."
    fi

    file=$TUE_INSTALL_CURRENT_TARGET_DIR/$1

    if [ ! -f $file ]
    then
        tue-install-error "Invalid tue-install-cp call: file '$1' does not exist."
    fi

    # Check if user is allowed to write on target destination
    local root_required=true
    if namei -l $2 | grep -q $USER
    then
        root_required=false
    fi

    if ! cmp --quiet $file $2
    then
        if $root_required
        then
            sudo cp --verbose $file $2
        else
            cp --verbose $file $2
        fi
    fi
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
    if [ -z "$2" ]
    then
        tue-install-error "Invalid tue-install-add-text call. Usage: tue-install-add-text SOURCE_FILE TARGET_FILE"
    fi

    local source_file=$TUE_INSTALL_CURRENT_TARGET_DIR/$1
    local target_file=$2

    local root_required=true
    if namei -l $target_file | grep -q $USER
    then
        root_required=false
    fi

    if [ ! -f $source_file ]
    then
        tue-install-error "tue-install-add-text: No such file: $source_file"
    fi

    if [ ! -f $target_file ]
    then
        tue-install-error "tue-install-add-text: No such file: $target_file"
    fi

    local begin_tag=$(head -n 1 $source_file)
    local end_tag=$(tail -n 1 $source_file)
    local text=$(cat $source_file)

    if ! grep -q "$begin_tag" $target_file
    then
        if $root_required
        then
            echo -e "$text" | sudo tee --append $target_file
        else
            echo -e "$text" | tee --append $target_file
        fi
    else
        if $root_required
        then
            sed -e "/^$end_tag/r $source_file" -e "/^$begin_tag/,/^$end_tag/d" $target_file | sudo tee $target_file.tmp
            sudo mv $target_file.tmp $target_file
        else
            sed -e "/^$end_tag/r $source_file" -e "/^$begin_tag/,/^$end_tag/d" $target_file | tee $target_file.tmp
            mv $target_file.tmp $target_file
        fi
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-system
{
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
    tue-install-debug "tue-install-system-now $@"
    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-system-now call: needs package as argument."
    fi

    pkgs_to_install=""
    for pkg in $@ # Unquoted to seperate arguments based on spaces
    do
        # Check if pkg is not already installed dpkg -S does not cover previously removed packages
        # Based on https://stackoverflow.com/questions/1298066
        if ! dpkg-query -W -f='${Status}' $pkg 2>/dev/null | grep -q "ok installed"
        then
            pkgs_to_install="$pkgs_to_install $pkg"
        else
            tue-install-debug "$pkg is already installed"
        fi
    done

    if [ -n "$pkgs_to_install" ]
    then
        echo -e "Going to run the following command:\n"
        echo -e "sudo apt-get install --assume-yes $pkgs_to_install\n"

        # Wait for apt-lock first (https://askubuntu.com/a/375031)
        i=0
        tput sc
        while fuser /var/lib/dpkg/lock >/dev/null 2>&1
        do
            case $(($i % 4)) in
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

        sudo apt-get install --assume-yes $pkgs_to_install
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ppa
{
    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-ppa call: needs ppa as argument."
    fi

    if [[ ! $1 == ppa:* ]]
    then
        tue-install-error "Invalid tue-install-ppa call: needs to start with ppa:"
    fi
    tue-install-debug "Adding $1 to PPA list"
    TUE_INSTALL_PPA="$1 $TUE_INSTALL_PPA"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-pip
{
    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-pip call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to pip list"
    TUE_INSTALL_PIPS="$1 $TUE_INSTALL_PIPS"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-snap
{
    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-snap call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to snap list"
    TUE_INSTALL_SNAPS="$1 $TUE_INSTALL_SNAPS"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ros
{
    install_type=$1
    src=$2
    sub_dir=$3
    version=$4

    tue-install-debug "Installing ros package: type = $install_type, source = $src"

    [ -n "$TUE_ROS_DISTRO" ] || tue-install-error "Environment variable 'TUE_ROS_DISTRO' is not set."

    local ros_pkg_name=${TUE_INSTALL_CURRENT_TARGET#ros-}

    # First of all, make sure ROS itself is installed
    tue-install-target ros

    if [ "$install_type" = "system" ]; then
        tue-install-debug "tue-install-system ros-$TUE_ROS_DISTRO-$src"

        # all HSR system targets from Toyota need extra apt sources
        if [[ $src == *"hsr"* ||  "$src" == *"tmc"* ]]
        then
            tue-install-target hsr-setup
        fi

        tue-install-system ros-$TUE_ROS_DISTRO-$src
        return
    fi

    if [ -z $ROS_PACKAGE_INSTALL_DIR ]; then
        tue-install-error "Environment variable ROS_PACKAGE_INSTALL_DIR not set."
    fi

    # Make sure the ROS package install dir exists
    tue-install-debug "Creating ROS package install dir: $ROS_PACKAGE_INSTALL_DIR"
    mkdir -p $ROS_PACKAGE_INSTALL_DIR

    local ros_pkg_dir=$ROS_PACKAGE_INSTALL_DIR/$ros_pkg_name
    local repos_dir=$TUE_REPOS_DIR/$src
    # replace spaces with underscores
    repos_dir=${repos_dir// /_}
    # now, clean out anything that's not alphanumeric or an underscore
    repos_dir=${repos_dir//[^a-zA-Z0-9\/\.-]/_}

    #mkdir -p $repos_dir/..

    # For backwards compatibility: if the ros_pkg_dir already exists and is NOT
    # a symbolic link, then update this direcory instead of creating a symbolic
    # link from the repos directory. In other words, the ros_pkg_dir becomes the
    # repos_dir
    if [[ -d $ros_pkg_dir && ! -L $ros_pkg_dir ]]
    then
        repos_dir=$ros_pkg_dir
    fi
    tue-install-debug "repos_dir = $repos_dir"

    if [ "$install_type" = "git" ]; then
        tue-install-git $src $repos_dir $version
        tue-install-debug "git clone $src"
        echo "git clone $src" >> $INSTALL_DETAILS_FILE
        [ "$version" ] && echo "# NOTE: check-out version $version" >> $INSTALL_DETAILS_FILE
    elif [ "$install_type" = "svn" ]; then
        tue-install-svn $src $repos_dir $version
        if [ "$version" ]; then
            echo "svn co $src -r $version" >> $INSTALL_DETAILS_FILE
        else
            echo "svn co $src" >> $INSTALL_DETAILS_FILE
        fi
    else
        tue-install-error "Unknown ros install type: '${install_type}'"
        return 1
    fi

    if [ -d $repos_dir ]; then

        if [ ! -d $repos_dir/$sub_dir ]
        then
            tue-install-error "Subdirectory '$sub_dir' does not exist for URL '$src'."
        fi

        if [ -L $ros_pkg_dir ]
        then
            # Test if the current symbolic link points to the same repository dir. If not, give a warning
            # because it means the source URL has changed
            if [ ! $ros_pkg_dir -ef $repos_dir/$sub_dir ]
            then
                tue-install-info "URL has changed to $src/$subdir"
                rm $ros_pkg_dir
                ln -s $repos_dir/$sub_dir $ros_pkg_dir
            fi
        elif [ ! -d $ros_pkg_dir ]
        then
            # Create a symbolic link to the system workspace
            ln -s $repos_dir/$sub_dir $ros_pkg_dir
        fi

        if  [ -f $ros_pkg_dir/package.xml ]; then
            # Catkin
            deps=`$TUE_INSTALL_SCRIPTS_DIR/parse-ros-package-deps.py $ros_pkg_dir/package.xml`
            tue-install-debug "Parsed package.xml \n$deps"

            for dep in $deps
            do
                # Preference given to target name starting with ros-
                tue-install-target ros-$dep || tue-install-target $dep || \
                    tue-install-error "Target '$dep' does not exist."
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

function generate_setup_file
{
    # Check whether this target was already added to the setup
    if [[ "$TUE_SETUP_TARGETS" == *" $1 "* ]];
    then
        return
    fi

    TUE_SETUP_TARGETS=" $1$TUE_SETUP_TARGETS"

    # Check if the dependency file exists. If not, return
    if [ ! -f $TUE_INSTALL_DEPENDENCIES_DIR/$1 ]
    then
        return
    fi

    # Recursively add a setup for each dependency
    deps=`cat $TUE_INSTALL_DEPENDENCIES_DIR/$1`
    for dep in $deps
    do
        # You shouldn't depend on yourself
        if [ "$1" != "$dep" ]
        then
            generate_setup_file $dep
        fi
    done

    local tue_setup_file=$TUE_INSTALL_TARGETS_DIR/$1/setup
    if [ -f $tue_setup_file ]
    then
        echo "source $tue_setup_file" >> $TUE_ENV_DIR/.env/setup/target_setup.bash
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                           MAIN LOOP
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# Make sure tools used by this installer are installed
tue-install-system-now python-yaml git subversion python-pip

stamp=$(date_stamp)
INSTALL_DETAILS_FILE=/tmp/tue-get-details-$stamp
touch $INSTALL_DETAILS_FILE

# CATKIN PACKAGES
ROS_PACKAGE_INSTALL_DIR=$TUE_SYSTEM_DIR/src

TUE_INSTALL_TARGET_DIR=$TUE_INSTALL_TARGETS_DIR
TUE_INSTALL_SCRIPTS_DIR=$TUE_DIR/installer/scripts

TUE_INSTALL_STATE_DIR=/tmp/tue-installer/$stamp
mkdir -p $TUE_INSTALL_STATE_DIR

TUE_INSTALL_GIT_PULL_Q=()

TUE_INSTALL_SYSTEMS=
TUE_INSTALL_PPA=
TUE_INSTALL_PIPS=
TUE_INSTALL_SNAPS=

TUE_INSTALL_WARNINGS=
TUE_INSTALL_INFOS=

if [ -d "/usr/local/cuda/" ]
then
    export TUE_CUDA=1
fi

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                           For backwards compatibility
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
if [ -d $TUE_REPOS_DIR/https: ]
then
    echo "Moving to new repos format: $TUE_REPOS_DIR"
    mv -bv "$TUE_REPOS_DIR/https:"/* "$TUE_REPOS_DIR/https_"
    rmdir -v "$TUE_REPOS_DIR/https:"
fi

if [ -d $TUE_REPOS_DIR/github.com ]
then
    echo "Moving to new old repos format: $TUE_REPOS_DIR"
    mkdir -p "$TUE_REPOS_DIR/https_/github.com"
    mv -bv "$TUE_REPOS_DIR/github.com"/* "$TUE_REPOS_DIR/https_/github.com"
    rmdir -v "$TUE_REPOS_DIR/github.com"
fi
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

tue_cmd=$1
shift

# idiomatic parameter and option handling in sh
targets=""
while test $# -gt 0
do
    case "$1" in
        --debug) DEBUG=true
            ;;
        --branch*) BRANCH=`echo $1 | sed -e 's/^[^=]*=//g'`
            ;;
        --*) echo "unknown option $1"
            ;;
        *) targets="$targets $1"
            ;;
    esac
    shift
done

if [[ -z "${targets// }" ]] #If only whitespace
then
    # If no targets are provided, update all installed targets
    targets=`ls $TUE_INSTALL_INSTALLED_DIR`
fi

for target in $targets
do
    tue-install-debug "Main loop: installing $target"
    tue-install-target $target

    if [[ -d $TUE_INSTALL_TARGET_DIR/$target && "$tue_cmd" == "install" ]]
    then
        # Mark as installed
        tue-install-debug "[$target] marked as installed after a successful install"
        touch $TUE_INSTALL_INSTALLED_DIR/$target
    fi
done

# (Re-)generate setup file
mkdir -p $TUE_ENV_DIR/.env/setup
echo "# This file was auto-generated by tue-install. Do not change this file." > $TUE_ENV_DIR/.env/setup/target_setup.bash

mkdir -p $TUE_INSTALL_DEPENDENCIES_DIR
installed_targets=`ls $TUE_INSTALL_DEPENDENCIES_DIR`
TUE_SETUP_TARGETS=" "
for t in $installed_targets
do
    generate_setup_file $t
done

# Display infos
if [ -n "$TUE_INSTALL_INFOS" ]; then
    echo -e "\e[0;36m\nSome information you may have missed:\n\n$TUE_INSTALL_INFOS\033[0m"
fi

# Display warnings
if [ -n "$TUE_INSTALL_WARNINGS" ]; then
    echo -e "\033[33;5;1m\nOverview of warnings:\n\n$TUE_INSTALL_WARNINGS\033[0m"
fi

# Remove temp directories
rm -rf $TUE_INSTALL_STATE_DIR

# Installing all the ppa repo's, which are collected during install
if [ -n "$TUE_INSTALL_PPA" ]; then

    TUE_INSTALL_CURRENT_TARGET="PPA-ADD"

    echo -e "\nsudo add-apt-repository --yes $TUE_INSTALL_PPA" >> $INSTALL_DETAILS_FILE
    PPA_ADDED=""
    for ppa in $TUE_INSTALL_PPA
    do
        if [ -z "$(grep -h "^deb.*${ppa#ppa:}" /etc/apt/sources.list.d/* 2>&1)" ]
        then
            PPA_ADDED=true
            tue-install-info "Adding ppa: $ppa"
            sudo add-apt-repository --yes $ppa
        else
            tue-install-debug "$ppa is already added previously"
        fi
    done
    if [ -n "$PPA_ADDED" ]; then
        tue-install-debug "Updating apt-get"
        sudo apt-get update
    fi
fi

# Installing all system (apt-get) targets, which are collected during the install
if [ -n "$TUE_INSTALL_SYSTEMS" ]; then

    TUE_INSTALL_CURRENT_TARGET="APT-GET"

    echo -e "\nsudo apt-get install --assume-yes $TUE_INSTALL_SYSTEMS" >> $INSTALL_DETAILS_FILE

    tue-install-debug "tue-install-system-now $TUE_INSTALL_SYSTEMS"
    tue-install-system-now "$TUE_INSTALL_SYSTEMS"
fi

# Installing all python (pip) targets, which are collected during the install
if [ -n "$TUE_INSTALL_PIPS" ]; then

    TUE_INSTALL_CURRENT_TARGET="PIP"

    echo -e "\nyes | pip install --user $TUE_INSTALL_PIPS" >> $INSTALL_DETAILS_FILE

    pip_version=$(pip --version | awk '{print $2}')
    if version_gt "9" "$pip_version"; then
        tue-install-debug "pip not yet version >=9, but $pip_version"
        sudo -H pip install --upgrade pip
    else
        tue-install-debug "Already pip>=9\n"
    fi

    # Just install the packages because checking for installation is not faster
    echo -e "Going to run the following command:\n"
    echo -e "yes | pip install --user $TUE_INSTALL_PIPS\n"
    yes | pip install --user $TUE_INSTALL_PIPS
fi

# Installing all snap targets, which are collected during the install
if [ -n "$TUE_INSTALL_SNAPS" ]; then

    TUE_INSTALL_CURRENT_TARGET="SNAP"

    echo -e "\nyes | sudo snap install --classic $TUE_INSTALL_SNAPS" >> $INSTALL_DETAILS_FILE

    tue-install-system-now snapd

    snaps_to_install=""
    snaps_installed=$(snap list)
    for pkg in $TUE_INSTALL_SNAPS
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
            yes | sudo snap install --classic $pkg
        done
    fi
fi
