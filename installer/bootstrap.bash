#! /usr/bin/env bash

# Make sure git is installed
hash git 2> /dev/null || sudo apt-get install --assume-yes -qq git
# Make sure lsb-release is installed
hash lsb_release 2> /dev/null || sudo apt-get install --assume-yes -qq lsb-release

# Check if OS is Ubuntu
# shellcheck disable=SC1091
source /etc/lsb-release

if [ "$DISTRIB_ID" != "Ubuntu" ]
then
    echo "[bootstrap] Unsupported OS $DISTRIB_ID. Use Ubuntu."
    exit 1
fi

# Set ROS version
case $DISTRIB_RELEASE in
    "18.04")
        TUE_ROS_DISTRO=melodic
        ;;
    "20.04")
        TUE_ROS_DISTRO=noetic
        ;;
    *)
        echo "[tue-env](bootstrap) Ubuntu $DISTRIB_RELEASE is unsupported. Please use Ubuntu 20.04."
        exit 1
        ;;
esac

# Script variables
env_url="git@gitlab.com:avular/common-tools/package-manager/tue-env.git"
env_targets_url="git@gitlab.com:avular/common-tools/package-manager/tue-env-targets.git"
env_dir="$HOME/.tue"
workspace="ros-$TUE_ROS_DISTRO"
workspace_dir="$HOME/ros/$TUE_ROS_DISTRO"

# Move old environments and installer
if [ -d "$env_dir" ] && [ -z "$CI" ]
then
    FILES=$(find "$env_dir"/user/envs -maxdepth 1 -type f)
    date_now=$(date +%F_%R)
    for env in $FILES
    do
        mv -f "$(cat "$env")" "$(cat "$env")"."$date_now"
    done
    mv -f "$env_dir" "$env_dir"."$date_now"
fi

# If in CI with Docker, then clone tue-env with BRANCH when not testing a PR
if [ "$CI" == "true" ] && [ "$DOCKER" == "true" ]
then
    # Docker has a default value as false for PULL_REQUEST
    if [ "$PULL_REQUEST" == "false" ]
    then
        if [ -n "$COMMIT" ]
        then
            if [ -n "$BRANCH" ]
            then
                echo -e "[tue-env](bootstrap) Cloning tue-env repository with branch: $BRANCH at commit: $COMMIT"
                git clone -q --single-branch --branch "$BRANCH" "$env_url" "$env_dir"
            else
                echo -e "[tue-env](bootstrap) Cloning tue-env repository with default branch at commit: $COMMIT"
                git clone -q --single-branch "$env_url" "$env_dir"
            fi
            git -C "$env_dir" reset --hard "$COMMIT"
        else
            echo -e "[tue-env](bootstrap) Error! CI branch or commit is unset"
            return 1
        fi
    else
        echo -e "[tue-env](bootstrap) Testing Pull Request"
        [ -z "$REF_NAME" ] && { echo "Error! Environment variable REF_NAME is not set."; exit 1; }

        git clone -q --depth=10 "$env_url" "$env_dir"
        git -C "$env_dir" fetch origin "$REF_NAME"/"$PULL_REQUEST"/merge:PULLREQUEST || { echo "Error! Could not fetch refs"; exit 1; }
        git -C "$env_dir" checkout PULLREQUEST
    fi
else
    # Update installer
    echo -e "[tue-env](bootstrap) Cloning tue-env repository"
    git clone "$env_url" "$env_dir"
fi

# Source the installer commands
# No need to follow to a file which is already checked by CI
# shellcheck disable=SC1090
source "$env_dir"/setup.bash

# Create ros environment directory
mkdir -p "$workspace_dir"

# Initialize ros environment directory incl. targets
tue-env init "$workspace" "$workspace_dir" "$env_targets_url"

# Configure environment
tue-env config "$workspace" set "TUE_ROS_DISTRO" "$TUE_ROS_DISTRO"
tue-env config "$workspace" git-use-ssh
tue-env config "$workspace" github-use-https

# Add loading of TU/e tools (tue-env, tue-get, etc) to bashrc
# shellcheck disable=SC2088
if ! grep -q "$env_dir/setup.bash" ~/.bashrc;
then
    echo "
# Load TU/e tools
source $env_dir/setup.bash" >> ~/.bashrc
fi

# Set this environment as default
tue-env set-default "$workspace"

# Activate the default environment
# No need to follow to file which is already checked by CI
# shellcheck disable=SC1090
source "$env_dir"/setup.bash
