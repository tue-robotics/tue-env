#!/bin/bash

# Set ROS version
case $1 in
    kinetic)
        TUE_ROS_DISTRO=kinetic
        ;;
    melodic)
        TUE_ROS_DISTRO=melodic
        ;;
    *)
        echo "[bootstrap] Unknown ROS distribution $1"
        exit 1
        ;;
esac

# make sure git is installed
hash git 2> /dev/null || sudo apt-get install --assume-yes git

# Move old environments and installer
if [[ -d ~/.tue && -z "$CI" ]]
then
    FILES=$(find ~/.tue/user/envs -maxdepth 1 -type f)
    date_now=$(date +%F_%R)
    for env in $FILES
    do
        mv -f $(cat $env) $(cat $env).$date_now
    done
    mv -f ~/.tue ~/.tue.$date_now
fi

if [[ -z "$CI" ]]
then
    # Update installer
    git clone https://github.com/tue-robotics/tue-env.git ~/.tue
    git checkout ar13pit-patch/fix-names-scripts-targets
fi

# Source the installer commands
source ~/.tue/setup.bash
echo "[bootstrap] Loading tue-env"

# Create ros environment directory
mkdir -p ~/ros/$TUE_ROS_DISTRO

# Initialize ros environment directory
tue-env init ros-$TUE_ROS_DISTRO ~/ros/$TUE_ROS_DISTRO

# Set the correct ROS version for this environment
mkdir -p ~/ros/$TUE_ROS_DISTRO/.env/setup
echo "export TUE_ROS_DISTRO=$TUE_ROS_DISTRO" > ~/ros/$TUE_ROS_DISTRO/.env/setup/user_setup.bash

# Add loading of TU/e tools (tue-env, tue-get, etc) to bashrc
if ! grep -q '~/.tue/setup.bash' ~/.bashrc;
then
    echo '
# Load TU/e tools
source ~/.tue/setup.bash' >> ~/.bashrc
fi

# Set this environment as default
tue-env set-default ros-$TUE_ROS_DISTRO

# Activate the default environment
source ~/.tue/setup.bash
