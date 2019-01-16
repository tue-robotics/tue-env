#!/bin/bash

# Make sure git is installed
hash git 2> /dev/null || sudo apt-get install --assume-yes git
# Make sure lsb-release is installed
hash lsb_release 2> /dev/null || sudo apt-get install --assume-yes lsb-release

# Check if OS is Ubuntu
source /etc/lsb-release

if [ "$DISTRIB_ID" != "Ubuntu" ]
then
    echo "[bootstrap] Unsupported OS $DISTRIB_ID. Use Ubuntu."
    exit 1
fi

# Set ROS version
case $DISTRIB_RELEASE in
    "16.04")
        TUE_ROS_DISTRO=kinetic
        ;;
    "18.04")
        TUE_ROS_DISTRO=melodic
        ;;
    *)
        echo "[bootstrap] Ubuntu $DISTRIB_RELEASE is unsupported. Use either 16.04 or 18.04"
        exit 1
        ;;
esac

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
fi

# Source the installer commands
source ~/.tue/setup.bash

# Create ros environment directory
mkdir -p ~/ros/$TUE_ROS_DISTRO

# Initialize ros environment directory
tue-env init ros-$TUE_ROS_DISTRO ~/ros/$TUE_ROS_DISTRO https://github.com/tue-robotics/tue-env-targets

# Set the correct ROS version for this environment
mkdir -p ~/ros/$TUE_ROS_DISTRO/.env/setup
echo "export TUE_ROS_DISTRO=$TUE_ROS_DISTRO" > ~/ros/$TUE_ROS_DISTRO/.env/setup/user_setup.bash

# Download targets
git clone https://github.com/tue-robotics/tue-env-targets.git  ~/ros/$TUE_ROS_DISTRO/.env/targets
echo "export TUE_ENV_TARGETS_DIR=~/ros/$TUE_ROS_DISTRO/.env/targets" >> ~/ros/$TUE_ROS_DISTRO/.env/setup/user_setup.bash

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
