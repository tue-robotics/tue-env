#! /usr/bin/env bash

# Make sure git is installed
hash git 2> /dev/null || sudo apt-get install --assume-yes git
# Make sure lsb-release is installed
hash lsb_release 2> /dev/null || sudo apt-get install --assume-yes lsb-release

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
    "16.04")
        TUE_ROS_DISTRO=kinetic
        ;;
    "18.04")
        TUE_ROS_DISTRO=melodic
        ;;
    *)
        echo "[tue-env](bootstrap) Ubuntu $DISTRIB_RELEASE is unsupported. Use either 16.04 or 18.04"
        exit 1
        ;;
esac

# Move old environments and installer
if [ -d ~/.tue ] && [ -z "$CI" ]
then
    FILES=$(find ~/.tue/user/envs -maxdepth 1 -type f)
    date_now=$(date +%F_%R)
    for env in $FILES
    do
        mv -f "$(cat "$env")" "$(cat "$env")"."$date_now"
    done
    mv -f ~/.tue ~/.tue."$date_now"
fi

# If in CI with Docker, then clone tue-env with BRANCH when not testing a PR
if [ "$CI" == "true" ] && [ "$DOCKER" == "true" ]
then
    # Docker has a default value as false for PULL_REQUEST
    if [ "$PULL_REQUEST" == "false" ]
    then
        # Docker has a default value as master for BRANCH
        if [ -n "$BRANCH" ] && [ -n "$COMMIT" ]
        then
            echo -e "[tue-env](bootstrap) Cloning tue-env repository with branch: $BRANCH at commit: $COMMIT"
            git clone -q --single-branch --branch "$BRANCH" https://github.com/tue-robotics/tue-env.git ~/.tue
            git -C ~/.tue reset --hard "$COMMIT"
        else
            echo -e "[tue-env](bootstrap) Error! CI branch or commit is unset"
            return 1
        fi
    else
        echo -e "[tue-env](bootstrap) Testing Pull Request"
        git clone -q --depth=10 https://github.com/tue-robotics/tue-env.git ~/.tue
        git -C ~/.tue fetch origin pull/"$PULL_REQUEST"/head:PULLREQUEST
        git -C ~/.tue checkout PULLREQUEST
    fi
else
    # Update installer
    echo -e "[tue-env](bootstrap) Cloning tue-env repository"
    git clone https://github.com/tue-robotics/tue-env.git ~/.tue
fi

# Source the installer commands
# No need to follow to a file which is already checked by CI
# shellcheck disable=SC1090
source ~/.tue/setup.bash

# Create ros environment directory
mkdir -p ~/ros/$TUE_ROS_DISTRO

# Initialize ros environment directory incl. targets
tue-env init ros-$TUE_ROS_DISTRO ~/ros/$TUE_ROS_DISTRO https://github.com/tue-robotics/tue-env-targets.git

# Set the correct ROS version for this environment
echo "export TUE_ROS_DISTRO=$TUE_ROS_DISTRO" >> ~/ros/$TUE_ROS_DISTRO/.env/setup/user_setup.bash

# Add loading of TU/e tools (tue-env, tue-get, etc) to bashrc
# shellcheck disable=SC2088
if ! grep -q '~/.tue/setup.bash' ~/.bashrc;
then
    echo '
# Load TU/e tools
source ~/.tue/setup.bash' >> ~/.bashrc
fi

# Set this environment as default
tue-env set-default ros-$TUE_ROS_DISTRO

# Activate the default environment
# No need to follow to file which is already checked by CI
# shellcheck disable=SC1090
source ~/.tue/setup.bash
