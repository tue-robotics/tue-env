#! /usr/bin/env bash
#
# Package installer (CI script)
# This script uses the Docker image of tue-env and installs the current git
# repository as a tue-env package using tue-install in the CI

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

# Standard argument parsing, example: install-package --branch=master --package=ros_robot
for i in "$@"
do
    case $i in
        -p=* | --package=* )
            PACKAGE="${i#*=}" ;;

        -b=* | --branch=* )
            BRANCH="${i#*=}" ;;

        -c=* | --commit=* )
            COMMIT="${i#*=}" ;;

        -r=* | --pullrequest=* )
            PULL_REQUEST="${i#*=}" ;;
        * )
            # unknown option
            echo -e "\e[35m\e[1m Unknown input argument '$i'. Check CI .yml file \e[0m"
            exit 1 ;;
    esac
    shift
done

echo -e "\e[35m\e[1m PACKAGE      = ${PACKAGE} \e[0m"
echo -e "\e[35m\e[1m BRANCH       = ${BRANCH} \e[0m"
echo -e "\e[35m\e[1m COMMIT       = ${COMMIT} \e[0m"
echo -e "\e[35m\e[1m PULL_REQUEST = ${PULL_REQUEST} \e[0m"

echo -e "\e[35m\e[1m
This build can be reproduced locally using the following commands:

tue-get install docker
~/.tue/ci/install-package.sh --package=${PACKAGE} --branch=${BRANCH} --commit=${COMMIT} --pullrequest=${PULL_REQUEST}
~/.tue/ci/build-package.sh --package=${PACKAGE}

Optionally fix your compilation errors and re-run only the last command
\e[0m"

# If packages is non-zero, this is a multi-package repo. In multi-package repo, check if this package needs CI.
# If a single-package repo, CI is always needed.
# shellcheck disable=SC2153
if [ -n "$PACKAGES" ] && ! echo "$PACKAGES" | grep -sqw "$PACKAGE"
then
    echo -e "\e[35m\e[1m No changes in this package, so no need to run CI \e[0m"
    exit 0
fi

# Name of the docker image
IMAGE_NAME=tuerobotics/tue-env
# Determine docker tag if the same branch exists there
BRANCH_TAG=$(echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')

# Set the default fallback branch to master
MASTER_TAG=master

# Remove any previously started containers if they exist (if not exist, still return true to let the script continue)
docker stop tue-env  &> /dev/null || true
docker rm tue-env &> /dev/null || true

# Pull the identical branch name from dockerhub if exist, use master as fallback
echo -e "\e[35m\e[1m Trying to fetch docker image: $IMAGE_NAME:$BRANCH_TAG \e[0m"
if ! docker pull "$IMAGE_NAME:$BRANCH_TAG"
then
    echo -e "\e[35m\e[1m No worries, we just test against the master branch: $IMAGE_NAME:$MASTER_TAG \e[0m"
    docker pull $IMAGE_NAME:$MASTER_TAG
    BRANCH_TAG=master
fi

# Run the docker image along with setting new environment variables
docker run --detach --interactive -e CI=true -e PACKAGE="$PACKAGE" -e BRANCH="$BRANCH" -e COMMIT="$COMMIT" -e PULL_REQUEST="$PULL_REQUEST" --name tue-env "$IMAGE_NAME:$BRANCH_TAG"

# Refresh the apt cache in the docker image
docker exec tue-env bash -c 'sudo apt-get update -qq'

# Use docker environment variables in all exec commands instead of script variables
# Catch the ROS_DISTRO of the docker container
ROS_DISTRO=$(docker exec tue-env bash -c 'source ~/.bashrc; echo "$ROS_DISTRO"')
echo -e "\e[35m\e[1m ROS_DISTRO = ${ROS_DISTRO}\e[0m"

# Install the package
echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE --test-depend --branch=$BRANCH\e[0m"
docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"$PACKAGE" --test-depend --branch="$BRANCH"'

# Set the package to the right commit
echo -e "\e[35m\e[1m Reset package to this commit \e[0m"
if [[ $PULL_REQUEST == "false" ]]
then
    echo -e "\e[35m\e[1m cd ~/ros/$ROS_DISTRO/system/src/$PACKAGE && git reset --hard $COMMIT \e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; cd ~/ros/"$ROS_DISTRO"/system/src/"$PACKAGE" && git reset --hard "$COMMIT"'
else
    echo -e "\e[35m\e[1m cd ~/ros/$ROS_DISTRO/system/src/$PACKAGE && git fetch origin pull/$PULL_REQUEST/head:PULLREQUEST && git checkout PULLREQUEST \e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; cd ~/ros/"$ROS_DISTRO"/system/src/"$PACKAGE" && git fetch origin pull/"$PULL_REQUEST"/head:PULLREQUEST && git checkout PULLREQUEST'
fi
