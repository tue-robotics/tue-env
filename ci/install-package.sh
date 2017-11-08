#!/bin/bash
set -o errexit #Stop on errors

# Standard argument parsing, example: install-package --branch=master --package=ros_robot
for i in "$@"
do
case $i in
    -p=*|--package=*)
    PACKAGE="${i#*=}"
    shift 
    ;;
    -b=*|--branch=*)
    BRANCH="${i#*=}"
    shift 
    ;;
    -c=*|--commit=*)
    COMMIT="${i#*=}"
    shift 
    ;;
    -r=*|--pullrequest=*)
    PULL_REQUEST="${i#*=}"
    shift
    ;;    
    *)
            # unknown option
    ;;
esac
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

Optionally fix your compilation errors and rerun only the last command
\e[0m"

# Name of the docker image
IMAGE_NAME=tuerobotics/tue-env
# Determine docker tag if the same branch exists there
BRANCH_TAG=`echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g'`

# Set the default fallback branch to master
MASTER_TAG=master

# Remove any previously started containers if they exist (if not exist, still return true to let the script continue)
docker stop tue-env  &> /dev/null || true && docker rm tue-env &> /dev/null || true

# Pull the identical branch name from dockerhub if exist, use master as fallback
echo -e "\e[35m\e[1m Trying to fetch docker image: $IMAGE_NAME:$BRANCH_TAG \e[0m"
if ! docker pull $IMAGE_NAME:$BRANCH_TAG
then
    echo -e "\e[35m\e[1m No worries, we just test against the master branch: $IMAGE_NAME:$MASTER_TAG \e[0m"
    docker pull $IMAGE_NAME:$MASTER_TAG 
    BRANCH_TAG=master
fi

# Run the docker image
docker run --detach --interactive --name tue-env $IMAGE_NAME:$BRANCH_TAG

# Refresh the apt cache in the docker image
docker exec tue-env bash -c "sudo apt-get update -qq"

# Install the package
echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE \e[0m"
docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; tue-get install ros-$PACKAGE"

# Set the package to the right commit
echo -e "\e[35m\e[1m Reset package to this commit \e[0m"
if [[ $PULL_REQUEST == "false" ]]; 
then
    echo -e "\e[35m\e[1m cd ~/ros/kinetic/system/src/$PACKAGE && git reset --hard $COMMIT \e[0m"
    docker exec tue-env bash -c "cd ~/ros/kinetic/system/src/$PACKAGE && git reset --hard $COMMIT"
else
    echo -e "\e[35m\e[1m cd ~/ros/kinetic/system/src/$PACKAGE && git fetch origin pull/$PULL_REQUEST/head:PULLREQUEST && git checkout PULLREQUEST \e[0m"
    docker exec tue-env bash -c "cd ~/ros/kinetic/system/src/$PACKAGE && git fetch origin pull/$PULL_REQUEST/head:PULLREQUEST && git checkout PULLREQUEST"
fi
