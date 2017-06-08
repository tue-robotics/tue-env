#!/bin/bash
set -o errexit #Stop on errors

for i in "$@"
do
case $i in
    -p=*|--package=*)
    PACKAGE="${i#*=}"
    shift # past argument=value
    ;;
    -b=*|--branch=*)
    BRANCH="${i#*=}"
    shift # past argument=value
    ;;
    -c=*|--commit=*)
    COMMIT="${i#*=}"
    shift # past argument=value
    ;;
    -r=*|--pullrequest=*)
    PULL_REQUEST="${i#*=}"
    shift # past argument=value
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

# Determine docker tag if the same branch exists there
IMAGE_BRANCH_NAME=tuerobotics/tue-env:`echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g'`

# Set the default fallback branch to master
IMAGE_MASTER_NAME=tuerobotics/tue-env:master

# Remove any previously started containers if they exist (if not exist, still return true to let the script continue)
docker stop tue-env || true && docker rm tue-env || true

# Run the docker image (user master as fallback)
echo -e "\e[35m\e[1m Trying to fetch docker image: $IMAGE_BRANCH_NAME \e[0m"
if ! docker run --detach --interactive --name tue-env $IMAGE_BRANCH_NAME
then
    echo -e "\e[35m\e[1m Not found, fall back to master branch: $IMAGE_MASTER_NAME \e[0m"
    docker run --detach --interactive --name tue-env $IMAGE_MASTER_NAME 
fi

# Install the package
echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE \e[0m"
docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; tue-get install ros-$PACKAGE"

# Set the package to the right commit
echo -e "\e[35m\e[1m Reset package to this commit \e[0m"
if [[ $PULL_REQUEST == "false" ]]; 
then
    docker exec tue-env bash -c "cd ~/ros/kinetic/system/src/$PACKAGE && git reset --hard $COMMIT"
else
    docker exec tue-env bash -c "cd ~/ros/kinetic/system/src/$PACKAGE && git fetch origin pull/$PULL_REQUEST/head:PULLREQUEST && git checkout PULLREQUEST"
fi
