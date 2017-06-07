#!/bin/bash

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
    *)
            # unknown option
    ;;
esac
done

echo "PACKAGE     = ${PACKAGE}"
echo "BRANCH      = ${BRANCH}"
echo "COMMIT PATH = ${COMMIT}"

# Determine docker tag if the same branch exists there
IMAGE_BRANCH_NAME=tuerobotics/tue-env:`echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g'`

# Set the default fallback branch to master
IMAGE_MASTER_NAME=tuerobotics/tue-env:master

# Run the docker image (user master as fallback)
docker run --detach --interactive --name tue-env $IMAGE_BRANCH_NAME || docker run --detach --interactive --name tue-env $IMAGE_MASTER_NAME 

# Install the pacakge
  - docker exec tue-env bash -c "export CI='true'; source /home/amigo/.bashrc; tue-get install ros-$PACKAGE"
  - docker exec tue-env bash -c "cd ~/ros/kinetic/system/src/$PACKAGE && git reset --hard $COMMIT"
